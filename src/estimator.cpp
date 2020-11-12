#include "estimator.h"

#include <ostream>
#include <fstream>

//int numSolveSLAM = 5;

using namespace myslam;

Estimator::Estimator() : f_manager{Rs}
{
    std::cout << "[ESTIMATOR] initialize. " << std::endl;
    ClearState(true);
}

void Estimator::setParameter()
{
    for (int i = 0; i < NUM_OF_CAM; i++){
        tic[i] = TIC[i];
        ric[i] = RIC[i];
        Quaterniond q_ic{RIC[i]};

        vPic[i] << TIC[i][0], TIC[i][1], TIC[i][2], q_ic.x(), q_ic.y(), q_ic.z(), q_ic.w();
    }
    f_manager.setRic(ric);
    project_sqrt_info_ = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    td = TD;
}

//  BASTIAN_M : some time the pointer will not be nullptr when initialize,
//          which will lead to segmentation error.
//          so I fix the error by adding a flag for initialize.
void Estimator::ClearState(bool bInit)
{
    for (int i = 0; i < WINDOW_SIZE + 1; i++){
        Rs[i].setIdentity();
        //vPose[i].setZero(); vPose[i][7] = 1.0;
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();
        if(!bInit){
            if (pre_integrations[i] != nullptr){
                delete pre_integrations[i];
            }
        }
        pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i++){
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
        Quaterniond q_ic{ric[i]};
        vPic[i] = Vec7::Zero(); vPic[i][6] = 1.0;
    }

    if(!bInit){
    for (auto &it : all_image_frame){
        if (it.second.pre_integration != nullptr){
            delete it.second.pre_integration;
            it.second.pre_integration = nullptr;
        }
    }
    }

    solver_flag = INITIAL;
    first_imu = false,
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();
    td = TD;

    if(!bInit){
        if (tmp_pre_integration != nullptr)
            delete tmp_pre_integration;
    }
    tmp_pre_integration = nullptr;

    last_marginalization_parameter_blocks.clear();

    f_manager.ClearState();

    failure_occur = 0;
    relocalization_info = 0;

    drift_correct_r = Matrix3d::Identity();
    drift_correct_t = Vector3d::Zero();
}

void Estimator::processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu){
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (!pre_integrations[frame_count]){
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }

    if (frame_count != 0){
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        //if(solver_flag != NON_LINEAR)
        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void Estimator::ProcessImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double header)
{
    if (f_manager.AddFeatureCheckParallax(frame_count, image, td))
        marginalization_flag = MARGIN_OLD;
    else
        marginalization_flag = MARGIN_SECOND_NEW;

    Headers[frame_count] = header;

    ImageFrame imageframe(image, header);
    imageframe.pre_integration = tmp_pre_integration;
    all_image_frame.insert(make_pair(header, imageframe));

    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    if (ESTIMATE_EXTRINSIC == 2){
        std::cout << "calibrating extrinsic param, rotation movement is needed" << endl;
        if (frame_count != 0){
            vector<pair<Vector3d, Vector3d>> corres = f_manager.GetCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric)){
                // ROS_WARN("initial extrinsic rotation calib success");
                // ROS_WARN_STREAM("initial extrinsic rotation: " << endl
                                                            //    << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                Quaterniond q_ic{calib_ric};
                vPic[0] << TIC[0][0], TIC[0][1], TIC[0][2], q_ic.x(), q_ic.y(), q_ic.z(), q_ic.w();

                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    if (solver_flag == INITIAL){
        if (frame_count == WINDOW_SIZE){
            bool result = false;
            if (ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1){
                // cout << "1 InitialStructure" << endl;
                result = InitialStructure();
                initial_timestamp = header;
            }
            if (result){
                solver_flag = NON_LINEAR;
                SolveOdometry();
                slideWindow();
                f_manager.removeFailures();
                std::cout << "==> Initialization finish!" << endl;
                last_R = Rs[WINDOW_SIZE];
                last_P = Ps[WINDOW_SIZE];
                last_R0 = Rs[0];
                last_P0 = Ps[0];
            } else {
                slideWindow();
            }
        } else {
            frame_count++;
        }
    } else {
        SolveOdometry();

        if (failureDetection()){
            failure_occur = 1;
            ClearState();
            setParameter();
            return;
        }

        //TicToc t_margin;
        slideWindow();
        f_manager.removeFailures();

        // prepare output of VINS visualization
        if(B_SAVE_CLOUD){
            key_poses.clear();
            for (int i = 0; i <= WINDOW_SIZE; i++)
                key_poses.push_back(Ps[i]);
        }

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
    }
    //UpdateCurrentPointcloud();
}

Eigen::Matrix<double, 3, 4> Estimator::GetCurrentCameraPose()
{
    Eigen::Matrix<double, 3, 4> Twc;
    Twc.setZero();

    Matrix3d rotate = Rs[WINDOW_SIZE] * ric[0];    //  Rs is Rwi
    Vector3d pose = Rs[WINDOW_SIZE] * tic[0] + Ps[WINDOW_SIZE];

    Twc.block(0,0,3,3) = rotate;
    Twc.block(0,3,3,1) = pose;
    return Twc;
}



bool Estimator::InitialStructure()
{
    TicToc t_sfm;
    //check imu observibility
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            //cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        //ROS_WARN("IMU variation %f!", var);
        if (var < 0.25)
        {
            // ROS_INFO("IMU excitation not enouth!");
            return false;
        }
    }
    // global sfm
    Quaterniond Q[frame_count + 1];
    Vector3d T[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f;
    for (auto &it_per_id : f_manager.feature)
    {
        int imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    }
    Matrix3d relative_R;
    Vector3d relative_T;
    int l;
    if (!relativePose(relative_R, relative_T, l))
    {
        cout << "Not enough features or parallax; Move device around" << endl;
        return false;
    }
    GlobalSFM sfm;
    if (!sfm.construct(frame_count + 1, Q, T, l,
                       relative_R, relative_T,
                       sfm_f, sfm_tracked_points))
    {
        cout << "global SFM failed!" << endl;
        marginalization_flag = MARGIN_OLD;
        return false;
    }

    //solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin();
    for (int i = 0; frame_it != all_image_frame.end(); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        if ((frame_it->first) == Headers[i])
        {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if ((frame_it->first) > Headers[i])
        {
            i++;
        }
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = -R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second)
            {
                it = sfm_tracked_points.find(feature_id);
                if (it != sfm_tracked_points.end())
                {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        if (pts_3_vector.size() < 6)
        {
            cout << "Not enough points for solve pnp pts_3_vector size " << pts_3_vector.size() << endl;
            return false;
        }
        if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            cout << " solve pnp fail!" << endl;
            return false;
        }
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp, tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
    }
    if (visualInitialAlign())
        return true;
    else
    {
        cout << "misalign visual structure with IMU" << endl;
        return false;
    }
}

bool Estimator::visualInitialAlign()
{
    TicToc t_g;
    VectorXd x;

    //solve scale
    if (!VisualIMUAlignment(all_image_frame, Bgs, g, x)){
        return false;
    }

    // change state
    for (int i = 0; i <= frame_count; i++){
        Matrix3d Ri = all_image_frame[Headers[i]].R;
        Vector3d Pi = all_image_frame[Headers[i]].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i]].is_key_frame = true;
    }

    f_manager.ClearDepth();

    //triangulat on cam pose , no tic
    Vector3d TIC_TMP[NUM_OF_CAM];
    for (int i = 0; i < NUM_OF_CAM; i++)
        TIC_TMP[i].setZero();
    ric[0] = RIC[0];
    f_manager.setRic(ric);
    f_manager.triangulate(Ps, &(TIC_TMP[0]), &(RIC[0]));

    double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++){
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++){
        if (frame_i->second.is_key_frame){
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }
    for (auto &it_per_id : f_manager.feature){
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth *= s;
    }

    Matrix3d R0 = Utility::g2R(g);
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    g = R0 * g;
    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++){
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }

    return true;
}

bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager.GetCorresponding(i, WINDOW_SIZE);
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;
            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            if (average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                //ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true;
            }
        }
    }
    return false;
}

void Estimator::SolveOdometry()
{
    if (frame_count < WINDOW_SIZE)
        return;
    if (solver_flag == NON_LINEAR)
    {
        TicToc t_tri;
        f_manager.triangulate(Ps, tic, ric);
        //cout << "triangulation costs : " << t_tri.toc() << endl;
        BackendOptimization();
    }
}

bool Estimator::failureDetection()
{
    if (f_manager.last_track_num < 2)
    {
        //ROS_INFO(" little feature %d", f_manager.last_track_num);
        return true;
    }
    if (Bas[WINDOW_SIZE].norm() > 2.5)
    {
        //ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
        //ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)
    {
        //ROS_INFO(" big translation");
        return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1)
    {
        //ROS_INFO(" big z translation");
        return true;
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50)
    {
        //ROS_INFO(" big delta_angle ");
        return true;
    }
    return false;
}

void Estimator::MargOldFrame()
{
    backend::LossFunction *lossfunction;
    lossfunction = new backend::CauchyLoss(1.0);

    // step1. build problem
    backend::Problem problem(backend::Problem::ProblemType::SLAM_PROBLEM);
    vector<shared_ptr<backend::VertexPose>> vertexCams_vec;
    vector<shared_ptr<backend::VertexSpeedBias>> vertexVB_vec;
    int pose_dim = 0;

    // add the externion parameters to the graph, body camera transformation, camera calibrations, etc.
    // as it is frequency used, put it in the first place.
    shared_ptr<backend::VertexPose> vertexExt(new backend::VertexPose());
    {
        Eigen::VectorXd pose = vPic[0];
        //pose << para_Ex_Pose[0][0], para_Ex_Pose[0][1], para_Ex_Pose[0][2], para_Ex_Pose[0][3], para_Ex_Pose[0][4], para_Ex_Pose[0][5], para_Ex_Pose[0][6];
        vertexExt->SetParameters(pose);
        problem.AddVertex(vertexExt);
        pose_dim += vertexExt->LocalDimension();
    }

    // add the camera pose vertexs
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        shared_ptr<backend::VertexPose> vertexCam(new backend::VertexPose());
        Eigen::VectorXd pose(7);
        Quaterniond q_init(Rs[i]);
        pose << Ps[i][0], Ps[i][1], Ps[i][2], q_init.x(), q_init.y(), q_init.z(), q_init.w();
        vertexCam->SetParameters(pose);
        vertexCams_vec.push_back(vertexCam);
        problem.AddVertex(vertexCam);
        pose_dim += vertexCam->LocalDimension();

        shared_ptr<backend::VertexSpeedBias> vertexVB(new backend::VertexSpeedBias());
        Eigen::VectorXd vb(9);
        vb << Vs[i][0], Vs[i][1], Vs[i][2],
              Bas[i][0], Bas[i][1], Bas[i][2],
              Bgs[i][0], Bgs[i][1], Bgs[i][2];
        vertexVB->SetParameters(vb);
        vertexVB_vec.push_back(vertexVB);
        problem.AddVertex(vertexVB);
        pose_dim += vertexVB->LocalDimension();
    }

    // add IMU preintegration edges
    {
        if (pre_integrations[1]->sum_dt < 10.0)
        {
            std::shared_ptr<backend::EdgeImu> imuEdge(new backend::EdgeImu(pre_integrations[1]));
            std::vector<std::shared_ptr<backend::Vertex>> edge_vertex;
            edge_vertex.push_back(vertexCams_vec[0]);
            edge_vertex.push_back(vertexVB_vec[0]);
            edge_vertex.push_back(vertexCams_vec[1]);
            edge_vertex.push_back(vertexVB_vec[1]);
            imuEdge->SetVertex(edge_vertex);
            problem.AddEdge(imuEdge);
        }
    }

    // Visual Factor
    {
        int feature_index = -1;
        // for all the viewed features
        for (auto &it_per_id : f_manager.feature)
        {
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;

            ++feature_index;

            int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
            if (imu_i != 0)
                continue;

            Vector3d pts_i = it_per_id.feature_per_frame[0].point;

            shared_ptr<backend::VertexInverseDepth> verterxPoint(new backend::VertexInverseDepth());
            VecX inv_d(1);
            inv_d << vInverseDepth[feature_index];
            verterxPoint->SetParameters(inv_d);
            problem.AddVertex(verterxPoint);

            // for all its observations -> each has a reprojection error w.r.t the first observation
            for (auto &it_per_frame : it_per_id.feature_per_frame)
            {
                imu_j++;
                if (imu_i == imu_j)
                    continue;

                Vector3d pts_j = it_per_frame.point;

                std::shared_ptr<backend::EdgeReprojection> edge(new backend::EdgeReprojection(pts_i, pts_j));
                std::vector<std::shared_ptr<backend::Vertex>> edge_vertex;
                edge_vertex.push_back(verterxPoint);
                edge_vertex.push_back(vertexCams_vec[imu_i]);
                edge_vertex.push_back(vertexCams_vec[imu_j]);
                edge_vertex.push_back(vertexExt);

                edge->SetVertex(edge_vertex);
                edge->SetInformation(project_sqrt_info_.transpose() * project_sqrt_info_);

                edge->SetLossFunction(lossfunction);
                problem.AddEdge(edge);
            }
        }
    }

    // prior observations (the vertex and edges that are marginalized)
    {
        // If it is already exist
        if (Hprior_.rows() > 0)
        {
            problem.SetHessianPrior(Hprior_); // set the prior matrix
            problem.SetbPrior(bprior_);
            problem.SetErrPrior(errprior_);
            problem.SetJtPrior(Jprior_inv_);
            problem.ExtendHessiansPriorSize(15); // extend the prior matrix for new marginalization
        } else {
            Hprior_ = MatXX(pose_dim, pose_dim);
            Hprior_.setZero();
            bprior_ = VecX(pose_dim);
            bprior_.setZero();
            problem.SetHessianPrior(Hprior_); // set the initial prior matrix
            problem.SetbPrior(bprior_);
        }
    }

    // build the marginalization elements
    std::vector<std::shared_ptr<backend::Vertex>> marg_vertex;
    marg_vertex.push_back(vertexCams_vec[0]);
    marg_vertex.push_back(vertexVB_vec[0]);

    // marginalize the old camera
    problem.Marginalize(marg_vertex, pose_dim);

    // save marginalized points
    SaveMarginalizedFrameHostedPoints(problem);

    Hprior_ = problem.GetHessianPrior();
    bprior_ = problem.GetbPrior();
    errprior_ = problem.GetErrPrior();
    Jprior_inv_ = problem.GetJtPrior();
}


void Estimator::MargNewFrame()
{

    // step1. build problem
    backend::Problem problem(backend::Problem::ProblemType::SLAM_PROBLEM);
    vector<shared_ptr<backend::VertexPose>> vertexCams_vec;
    vector<shared_ptr<backend::VertexSpeedBias>> vertexVB_vec;
    //    vector<backend::Point3d> points;
    int pose_dim = 0;

    // add the externion parameters to the graph, body camera transformation, camera calibrations, etc.
    // as it is frequency used, put it in the first place.
    shared_ptr<backend::VertexPose> vertexExt(new backend::VertexPose());
    {
        Eigen::VectorXd pose = vPic[0];
        //pose << para_Ex_Pose[0][0], para_Ex_Pose[0][1], para_Ex_Pose[0][2], para_Ex_Pose[0][3], para_Ex_Pose[0][4], para_Ex_Pose[0][5], para_Ex_Pose[0][6];
        vertexExt->SetParameters(pose);
        problem.AddVertex(vertexExt);
        pose_dim += vertexExt->LocalDimension();
    }

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        shared_ptr<backend::VertexPose> vertexCam(new backend::VertexPose());
        Eigen::VectorXd pose(7);
        Quaterniond q_init(Rs[i]);
        pose << Ps[i][0], Ps[i][1], Ps[i][2], q_init.x(), q_init.y(), q_init.z(), q_init.w();
        //pose << para_Pose[i][0], para_Pose[i][1], para_Pose[i][2], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5], para_Pose[i][6];
        vertexCam->SetParameters(pose);
        vertexCams_vec.push_back(vertexCam);
        problem.AddVertex(vertexCam);
        pose_dim += vertexCam->LocalDimension();

        shared_ptr<backend::VertexSpeedBias> vertexVB(new backend::VertexSpeedBias());
        Eigen::VectorXd vb(9);
        vb << Vs[i][0], Vs[i][1], Vs[i][2],
              Bas[i][0], Bas[i][1], Bas[i][2],
              Bgs[i][0], Bgs[i][1], Bgs[i][2];
        vertexVB->SetParameters(vb);
        vertexVB_vec.push_back(vertexVB);
        problem.AddVertex(vertexVB);
        pose_dim += vertexVB->LocalDimension();
    }

    // prior
    {
        // already got Prior
        if (Hprior_.rows() > 0)
        {
            problem.SetHessianPrior(Hprior_); // tell the problem
            problem.SetbPrior(bprior_);
            problem.SetErrPrior(errprior_);
            problem.SetJtPrior(Jprior_inv_);

            problem.ExtendHessiansPriorSize(15); // extand its dimension
        }
        else
        {
            Hprior_ = MatXX(pose_dim, pose_dim);
            Hprior_.setZero();
            bprior_ = VecX(pose_dim);
            bprior_.setZero();
        }
    }

    std::vector<std::shared_ptr<backend::Vertex>> marg_vertex;
    // marginalized the second last frame
    marg_vertex.push_back(vertexCams_vec[WINDOW_SIZE - 1]);
    marg_vertex.push_back(vertexVB_vec[WINDOW_SIZE - 1]);

    problem.Marginalize(marg_vertex, pose_dim);

    Hprior_ = problem.GetHessianPrior();
    bprior_ = problem.GetbPrior();
    errprior_ = problem.GetErrPrior();
    Jprior_inv_ = problem.GetJtPrior();
}

void Estimator::ProblemSolve()
{
    vInverseDepth = f_manager.GetInverseDepthVector();

    backend::LossFunction *lossfunction;
    lossfunction = new backend::CauchyLoss(1.0);
    //    lossfunction = new backend::TukeyLoss(1.0);

    // step1. build the problem
    backend::Problem problem(backend::Problem::ProblemType::SLAM_PROBLEM);
    vector<shared_ptr<backend::VertexPose>> vertexCams_vec;
    vector<shared_ptr<backend::VertexSpeedBias>> vertexVB_vec;
    int pose_dim = 0;

    // add the externion parameters to the graph, body camera transformation, camera calibrations, etc.
    // as it is frequency used, put it in the first place.
    shared_ptr<backend::VertexPose> vertexExt(new backend::VertexPose());
    {
        Eigen::VectorXd pose = vPic[0];
        vertexExt->SetParameters(pose);
        if (!ESTIMATE_EXTRINSIC){
            vertexExt->SetFixed();
        }
        problem.AddVertex(vertexExt);
        pose_dim += vertexExt->LocalDimension();
    }

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        shared_ptr<backend::VertexPose> vertexCam(new backend::VertexPose());
        Quaterniond q_init(Rs[i]);
        Eigen::VectorXd pose(7);
        pose << Ps[i][0], Ps[i][1], Ps[i][2], q_init.x(), q_init.y(), q_init.z(), q_init.w();
        vertexCam->SetParameters(pose);
        vertexCams_vec.push_back(vertexCam);
        problem.AddVertex(vertexCam);
        pose_dim += vertexCam->LocalDimension();

        shared_ptr<backend::VertexSpeedBias> vertexVB(new backend::VertexSpeedBias());
        Eigen::VectorXd vb(9);
        vb << Vs[i][0], Vs[i][1], Vs[i][2],
              Bas[i][0], Bas[i][1], Bas[i][2],
              Bgs[i][0], Bgs[i][1], Bgs[i][2];
        vertexVB->SetParameters(vb);
        vertexVB_vec.push_back(vertexVB);
        problem.AddVertex(vertexVB);
        pose_dim += vertexVB->LocalDimension();
    }


    // IMU
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0)
            continue;

        std::shared_ptr<backend::EdgeImu> imuEdge(new backend::EdgeImu(pre_integrations[j]));
        std::vector<std::shared_ptr<backend::Vertex>> edge_vertex;
        edge_vertex.push_back(vertexCams_vec[i]);
        edge_vertex.push_back(vertexVB_vec[i]);
        edge_vertex.push_back(vertexCams_vec[j]);
        edge_vertex.push_back(vertexVB_vec[j]);
        imuEdge->SetVertex(edge_vertex);
        problem.AddEdge(imuEdge);
    }

    // Visual Factor
    vector<shared_ptr<backend::VertexInverseDepth>> vertexPt_vec;
    {
        int feature_index = -1;
        // for all the features
        for (auto &it_per_id : f_manager.feature)
        {
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;

            ++feature_index;

            int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point;

            shared_ptr<backend::VertexInverseDepth> verterxPoint(new backend::VertexInverseDepth());
            VecX inv_d(1);
            inv_d << vInverseDepth[feature_index];
            verterxPoint->SetParameters(inv_d);
            problem.AddVertex(verterxPoint);
            vertexPt_vec.push_back(verterxPoint);

            // for all its observations
            for (auto &it_per_frame : it_per_id.feature_per_frame)
            {
                imu_j++;
                if (imu_i == imu_j)
                    continue;

                Vector3d pts_j = it_per_frame.point;

                std::shared_ptr<backend::EdgeReprojection> edge(new backend::EdgeReprojection(pts_i, pts_j));
                std::vector<std::shared_ptr<backend::Vertex>> edge_vertex;
                edge_vertex.push_back(verterxPoint);
                edge_vertex.push_back(vertexCams_vec[imu_i]);
                edge_vertex.push_back(vertexCams_vec[imu_j]);
                edge_vertex.push_back(vertexExt);

                edge->SetVertex(edge_vertex);
                edge->SetInformation(project_sqrt_info_.transpose() * project_sqrt_info_);

                edge->SetLossFunction(lossfunction);
                problem.AddEdge(edge);
            }
        }
    }

    // prior process
    {
        // already got one
        if (Hprior_.rows() > 0)
        {
            // set patermeters to be 0. TODO:: should be done in solver
            //            Hprior_.block(0,0,6,Hprior_.cols()).setZero();
            //            Hprior_.block(0,0,Hprior_.rows(),6).setZero();

            problem.SetHessianPrior(Hprior_); // tell the problem
            problem.SetbPrior(bprior_);
            problem.SetErrPrior(errprior_);
            problem.SetJtPrior(Jprior_inv_);
            problem.ExtendHessiansPriorSize(15); // extand the hessian prior
        }
    }

    problem.Solve(NUM_ITERATIONS);

    // update bprior_,  Hprior_ do not need update
    if (Hprior_.rows() > 0)
    {
        if(verbose){
            std::cout << "----------- update bprior -------------\n";
            std::cout << "             before: " << bprior_.norm() << std::endl;
            std::cout << "                     " << errprior_.norm() << std::endl;
        }
        bprior_ = problem.GetbPrior();
        errprior_ = problem.GetErrPrior();
        if(verbose){
            std::cout << "             after: " << bprior_.norm() << std::endl;
            std::cout << "                    " << errprior_.norm() << std::endl;
        }
    }

    // BASTIAN_M : directly update the vectors, instead of call double 2 vector later.
    // for the optimized variables : double2vector() of the original project
    Eigen::Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Eigen::Vector3d origin_P0 = Ps[0];

    if (failure_occur){
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }
    VecX vPoseCam0 = vertexCams_vec[0]->Parameters();
    Eigen::Matrix3d mRotCam0 = Quaterniond(vPoseCam0[6],vPoseCam0[3],vPoseCam0[4],vPoseCam0[5]).toRotationMatrix();
    Vector3d origin_R00 = Utility::R2ypr(mRotCam0);
    double y_diff = origin_R0.x() - origin_R00.x();

    // as the optimization may change the pose of all the frames
    // if the first frames pose changed, this will lead to the system random walk behaviour
    // to solve this, we calculate the difference between the frist frame pose before and
    // after the optimization, then propragate it to all other window frames
    //      the system has one rotation DOF and three poisition DOF
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
    if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
    {
        //ROS_DEBUG("euler singular point!");
        rot_diff = Rs[0] * mRotCam0.transpose();
    }
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        VecX vPoseCam_i = vertexCams_vec[i]->Parameters();
        Rs[i] = rot_diff * Quaterniond(vPoseCam_i[6], vPoseCam_i[3],
                   vPoseCam_i[4], vPoseCam_i[5]).normalized().toRotationMatrix();

        Ps[i] = rot_diff * Vector3d(vPoseCam_i[0] - vPoseCam0[0],
                                    vPoseCam_i[1] - vPoseCam0[1],
                                    vPoseCam_i[2] - vPoseCam0[2]) +
                origin_P0;

        VecX vSpeedBias_i = vertexVB_vec[i]->Parameters();
        Vs[i] = rot_diff * Vector3d(vSpeedBias_i[0],
                                    vSpeedBias_i[1],
                                    vSpeedBias_i[2]);

        Bas[i] = Vector3d(vSpeedBias_i[3],
                          vSpeedBias_i[4],
                          vSpeedBias_i[5]);

        Bgs[i] = Vector3d(vSpeedBias_i[6],
                          vSpeedBias_i[7],
                          vSpeedBias_i[8]);
    }

    if (ESTIMATE_EXTRINSIC){
        VecX vExterCali = vertexExt->Parameters();
        tic[0] = Vector3d(vExterCali[0], vExterCali[1], vExterCali[2]);
        ric[0] = Quaterniond(vExterCali[6],vExterCali[3],vExterCali[4],vExterCali[5]).toRotationMatrix();
        vPic[0] << vExterCali[0], vExterCali[1], vExterCali[2], vExterCali[3], vExterCali[4], vExterCali[5], vExterCali[6];
    }

    int f_count = f_manager.GetFeatureCount();
    VectorXd vInvDepToSet(f_count);
    for (size_t i = 0; i < f_count; ++i){
        VecX f = vertexPt_vec[i]->Parameters();
        vInvDepToSet(i) = f[0];
        vInverseDepth(i) = f[0];
    }
    f_manager.SetDepth(vInvDepToSet);

    // BASTIAN_M: td is never estiamted
    //if (ESTIMATE_TD)
    //    td = para_Td[0][0];

    // relative info between two loop frame
    // adjustment while loop detected
    // TODO loop closure
    //if (relocalization_info){}

}

void Estimator::BackendOptimization()
{
    //TicToc t_solver;
    // build the solver
    ProblemSolve();
    //printf(" opt - whole time for solver: %f", t_solver.toc());

    // maintain marg
    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD){
        MargOldFrame();
    } else {
        // if have prior
        if (Hprior_.rows() > 0) {
            MargNewFrame();
        }
    }
    //printf(" opt - marginalization: %f\n", t_whole_marginalization.toc());
}


void Estimator::slideWindow()
{
    //TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD)
    {
        double t_0 = Headers[0];
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE){
            for (int i = 0; i < WINDOW_SIZE; i++){
                Rs[i].swap(Rs[i + 1]);
                std::swap(pre_integrations[i], pre_integrations[i + 1]);
                dt_buf[i].swap(dt_buf[i + 1]);
                linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);
                Headers[i] = Headers[i + 1];
                Ps[i].swap(Ps[i + 1]);
                Vs[i].swap(Vs[i + 1]);
                Bas[i].swap(Bas[i + 1]);
                Bgs[i].swap(Bgs[i + 1]);
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
            Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
            Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            if (true || solver_flag == INITIAL)
            {
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                it_0->second.pre_integration = nullptr;

                for (map<double, ImageFrame>::iterator it = all_image_frame.begin(); it != it_0; ++it)
                {
                    if (it->second.pre_integration)
                        delete it->second.pre_integration;
                    it->second.pre_integration = NULL;
                }

                all_image_frame.erase(all_image_frame.begin(), it_0);
                all_image_frame.erase(t_0);
            }
            slideWindowOld();
        }
    } else {
        if (frame_count == WINDOW_SIZE)
        {
            for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
            {
                double tmp_dt = dt_buf[frame_count][i];
                Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                dt_buf[frame_count - 1].push_back(tmp_dt);
                linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
            }

            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Vs[frame_count - 1] = Vs[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];
            Bas[frame_count - 1] = Bas[frame_count];
            Bgs[frame_count - 1] = Bgs[frame_count];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            slideWindowNew();
        }
    }
}

// real marginalization is removed in solve_ceres()
void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(frame_count);
}

// real marginalization is removed in solve_ceres()
void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        f_manager.RemoveBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.RemoveBack();
}


void Estimator::SaveMarginalizedFrameHostedPoints(backend::Problem &problem)
{
    if(B_SAVE_CLOUD){
        margin_cloud_cloud.push_back(problem.marginalized_pts);
        int cloud_size = margin_cloud_cloud.size();
        if(verbose){
            std::cout << "There are " << cloud_size << " cloud. " << std::endl;
            std::cout << "There are " << margin_cloud_cloud[cloud_size-1].size() << " points been marginalized. " << std::endl;
        }
        point_cloud.clear();
        point_cloud.assign(problem.current_pts.begin(), problem.current_pts.end());
    }
}

void Estimator::UpdateCurrentPointcloud()
{
    if(B_SAVE_CLOUD){
        point_cloud.clear();

        Qd qic(vPic[0][6], vPic[0][3], vPic[0][4], vPic[0][5]);
        Vector3d tic(vPic[0][0], vPic[0][1], vPic[0][2]);

        Matrix3d Qi = Rs[WINDOW_SIZE-1];
        Vector3d Pi = Ps[WINDOW_SIZE-1];

        for (auto &it_per_id : f_manager.feature){
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if(!(it_per_id.used_num >= 2))
                continue;
            if(it_per_id.estimated_depth < 0)
                continue;
            Vector3d pts_camera_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Vector3d pts_imu_i = qic * pts_camera_i + tic;
            Vec3 pts_w = Qi * pts_imu_i + Pi;

            point_cloud.push_back(pts_w);
        }
    }
}
