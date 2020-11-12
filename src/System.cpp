#include "System.h"

#include <pangolin/pangolin.h>

using namespace std;
using namespace cv;
using namespace pangolin;

namespace BASTIAN{

System::System(std::string sConfig_file_, bool b_savepose_)
{
    std::cout << "==> [SYSTEM] initialize system." << std::endl;
    bStart_backend = true;
    b_savepose = b_savepose_;

    // set the configuration file
    std::string sConfig_file = sConfig_file_;// + "euroc_config.yaml";
    std::cout << "==> [SYSTEM] set the configuration from file : " << sConfig_file << endl;
    // read parameters into global variables
    readParameters(sConfig_file);
    trackerData[0].ReadIntrinsicParameter(sConfig_file);
    estimator.setParameter();

    // make files for output the pose results
    if(b_savepose){
        ofs_pose.open("./pose_output.txt",fstream::app | fstream::out);
        if(!ofs_pose.is_open()){
            std::cerr << "ofs_pose is not open" << endl;
        }
    }

    // the backend threshold will be opened outside the system
    // thread thd_RunBackend(&System::process,this);
    // thd_RunBackend.detach();

    std::cout << "==> [SYSTEM] system initilize done." << endl;
}

System::~System()
{
    bStart_backend = false;
    pangolin::QuitAll();

    m_buf.lock();
    while (!feature_buf.empty())
        feature_buf.pop();
    while (!imu_buf.empty())
        imu_buf.pop();
    m_buf.unlock();

    m_estimator.lock();
    estimator.ClearState();
    m_estimator.unlock();

    if(b_savepose){
        ofs_pose.close();
    }
}

void System::PubImageData(double dStampSec, cv::Mat &img)
{
    // skip the first detected feature, which doesn't contain optical flow speed
    if (!init_feature){
        init_feature = 1;
        return;
    }

    // first image that will enter the system
    if (first_image_flag){
        first_image_flag = false;
        first_image_time = dStampSec;
        last_image_time = dStampSec;
        return;
    }

    // detect unstable camera stream
    //   -- that receive the image to late or early image
    if (dStampSec - last_image_time > 1.0 || dStampSec < last_image_time)
    {
        std::cerr << "[UNSTABLE] image discontinue! reset the feature tracker!" << std::endl;
        first_image_flag = true;
        last_image_time = 0;
        pub_count = 1;
        return;
    }
    //std::cout << "receive Image" << std::endl;

    last_image_time = dStampSec;

    bool bPublish = false;
    // frequency control
    if (round(1.0 * pub_count / (dStampSec - first_image_time)) <= FREQ){
        bPublish = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (dStampSec - first_image_time) - FREQ) < 0.01 * FREQ){
            first_image_time = dStampSec;
            pub_count = 0;
        }
    }

    trackerData[0].ReadImage(img, dStampSec, bPublish);
    trackerData[0].UpdateIdMono();
    //for (unsigned int i = 0;; i++){
    //    bool completed = false;
    //    completed |= trackerData[0].updateID(i);
    //    if (!completed)
    //        break;
    //}

    if (bPublish){
        pub_count++;
        shared_ptr<IMG_MSG> feature_points(new IMG_MSG());

        //std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> image_buf;

        feature_points->header = dStampSec;
        vector<set<int>> hash_vFeatureIds(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++){
            auto &un_pts = trackerData[i].vCurUndistortPts;
            auto &vCurPts = trackerData[i].vCurPts;
            auto &vFeatureIds = trackerData[i].vFeatureIds;
            auto &vFeatureVelocity = trackerData[i].vFeatureVelocity;
            for (unsigned int j = 0; j < vFeatureIds.size(); j++){
                if (trackerData[i].vTrackCnt[j] > 1){
                    int p_id = vFeatureIds[j];
                    hash_vFeatureIds[i].insert(p_id);
                    double x = un_pts[j].x;
                    double y = un_pts[j].y;
                    double z = 1;
                    feature_points->points.push_back(Vector3d(x, y, z));
                    feature_points->id_of_point.push_back(p_id * NUM_OF_CAM + i);
                    feature_points->u_of_point.push_back(vCurPts[j].x);
                    feature_points->v_of_point.push_back(vCurPts[j].y);
                    feature_points->velocity_x_of_point.push_back(vFeatureVelocity[j].x);
                    feature_points->velocity_y_of_point.push_back(vFeatureVelocity[j].y);
                }
            }
            // TicToc t_s;
            /*
            for (unsigned int j = 0; j < vFeatureIds.size(); j++){
                if (trackerData[i].vTrackCnt[j] > 1){
                    int feature_id = vFeatureIds[j];
                    int camera_id = i;
                    Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                    xyz_uv_velocity << un_pts[j].x, un_pts[j].y, 1, vCurPts[j].x, vCurPts[j].y,
                                       vFeatureVelocity[j].x, vFeatureVelocity[j].y;
                    image_buf[feature_id].emplace_back(camera_id, xyz_uv_velocity);
                }
            }
            */
        }
        if (!init_pub){
            init_pub = 1;
        } else {
            m_buf.lock();
            feature_buf.push(feature_points);
            m_buf.unlock();

            con.notify_one();
        }
    }

	  if (SHOW_TRACK){
        cv::Mat show_img;
        cv::cvtColor(img, show_img, CV_GRAY2RGB);

		    for (unsigned int j = 0; j < trackerData[0].vCurPts.size(); j++){
			      double len = min(1.0, 1.0 * trackerData[0].vTrackCnt[j] / WINDOW_SIZE);
			      cv::circle(show_img, trackerData[0].vCurPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
		    }

        cv::namedWindow("IMAGE", CV_WINDOW_AUTOSIZE);
		    cv::imshow("IMAGE", show_img);
        cv::waitKey(1);
	  }
}

std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> System::getMeasurements()
{
    std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> measurements;

    while (true){
        if (imu_buf.empty() || feature_buf.empty()){
            //cerr << "1 imu_buf.empty() || feature_buf.empty()" << endl;
            return measurements;
        }

        if (!(imu_buf.back()->header > feature_buf.front()->header + estimator.td)){
            cerr << "wait for imu, only should happen at the beginning sum_of_wait: "
                << sum_of_wait << endl;
            sum_of_wait++;
            return measurements;
        }

        if (!(imu_buf.front()->header < feature_buf.front()->header + estimator.td)){
            cerr << "throw img, only should happen at the beginning" << endl;
            feature_buf.pop();
            continue;
        }
        ImgConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<ImuConstPtr> IMUs;
        while (imu_buf.front()->header < img_msg->header + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        // cout << "1 getMeasurements IMUs size: " << IMUs.size() << endl;
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty()){
            cerr << "no imu between two image" << endl;
        }
        // cout << "1 getMeasurements img t: " << fixed << img_msg->header
        //     << " imu begin: "<< IMUs.front()->header
        //     << " end: " << IMUs.back()->header
        //     << endl;
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

void System::PubImuData(double dStampSec, const Eigen::Vector3d &vGyr,
    const Eigen::Vector3d &vAcc)
{
    shared_ptr<IMU_MSG> imu_msg(new IMU_MSG());
	  imu_msg->header = dStampSec;
	  imu_msg->linear_acceleration = vAcc;
	  imu_msg->angular_velocity = vGyr;

    if (dStampSec <= last_imu_t){
        cerr << "imu message in disorder!" << dStampSec << " " << last_imu_t << endl;
        return;
    }
    last_imu_t = dStampSec;

    //std::cout << "receive IMU" << std::endl;

    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();
}

// thread: visual-inertial odometry
void System::ProcessBackEnd()
{
    cout << "==> [BACKEND] process back-end start" << endl;
    while (bStart_backend){
        std::vector<pair<vector<ImuConstPtr>, ImgConstPtr>> measurements;

        // get the measurements pairs of imus and image
        std::unique_lock<mutex> lk(m_buf);
        con.wait(lk, [&] {
            return (measurements = getMeasurements()).size() != 0;
        });
        lk.unlock();

        m_estimator.lock();
        TicToc t_ProcessImage;
        for (auto &measurement : measurements){
            std::cout << "==> [MEASUREMENT] process " << measurements.size() << " measurements." << std::endl;
            auto img_msg = measurement.second;
            // BASTIAN_M : this should be the image time stamp
            double img_t = img_msg->header + estimator.td;
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : measurement.first){
                double t = imu_msg->header;
                // if the imu is actually post the image
                // we will linear interpolate the imu value
                if (t <= img_t){
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    current_time = t;
                    estimator.processIMU(dt, imu_msg->linear_acceleration, imu_msg->angular_velocity);
                }else{
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    assert(dt_1 >= 0);
                    assert(dt_2 >= 0);
                    assert(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x();
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y();
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z();
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x();
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y();
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z();
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }

            cout << "==> [BACKEND] processing vision data with stamp:" << img_msg->header
                 << ", #features: "<< img_msg->points.size() << endl;

            // TicToc t_s;
            std::map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->id_of_point[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                double x = img_msg->points[i].x();
                double y = img_msg->points[i].y();
                double z = img_msg->points[i].z();
                double p_u = img_msg->u_of_point[i];
                double p_v = img_msg->v_of_point[i];
                double velocity_x = img_msg->velocity_x_of_point[i];
                double velocity_y = img_msg->velocity_y_of_point[i];
                assert(z == 1);
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id, xyz_uv_velocity);
            }
            estimator.ProcessImage(image, img_msg->header);

            if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR){
                Vector3d p_wi;
                Quaterniond q_wi;
                q_wi = Quaterniond(estimator.Rs[WINDOW_SIZE]);
                p_wi = estimator.Ps[WINDOW_SIZE];

                double dStamp = estimator.Headers[WINDOW_SIZE];

                vPath_to_draw.push_back(p_wi);
                keyframe_history.push_back(estimator.GetCurrentCameraPose());
                corresponding_timestamps.push_back(dStamp);

                cout << "==> [BACKEND] ProcessImage using: " << fixed << t_ProcessImage.toc() << ", p_wi: " << p_wi.transpose() << endl << endl;
                if(b_savepose)
                    ofs_pose << fixed << dStamp << " " << p_wi.transpose() << " " << q_wi.coeffs().transpose() << endl;
            }
        }
        m_estimator.unlock();
    }
}

void System::Draw()
{
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Map Viewer", 1224, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(200));
    pangolin::Var<bool> menuShowMarPoints("menu.Marginalized Points",true,true);
    pangolin::Var<bool> menuShowCurPoints("menu.Current Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.KeyFrames",true,true);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 384, 0.1, 1000),
            //pangolin::ModelViewLookAt(-5, 0, 15, 7, 0, 0, 1.0, 0.0, 0.0)
            pangolin::ModelViewLookAt(0,-0.7,-1.8, 0,0,0, 0.0, 0.0, 1.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glColor3f(0, 0, 1);
        pangolin::glDrawAxis(3);

        // draw poses
        glColor3f(0, 0, 0);
        glLineWidth(2);
        glBegin(GL_LINES);
        int nPath_size = vPath_to_draw.size();
        for(int i = 0; i < nPath_size-1; ++i)
        {
            glVertex3f(vPath_to_draw[i].x(), vPath_to_draw[i].y(), vPath_to_draw[i].z());
            glVertex3f(vPath_to_draw[i+1].x(), vPath_to_draw[i+1].y(), vPath_to_draw[i+1].z());
        }
        glEnd();

        if(menuShowKeyFrames){
          nPath_size = keyframe_history.size();
          for(int i = 0; i < nPath_size-1; i = i + 5)
          {
              DrawKeyframe(keyframe_history[i]);
          }
        }


        // points
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)  {
            glPointSize(5);
            glBegin(GL_POINTS);
            for(int i = 0; i < WINDOW_SIZE+1;++i)  {
                Vector3d p_wi = estimator.Ps[i];
                glColor3f(1, 0, 0);
                glVertex3d(p_wi[0],p_wi[1],p_wi[2]);
            }
            glEnd();
        }


        // points
        if(menuShowMarPoints){
        glPointSize(3);
        glBegin(GL_POINTS);
        glColor3f(0, 0, 0);
        int margalized_size = estimator.margin_cloud_cloud.size();
        for (int i = 0; i < margalized_size-1; i++)  {
            for(Vec3 p_wi : estimator.margin_cloud_cloud[i])  {
                glVertex3d(p_wi[0],p_wi[1],p_wi[2]);
            }
        }
        glEnd();

        glPointSize(5);
        glBegin(GL_POINTS);
        glColor3f(1, 0, 0);
        if(margalized_size > 0){
            for(Vec3 p_wi : estimator.margin_cloud_cloud[margalized_size-1]){
                glVertex3d(p_wi[0],p_wi[1],p_wi[2]);
            }
        }
        glEnd();
        }


        // map points currently seen
        if(menuShowCurPoints){
          glPointSize(5);
          glBegin(GL_POINTS);
          glColor3f(0, 1, 0);
          for(Vec3 p_wi : estimator.point_cloud){
              glVertex3d(p_wi[0],p_wi[1],p_wi[2]);
          }
          glEnd();
        }

        pangolin::FinishFrame();
        usleep(50000);   // sleep 5 ms
    }
}

void System::GetOpenGLCameraMatrix(Eigen::Matrix3d matrix_t, Eigen::Vector3d pVector_t, pangolin::OpenGlMatrix &M, bool if_inv)
{
    Eigen::Matrix3d matrix;
    Eigen::Vector3d pVector;

    //std::cout << matrix_t << std::endl;
    if(if_inv){
        matrix = matrix_t.transpose();
        pVector = - matrix * pVector_t;
    } else{
        matrix = matrix_t;
        pVector = pVector_t;
    }

    M.m[0] = matrix(0,0);M.m[1] = matrix(1,0);M.m[2] = matrix(2,0);M.m[3]  = 0.0;
    M.m[4] = matrix(0,1);M.m[5] = matrix(1,1);M.m[6] = matrix(2,1);M.m[7]  = 0.0;
    M.m[8] = matrix(0,2);M.m[9] = matrix(1,2);M.m[10] = matrix(2,2);M.m[11]  = 0.0;
    M.m[12] = pVector(0);M.m[13] = pVector(1);M.m[14] = pVector(2);M.m[15]  = 1.0;
}

void System::GetOpenGLMatrixCamera(pangolin::OpenGlMatrix &M, Eigen::Matrix<double, 3, 4> Twc)
{
    M.m[0] = Twc(0,0);M.m[1] = Twc(1,0);M.m[2] = Twc(2,0);M.m[3]  = 0.0;
    M.m[4] = Twc(0,1);M.m[5] = Twc(1,1);M.m[6] = Twc(2,1);M.m[7]  = 0.0;
    M.m[8] = Twc(0,2);M.m[9] = Twc(1,2);M.m[10] = Twc(2,2);M.m[11] = 0.0;
    M.m[12] = Twc(0,3);M.m[13] = Twc(1,3);M.m[14] = Twc(2,3);M.m[15] = 1.0;
}

void System::DrawKeyframe(Eigen::Matrix3d matrix_t, Eigen::Vector3d pVector_t)
{
    pangolin::OpenGlMatrix currentT;
    GetOpenGLCameraMatrix(matrix_t, pVector_t, currentT, false);

    glPushMatrix();
    glMultMatrixd(currentT.m);
    DrawCamera();
    glPopMatrix();
}

void System::DrawKeyframe(Eigen::Matrix<double, 3, 4> Twc)
{
    pangolin::OpenGlMatrix currentT;
    GetOpenGLMatrixCamera(currentT, Twc);

    glPushMatrix();
    glMultMatrixd(currentT.m);
    DrawCamera();
    glPopMatrix();
}

void System::DrawCamera()
{
    const float w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;
    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);glVertex3f(w,h,z);
    glVertex3f(0,0,0);glVertex3f(w,-h,z);
    glVertex3f(0,0,0);glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);glVertex3f(-w,h,z);
    glVertex3f(w,h,z);glVertex3f(w,-h,z);
    glVertex3f(-w,h,z);glVertex3f(-w,-h,z);
    glVertex3f(-w,h,z);glVertex3f(w,h,z);
    glVertex3f(-w,-h,z);glVertex3f(w,-h,z);
    glEnd();

}

Vector2d System::FindVelocity(IMG_MSG* last_features, int idx, double u, double v, double dStampSec)
{
    //if(last_feature)
    Vector2d velocity(0,0);
    for(size_t i = 0; i < last_features->id_of_point.size(); i ++){
        if(last_features->id_of_point[i] == idx){
            double inv_dt = 1/(dStampSec - last_features->header);
            velocity(0) = (last_features->u_of_point[i] - u) * inv_dt;
            velocity(1) = (last_features->v_of_point[i] - v) * inv_dt;
        }
    }

    return velocity;
}


void System::SavePoseAsTUM(std::string filename)
{
    std::cout << " save trajectory in TUM forme, to " << filename << std::endl;

    std::ofstream save_points;
    save_points.setf(std::ios::fixed, std::ios::floatfield);
    save_points.open(filename.c_str());

    for (size_t i = 0; i < keyframe_history.size(); ++i) {
        Eigen::Matrix<double, 3, 4> pose = keyframe_history[i];
        Eigen::Matrix3d RR = pose.block(0,0,3,3);

        double time = corresponding_timestamps[i];
        Eigen::Quaterniond q(RR);
        Eigen::Vector3d t = pose.block(0,3,3,1);

        save_points.precision(9);
        save_points <<time<<" ";
        save_points.precision(5);
        save_points <<t(0)<<" "
                    <<t(1)<<" "
                    <<t(2)<<" "
                    <<q.x()<<" "
                    <<q.y()<<" "
                    <<q.z()<<" "
                    <<q.w() <<std::endl;
    }

}

} // namespace
