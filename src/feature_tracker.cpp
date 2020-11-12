#include "feature_tracker.h"

int FeatureTracker::n_id = 0;

bool InBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 2;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

void ReduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void ReduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

template <typename T>
std::vector<size_t> ArgSortVector(const std::vector<T> &v) {
    // initialize original index locations
    std::vector<size_t> idx(v.size());
    std::iota(idx.begin(), idx.end(), 0);
    // sort the vector
    std::sort(idx.begin(), idx.end(),
         [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});
    return idx;
}

FeatureTracker::FeatureTracker()
{
}

void FeatureTracker::GetMaskAndFilterPoints(cv::Mat &mMask)
{
    if(FISHEYE)
        mMask = fisheye_mask.clone();
    else
        mMask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));

    // prefer to keep features that are tracked for long time

    // the VINS origianl implementation of the sort function. which is about 2 times slower than my implementation
    //std::vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;
    //for (unsigned int i = 0; i < vForwPts.size(); i++)
    //    cnt_pts_id.push_back(make_pair(vTrackCnt[i], make_pair(vForwPts[i], vFeatureIds[i])));
    //sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
    //     {  return a.first > b.first; });

    std::vector<cv::Point2f> vForwPts_old = vForwPts;
    std::vector<int> vFeatureIds_old = vFeatureIds;
    std::vector<int> vTrackCnt_old = vTrackCnt;

    vForwPts.clear();
    vFeatureIds.clear();
    vTrackCnt.clear();

    // BASTIAN_M : update a new version to sort and filter the version. use about half the time of the original.
    for (auto &it : ArgSortVector(vTrackCnt_old)){
        cv::Point2f &pt = vForwPts_old[it];
        if (mMask.at<uchar>(pt) == 255) {
            vForwPts.push_back(pt);
            vFeatureIds.push_back(vFeatureIds_old[it]);
            vTrackCnt.push_back(vTrackCnt_old[it]);
            cv::circle(mMask, pt, MIN_DIST, 0, -1);
        }
    }

}

void FeatureTracker::AddPointsToTrack(std::vector<cv::Point2f> &vNewFeatures)
{
    for (auto &p : vNewFeatures)
    {
        vForwPts.push_back(p);
        vFeatureIds.push_back(-1);
        vTrackCnt.push_back(1);
    }
}

void FeatureTracker::ReadImage(const cv::Mat &_img, double _cur_time, bool bPublish)
{
    cv::Mat img;
    cur_time = _cur_time;

    if (EQUALIZE){
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(_img, img);
    } else {
        img = _img;
    }

    if (mForwImg.empty()) {
        // for the initial state
        mCurImg = mForwImg = img;
    } else {
        mForwImg = img;
    }

    vForwPts.clear();

    if (vCurPts.size() > 0)
    {
        //TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        cv::calcOpticalFlowPyrLK(mCurImg, mForwImg, vCurPts, vForwPts, status, err, cv::Size(21, 21), 3);

        for (int i = 0; i < int(vForwPts.size()); i++)
            if (status[i] && !InBorder(vForwPts[i]))
                status[i] = 0;

        ReduceVector(vCurPts, status);
        ReduceVector(vForwPts, status);
        ReduceVector(vFeatureIds, status);
        ReduceVector(vTrackCnt, status);
    }

    // record tracked count
    for (auto &n : vTrackCnt)
        n++;

    if (bPublish){
        RejectWithFundamentalMatrix();
        cv::Mat mMask;
        GetMaskAndFilterPoints(mMask);
        // if we want more features for tracking
        int n_max_cnt = MAX_CNT - static_cast<int>(vForwPts.size());
        if (n_max_cnt > 0){
            std::vector<cv::Point2f> vNewFeatures;
            cv::goodFeaturesToTrack(mForwImg, vNewFeatures, MAX_CNT - vForwPts.size(), 0.01, MIN_DIST, mMask);
            AddPointsToTrack(vNewFeatures);
        }
    }
    mCurImg = mForwImg;
    vCurPts = vForwPts;
    prev_time = cur_time;
    UndistortedPoints();
}

void FeatureTracker::RejectWithFundamentalMatrix()
{
    if (vForwPts.size() >= 8)
    {
        //ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        std::vector<cv::Point2f> un_vCurPts(vCurPts.size()), un_vForwPts(vForwPts.size());
        for (unsigned int i = 0; i < vCurPts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(vCurPts[i].x, vCurPts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_vCurPts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(vForwPts[i].x, vForwPts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_vForwPts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_vCurPts, un_vForwPts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        //int size_a = vCurPts.size();

        ReduceVector(vCurPts, status);
        ReduceVector(vForwPts, status);
        ReduceVector(vFeatureIds, status);
        ReduceVector(vTrackCnt, status);
    }
}

bool FeatureTracker::updateID(unsigned int i)
{
    if (i < vFeatureIds.size()){
        if (vFeatureIds[i] == -1)
            vFeatureIds[i] = n_id++;
        return true;
    } else {
        return false;
    }
}

void FeatureTracker::UpdateIdMono()
{
    for(size_t i = 0; i < vFeatureIds.size(); i++){
        if (vFeatureIds[i] == -1)
            vFeatureIds[i] = n_id++;
    }
}

void FeatureTracker::ReadIntrinsicParameter(const string &calib_file)
{
    cout << "reading paramerter of camera " << calib_file << endl;
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

void FeatureTracker::ShowUndistortion(const string &name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    std::vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++){
        for (int j = 0; j < ROW; j++){
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
        }
    }
    for (int i = 0; i < int(undistortedp.size()); i++){
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = mCurImg.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}

void FeatureTracker::UndistortedPoints()
{
    //std::vector<cv::Point2f> vCurUndistortPts;
    vCurUndistortPts.clear();
    std::map<int, cv::Point2f> mapCurUndistortPts;

    for (unsigned int i = 0; i < vCurPts.size(); i++)
    {
        Eigen::Vector2d a(vCurPts[i].x, vCurPts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        vCurUndistortPts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        mapCurUndistortPts.insert(make_pair(vFeatureIds[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
    }
    // caculate points velocity
    if (!mapPrevUndistortPts.empty())
    {
        double dt = cur_time - prev_time;
        vFeatureVelocity.clear();
        for (unsigned int i = 0; i < vCurUndistortPts.size(); i++)
        {
            if (vFeatureIds[i] != -1){
                std::map<int, cv::Point2f>::iterator it;
                it = mapPrevUndistortPts.find(vFeatureIds[i]);
                if (it != mapPrevUndistortPts.end()){
                    double v_x = (vCurUndistortPts[i].x - it->second.x) / dt;
                    double v_y = (vCurUndistortPts[i].y - it->second.y) / dt;
                    vFeatureVelocity.push_back(cv::Point2f(v_x, v_y));
                } else {
                    vFeatureVelocity.push_back(cv::Point2f(0, 0));
                }
            } else {
                vFeatureVelocity.push_back(cv::Point2f(0, 0));
            }
        }
    } else {
        vFeatureVelocity = std::vector<cv::Point2f>(vCurPts.size(),cv::Point2f(0, 0));
    }
    mapPrevUndistortPts = mapCurUndistortPts;
}
