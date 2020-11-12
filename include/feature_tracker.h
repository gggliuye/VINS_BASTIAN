#ifndef VINS_INCLUDE_FEATURE_TRACKER_H_
#define VINS_INCLUDE_FEATURE_TRACKER_H_

#include <cstdio>
#include <iostream>
#include <queue>
#include <numeric>      // std::iota
#include <execinfo.h>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "parameters.h"
#include "utility/tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool InBorder(const cv::Point2f &pt);
void ReduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void ReduceVector(vector<int> &v, vector<uchar> status);

template <typename T>
std::vector<size_t> ArgSortVector(const std::vector<T> &v);

class FeatureTracker
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    FeatureTracker();

    void ReadImage(const cv::Mat &_img, double _cur_time, bool bPublish);
    void UndistortedPoints();

    void GetMaskAndFilterPoints(cv::Mat &mMask);
    void AddPointsToTrack(std::vector<cv::Point2f> &vNewFeatures);
    void RejectWithFundamentalMatrix();

    // update the vFeatureIds of the features (this complicate function allow correct update for multiple cameras)
    bool updateID(unsigned int i);

    // update Ids as linear process, only for monocular camera case
    void UpdateIdMono();

    void ReadIntrinsicParameter(const std::string &calib_file);
    void ShowUndistortion(const std::string &name);

public:
    double cur_time;
    double prev_time;
    cv::Mat mCurImg, mForwImg;
    std::vector<cv::Point2f> vCurPts, vForwPts;
    std::vector<int> vFeatureIds;
    std::vector<int> vTrackCnt;
    std::vector<cv::Point2f> vFeatureVelocity;

    // saved for publisher
    std::vector<cv::Point2f> vCurUndistortPts;

    // previous tracked points, for the calculation of feature velocities
    std::map<int, cv::Point2f> mapPrevUndistortPts;

    camodocal::CameraPtr m_camera;
    cv::Mat fisheye_mask;

    static int n_id;
};

#endif // VINS_INCLUDE_FEATURE_TRACKER_H_
