#include "parameters.h"
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

double ACC_N, ACC_W;
double GYR_N, GYR_W;
vector<Eigen::Matrix3d> RIC;
vector<Eigen::Vector3d> TIC;
Eigen::Vector3d G{0.0, 0.0, 9.8};
int ESTIMATE_EXTRINSIC;
int ROW;
int COL;

void SetImageSize(int row_t, int col_t)
{
    ROW = row_t;
    COL = col_t;
}

void readParameters(string config_file)
{
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        cerr << "1 readParameters ERROR: Wrong path to settings!" << endl;
        return;
    }

    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
    int ROW_t = fsSettings["image_height"];
    int COL_t = fsSettings["image_width"];
    SetImageSize(ROW_t, COL_t);
    // ROS_INFO("ROW: %f COL: %f ", ROW, COL);

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        // ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        //EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
    }
    else
    {
        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation"] >> cv_R;
        fsSettings["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        RIC.push_back(eigen_R);
        TIC.push_back(eigen_T);
    }

    fsSettings.release();

    cout << "1 readParameters:  "
        <<  "\n  ACC_N: " <<ACC_N
        <<  "\n  ACC_W: " <<ACC_W
        <<  "\n  GYR_N: " <<GYR_N
        <<  "\n  GYR_W: " <<GYR_W
        <<  "\n  RIC:   " << RIC[0]
        <<  "\n  TIC:   " <<TIC[0].transpose()
        <<  "\n  G:     " <<G.transpose()
        <<  "\n  ESTIMATE_EXTRINSIC:"<<ESTIMATE_EXTRINSIC
        << endl;

}
