#pragma once


#include <vector>
#include <eigen3/Eigen/Dense>
#include "utility/utility.h"
#include <fstream>

#define SHOW_TRACK true
#define FISHEYE false
#define EQUALIZE true

// whether to save point cloud
const bool B_SAVE_CLOUD = true;
const int NUM_OF_CAM = 1;

// feature tracker parameters
// as the feature are parameterized by inverse depth, which only need one parameter, will make a perfect diagonal matrix
// which is extremely fast for solve, so we could add more features without loss of the efficience.

const double FOCAL_LENGTH = 460; // not the camera focus, but for other processes
const int MAX_CNT = 200;  // max number of feature tracked in scene  #100
const int MIN_DIST = 20;  // min distance between two tracking features. #30
const int FREQ = 10;
const double F_THRESHOLD = 1.0;


//estimator
const int WINDOW_SIZE = 10;
//const int NUM_OF_F = 1000; // used to init the array in estimator
const double INIT_DEPTH = 2.0;
const double MIN_PARALLAX = 0.04; // only threshold for new keyframe #0.02

// set image size
extern int ROW;
extern int COL;

// from output
extern int ESTIMATE_EXTRINSIC;
extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;
extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern Eigen::Vector3d G;


// backend
#define ESTIMATE_TD false
const double TD = 0.0;
const double SOLVER_TIME = 0.04;
const int NUM_ITERATIONS = 8;
const double TR = 0.0;
const double BIAS_ACC_THRESHOLD = 0.1;
const double BIAS_GYR_THRESHOLD = 0.1;

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};

void readParameters(std::string config_file);
