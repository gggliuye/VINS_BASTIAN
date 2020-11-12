
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <iomanip>

#include <cv.h>
#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <eigen3/Eigen/Dense>
#include "System.h"

using namespace std;
using namespace cv;
using namespace Eigen;

const int nDelayTimes = 1;
string sData_path = "/home/bastain/Bastian/VINS/MH_05_difficult/mav0/";
string sConfig_path = "../config/euroc_config_short.yaml";

//string traj_save_path = "/home/pi/JOJO/TestData/mav0/res/trajectory_sim.txt";

//std::shared_ptr<System> pSystem;
BASTIAN::System* pSystem;

void PubImuData()
{
	string sImu_data_file = "../config/MH_05_imu0.txt";
	cout << "==> [IMU] PubImuData start sImu_data_filea: " << sImu_data_file << endl;
	ifstream fsImu;
	fsImu.open(sImu_data_file.c_str());
	if (!fsImu.is_open())
	{
		cerr << "Failed to open imu data file! " << sImu_data_file << endl;
		return;
	}

	std::string sImu_line;
	double dStampNSec = 0.0;
	Vector3d vAcc;
	Vector3d vGyr;
	while (std::getline(fsImu, sImu_line) && !sImu_line.empty()) // read imu data
	{
		std::istringstream ssImuData(sImu_line);
		ssImuData >> dStampNSec >> vGyr.x() >> vGyr.y() >> vGyr.z() >> vAcc.x() >> vAcc.y() >> vAcc.z();
		// cout << "Imu t: " << fixed << dStampNSec << " gyr: " << vGyr.transpose() << " acc: " << vAcc.transpose() << endl;
		pSystem->PubImuData(dStampNSec / 1e9, vGyr, vAcc);
		usleep(5000*nDelayTimes);
	}
	fsImu.close();
}

void PubImageData()
{
	string sImage_file = "../config/MH_05_cam0.txt";

	cout << "==> [IMAGE] PubImageData start sImage_file: " << sImage_file << endl;

	ifstream fsImage;
	fsImage.open(sImage_file.c_str());
	if (!fsImage.is_open())
	{
		cerr << "Failed to open image paths file! " << sImage_file << endl;
		return;
	}

	std::string sImage_line;
	double dStampNSec;
	string sImgFileName;

	// cv::namedWindow("SOURCE IMAGE", CV_WINDOW_AUTOSIZE);
	while (std::getline(fsImage, sImage_line) && !sImage_line.empty())
	{
		std::istringstream ssImuData(sImage_line);
		ssImuData >> dStampNSec >> sImgFileName;
		// cout << "Image t : " << fixed << dStampNSec << " Name: " << sImgFileName << endl;
		string imagePath = sData_path + "cam0/data/" + sImgFileName;

		Mat img = imread(imagePath.c_str(), 0);
		if (img.empty())
		{
			cerr << "image is empty! path: " << imagePath << endl;
			return;
		}
		pSystem->PubImageData(dStampNSec / 1e9, img);
		// cv::imshow("SOURCE IMAGE", img);
		// cv::waitKey(0);
		usleep(50000*nDelayTimes);
	}
	fsImage.close();
}

int main(int argc, char **argv)
{
	cout << "  ---  VINS start  ---" << std::endl;

	//pSystem.reset(new System(sConfig_path, false));
	//readParameters(sConfig_path);
	pSystem = new BASTIAN::System(sConfig_path, false);

  //sleep(5);
	std::thread thd_BackEnd(&BASTIAN::System::ProcessBackEnd, pSystem);

	std::thread thd_PubImuData(PubImuData);

	std::thread thd_PubImageData(PubImageData);

	std::thread thd_Draw(&BASTIAN::System::Draw, pSystem);

	thd_PubImuData.join();
	thd_PubImageData.join();

	thd_BackEnd.join();
	thd_Draw.join();

  //pSystem->SavePoseAsTUM(traj_save_path);

	cout << "main end... see you ..." << endl;
	return 0;
}
