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
string sConfig_path = "/home/viki/DeepMirror/VINS_BASTIAN/config/deepmirror_config_short.yaml";

std::string imu_file = "/home/viki/DeepMirror/Alpha/MapData/20201126T102742.602512854+0800_basement/SFM/imu.txt";
std::string image_folder = "/home/viki/DeepMirror";
std::string image_file = "/home/viki/DeepMirror/Alpha/MapData/20201126T102742.602512854+0800_basement/SFM/images.txt";

//string traj_save_path = "/home/pi/JOJO/TestData/mav0/res/trajectory_sim.txt";

//std::shared_ptr<System> pSystem;
BASTIAN::System* pSystem;

void PubImuData()
{
	string sImu_data_file = imu_file;
	cout << "==> [IMU] PubImuData start sImu_data_filea: " << sImu_data_file << endl;
	ifstream fsImu;
	fsImu.open(sImu_data_file.c_str());
	if (!fsImu.is_open()){
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
		usleep(2000*nDelayTimes);
	}
	fsImu.close();
}

void PubImageData()
{
	string sImage_file = image_file;

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
		string imagePath = image_folder + sImgFileName;
		//cout << "Image t : " << fixed << dStampNSec << " Name: " << imagePath << endl;

		Mat img = imread(imagePath.c_str(), 0);
		if (img.empty())
		{
			cerr << "image is empty! path: " << imagePath << endl;
			return;
		}
		cv::resize(img, img, cv::Size(img.cols/2, img.rows/2));
		pSystem->PubImageData(dStampNSec / 1e9, img);
		// cv::imshow("SOURCE IMAGE", img);
		// cv::waitKey(0);
		usleep(20000*nDelayTimes);
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
