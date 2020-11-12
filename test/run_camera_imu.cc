/*
* File:  run_camera_imu.cc
* Author:  BASTIAN
*/
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <librealsense2/rs.hpp>

#include "System.h"


/*
2020/11/11:
Prob : always miss aligned Imu with visual sfm result
      with result g abou the same value : 8.60848 -0.0635007   -8.58854
TODO : refine the initialization step
*/

bool check_imu_is_supported()
{
    bool found_gyro = false;
    bool found_accel = false;
    rs2::context ctx;
    for (auto dev : ctx.query_devices())
    {
        // The same device should support gyro and accel
        found_gyro = false;
        found_accel = false;
        for (auto sensor : dev.query_sensors())
        {
            for (auto profile : sensor.get_stream_profiles())
            {
                if (profile.stream_type() == RS2_STREAM_GYRO)
                    found_gyro = true;

                if (profile.stream_type() == RS2_STREAM_ACCEL)
                    found_accel = true;
            }
        }
        if (found_gyro && found_accel)
            break;
    }
    return found_gyro && found_accel;
}

int main()
{
	  std::string sConfig_path = "../config/Rs/realsense_config_short.yaml";
	  BASTIAN::System* pSystem = new BASTIAN::System(sConfig_path, true);

		std::thread thd_BackEnd(&BASTIAN::System::ProcessBackEnd, pSystem);
		std::thread thd_Draw(&BASTIAN::System::Draw, pSystem);

    ////////////// open realsense camera /////////////////
		// Before running the example, check that a device supporting IMU is connected
    if (!check_imu_is_supported())
    {
        std::cerr << "Device supporting IMU (D435i) not found";
        return EXIT_FAILURE;
    }

		// Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline pipe_rs;
		// Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // Add streams of gyro and accelerometer to configuration
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_COLOR, COL, ROW, RS2_FORMAT_BGR8, 30);

		// The callback is executed on a sensor thread and can be called simultaneously from multiple sensors
    // Therefore any modification to common memory should be done under lock
    std::mutex data_mutex;
		rs2_vector latest_gyro_data;

    // the gyro device has extremely high frequence, So we won't bother match it with acc data
		// we will simply use the latest received gyro data to match the acc data.
		auto callback_rs = [&](const rs2::frame& frame)
    {
        std::lock_guard<std::mutex> lock(data_mutex);

        if (auto motion = frame.as<rs2::motion_frame>()) {
				  	// If casting succeeded and the arrived frame is from gyro stream
				  	if (motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F){
						  	// Get gyro measures
						  	latest_gyro_data = motion.get_motion_data();
					  }
					  // If casting succeeded and the arrived frame is from accelerometer stream
					  else if (motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F){
							  // Get accelerometer measures Get the timestamp of the current frame
								double ts = motion.get_timestamp()/(1e3);
							  auto acc_data = motion.get_motion_data();

								Eigen::Vector3d vAcc;
								Eigen::Vector3d vGyr;
								vAcc << acc_data.x , acc_data.y, acc_data.z;
								vGyr << latest_gyro_data.x, latest_gyro_data.y, latest_gyro_data.z;
								pSystem->PubImuData(ts, vGyr, vAcc);
					  }
        }
        else if (auto frameset_t = frame.as<rs2::frameset>()) {
					  // Get tiemstamp and color image
            double ts = frameset_t.get_timestamp()/(1e3);
						auto color = frameset_t.get_color_frame();
						//printf("time: %f \n", ts);

						// Query frame size (width and height)
						const int w = color.as<rs2::video_frame>().get_width();
						const int h = color.as<rs2::video_frame>().get_height();

						// Create OpenCV matrix of size (w,h) from the colorized depth data
						cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
						cv::Mat gray;
						cv::cvtColor(image, gray, CV_RGB2GRAY);
						pSystem->PubImageData(ts, gray);
        }

        // Print the approximate pose and image rates once per second
				// example output ==> Acc rate: 62 Gyr rate: 1998 Image rate: 30
				/*
        auto now = std::chrono::system_clock::now();
        if (now - last_print >= std::chrono::seconds(1)) {
            std::cout << "\r" << std::setprecision(0) << std::fixed
                      << "Acc rate: "  << acc_counter << " "
											<< "Gyr rate: "  << gyr_counter << " "
                      << "Image rate: " << frame_counter << std::flush;
						acc_counter = 0;
            gyr_counter = 0;
            frame_counter = 0;
            last_print = now;
        }
				*/
    };

		// Configure and start the pipeline
		std::cout << "\n==> Start Realsense Device\n";
		rs2::pipeline_profile profiles = pipe_rs.start(cfg, callback_rs);

		thd_BackEnd.join();
		thd_Draw.join();

		// Sleep this thread until we are done
		//while(true) {
		//		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		//}

	  return 0;
}
