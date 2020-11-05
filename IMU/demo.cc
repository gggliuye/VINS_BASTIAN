#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/param.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <fstream>
#include <chrono>
#include <ctime>
extern "C" {

#include "adxl345/adxl345.h"
#include "hmc5883l/hmc5883l.h"
#include "itg3200/itg3200.h"

}

#define DELAY 30000

static int aFile = 0;
static int cFile = 0;
static int gFile = 0;
static int running;

static int closeAndExit(int code)
{
    close(aFile);
    close(cFile);
    close(gFile);
    return code;
}

/*
static void ctrlCHandler(int signum)
{
	running = 0;
}

static void setupHandlers(void)
{
    struct sigaction sa =
    {
        .sa_handler = ctrlCHandler,
    };

    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
}
*/
struct Coordinate
{
    short data;
	float smoothed;
	float min;
	float max;
    float normalized;
};

#define LERP_RATIO (0.000005f * DELAY)

static float lerp(float a, float b, float t)
{
	return a + (b - a) * t;
}

static void initCoordinate(struct Coordinate *coordinate)
{
	coordinate->smoothed = coordinate->min = coordinate->max = coordinate->normalized = coordinate->data;
}

static void updateCoordinate(struct Coordinate *coordinate)
{
	coordinate->smoothed = lerp(coordinate->smoothed, coordinate->data, LERP_RATIO);
	coordinate->min = MIN(coordinate->min, coordinate->smoothed);
 	coordinate->max = MAX(coordinate->max, coordinate->smoothed);
    coordinate->normalized = (coordinate->smoothed - coordinate->min) / (coordinate->max - coordinate->min) * 2.0f - 1.0f;
}

static struct Coordinate aX, aY, aZ;
//static struct Coordinate cX, cY, cZ;
static struct Coordinate gX, gY, gZ, gT;

#define MAX_ADAPTERS 2

#define CHECK(S) if (S) return -1;

static int aConfigure(int aFile)
{
    CHECK(ADXL345_Init(aFile, ADXL345_ID, true));

    struct ADXL345_DataFormat confDataFormat;
    confDataFormat.range = ADXL345_RANGE_2G;
    CHECK(ADXL345_ConfigureDataFormat(aFile, &confDataFormat));

    struct ADXL345_Power confPowerControl;
    confPowerControl.measurement = true;
    CHECK(ADXL345_ConfigurePower(aFile, &confPowerControl));

    return 0;
}

static int gConfigure(int gFile)
{
    CHECK(ITG3200_Init(gFile, ITG3200_ID, true));
    
    struct ITG3200_Acquisition confAcquisition = {
        .lowPassFilter = ITG3200_LOWPASSFILTER_42,
        .sampleRateDivider = 0,
    };
    CHECK(ITG3200_ConfigureAcquisition(gFile, &confAcquisition));
    
    struct ITG3200_Power confPower = {
        .clockSource = ITG3200_CLOCKSOURCE_PLL_X,
    };
    CHECK(ITG3200_ConfigurePower(gFile, &confPower));
    
    return 0;
}

int main(void)
{
    // open a file for writing the data (for use of calibration)
    std::ofstream ofs_imu;
    double max_during = 6 * 60 * 60;
    ofs_imu.open("./imu_data.txt",std::fstream::app | std::fstream::out);
    if(!ofs_imu.is_open()){
        //std::cerr << "ofs_pose is not open" << endl;
    }
    
    
	int adapter;
    char filename[20];

    for (adapter = 0; adapter < MAX_ADAPTERS; ++adapter)
    {
        snprintf(filename, 20, "/dev/i2c-%d", adapter);
		if (!access(filename, R_OK | W_OK))
			break;
    }

	if (adapter == MAX_ADAPTERS)
	{
		fprintf(stderr, "No I2C adapter found\n");
		return -1;
	}

    aFile = open(filename, O_RDWR);
    if (aConfigure(aFile))
    {
        fprintf(stderr, "Failed to initialize accelerometer\n");
        return closeAndExit(-1);
    }

    /*
    cFile = open(filename, O_RDWR);
    if (cConfigure(cFile))
    {
        fprintf(stderr, "Failed to initialize compass\n");
        return closeAndExit(-1);
    }
    */
    
    gFile = open(filename, O_RDWR);
    if (gConfigure(gFile))
    {
        fprintf(stderr, "Failed to initialize gyroscope\n");
        return closeAndExit(-1);
    }

    ADXL345_ReadData(aFile, &aX.data, &aY.data, &aZ.data);
    //HMC5883L_ReadData(cFile, &cX.data, &cY.data, &cZ.data);
    ITG3200_ReadData(gFile, &gX.data, &gY.data, &gZ.data);
    ITG3200_ReadTemperature(gFile, &gT.data);

    initCoordinate(&aX);
    initCoordinate(&aY);
    initCoordinate(&aZ);
    //initCoordinate(&cX);
    //initCoordinate(&cY);
    //initCoordinate(&cZ);
    initCoordinate(&gX);
    initCoordinate(&gY);
    initCoordinate(&gZ);
    initCoordinate(&gT);

    //setupHandlers();

    initscr();
    noecho();
    curs_set(false);

    running = 1;
    auto start = std::chrono::system_clock::now();
    while (running)
    {
        auto end = std::chrono::system_clock::now();
        
        ADXL345_ReadData(aFile, &aX.data, &aY.data, &aZ.data);
        //HMC5883L_ReadData(cFile, &cX.data, &cY.data, &cZ.data);
        ITG3200_ReadData(gFile, &gX.data, &gY.data, &gZ.data);
        ITG3200_ReadTemperature(gFile, &gT.data);

        updateCoordinate(&aX);
        updateCoordinate(&aY);
        updateCoordinate(&aZ);
        //updateCoordinate(&cX);
        //updateCoordinate(&cY);
        //updateCoordinate(&cZ);
        updateCoordinate(&gX);
        updateCoordinate(&gY);
        updateCoordinate(&gZ);
        updateCoordinate(&gT);

        std::chrono::duration<double> elapsed_seconds = end-start;
        ofs_imu << elapsed_seconds.count() << " " << aX.data << " " << aY.data << " " << aZ.data << " ";
        ofs_imu << gX.data << " " << gY.data << " " << gZ.data << " " << gT.data;
        ofs_imu << std::endl;
        if(elapsed_seconds.count() > max_during){
            break;
        }

        clear();
        
        int id_line = 0;
        mvprintw(id_line++, 0, "ADXL345:");
        mvprintw(id_line++, 0, "%16s: %8d X %8d Y %8d Z\n", "Accelerometer", aX.data, aY.data, aZ.data);
        mvprintw(id_line++, 0, "%16s: %8.1f X %8.1f Y %8.1f Z\n", "Smoothed", aX.smoothed, aY.smoothed, aZ.smoothed);
        mvprintw(id_line++, 0, "%16s: %8.1f X %8.1f Y %8.1f Z\n", "Min", aX.min, aY.min, aZ.min);
        mvprintw(id_line++, 0, "%16s: %8.1f X %8.1f Y %8.1f Z\n", "Max", aX.max, aY.max, aZ.max);
        mvprintw(id_line++, 0, "%16s: %8.1f X %8.1f Y %8.1f Z\n", "Normalized", aX.normalized, aY.normalized, aZ.normalized);

        //mvprintw(id_line++, 0, "%16s: %8d X %8d Y %8d Z\n", "Compass", cX.data, cY.data, cZ.data);
        //mvprintw(id_line++, 0, "%16s: %8.1f X %8.1f Y %8.1f Z\n", "Smoothed", cX.smoothed, cY.smoothed, cZ.smoothed);
        //mvprintw(id_line++, 0, "%16s: %8.1f X %8.1f Y %8.1f Z\n", "Min", cX.min, cY.min, cZ.min);
        //mvprintw(id_line++, 0, "%16s: %8.1f X %8.1f Y %8.1f Z\n", "Max", cX.max, cY.max, cZ.max);
        //mvprintw(id_line++, 0, "%16s: %8.1f X %8.1f Y %8.1f Z\n", "Normalized", cX.normalized, cY.normalized, cZ.normalized);
        id_line++;
        mvprintw(id_line++, 0, "ITG3200 Gyroscope:");
        mvprintw(id_line++, 0, "%16s: %8d X %8d Y %8d Z\n", "Gyroscope", gX.data, gY.data, gZ.data);
        mvprintw(id_line++, 0, "%16s: %8.1f X %8.1f Y %8.1f Z\n", "Smoothed", gX.smoothed, gY.smoothed, gZ.smoothed);
        mvprintw(id_line++, 0, "%16s: %8.1f X %8.1f Y %8.1f Z\n", "Min", gX.min, gY.min, gZ.min);
        mvprintw(id_line++, 0, "%16s: %8.1f X %8.1f Y %8.1f Z\n", "Max", gX.max, gY.max, gZ.max);
        mvprintw(id_line++, 0, "%16s: %8.1f X %8.1f Y %8.1f Z\n", "Normalized", gX.normalized, gY.normalized, gZ.normalized);

        id_line++;
        mvprintw(id_line++, 0, "ITG3200 Temperature:");
        mvprintw(id_line++, 0, "%16s: %8d %8.1f °C\n", "Temperature", gT.data, ITG3200_ConvertTemperature(gT.data));
        mvprintw(id_line++, 0, "%16s: %8.1f %8.1f °C\n", "Smoothed", gT.smoothed, ITG3200_ConvertTemperature(gT.smoothed));
        mvprintw(id_line++, 0, "%16s: %8.1f %8.1f °C\n", "Min", gT.min, ITG3200_ConvertTemperature(gT.min));
        mvprintw(id_line++, 0, "%16s: %8.1f %8.1f °C\n", "Max", gT.max, ITG3200_ConvertTemperature(gT.max));
        mvprintw(id_line++, 0, "%16s: %8.1f\n", "Normalized", gT.normalized);

        refresh();

        usleep(DELAY);
    }

    endwin();
	
    close(aFile);
    close(cFile);
    close(gFile);
    
    ofs_imu.close();

    return 0;
}
