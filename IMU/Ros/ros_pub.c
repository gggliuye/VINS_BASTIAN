#include <fcntl.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/param.h>

#include "adxl345/adxl345.h"
#include "hmc5883l/hmc5883l.h"
#include "itg3200/itg3200.h"


#define DELAY 30000

static int aFile = 0;
static int cFile = 0;
static int gFile = 0;

static int closeAndExit(int code)
{
    close(aFile);
    close(cFile);
    close(gFile);
    return code;
}

static int running;

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
static struct Coordinate cX, cY, cZ;
static struct Coordinate gX, gY, gZ, gT;

#define MAX_ADAPTERS 2

#define CHECK(S) if (S) return -1;

static int aConfigure(int aFile)
{
    CHECK(ADXL345_Init(aFile, ADXL345_ID, true));

    struct ADXL345_DataFormat confDataFormat = {
        .range = ADXL345_RANGE_2G,
    };
    CHECK(ADXL345_ConfigureDataFormat(aFile, &confDataFormat));

    struct ADXL345_Power confPowerControl = {
        .measurement = true,
    };
    CHECK(ADXL345_ConfigurePower(aFile, &confPowerControl));

    return 0;
}

static int cConfigure(int cFile)
{
    CHECK(HMC5883L_Init(cFile, HMC5883L_ID, true));

    struct HMC5883L conf = {
        .gain = HMC5883L_GAIN_1090,
        .measurementMode = HMC5883L_MEASUREMENTMODE_NORMAL,
        .outputRate = HMC5883L_OUTPUTRATE_30,
        .samples = HMC5883L_SAMPLES_2,
    };
    CHECK(HMC5883L_Configure(cFile, &conf));

    CHECK(HMC5883L_SetContinuousMeasurement(cFile));

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
	int adapter;
    char filename[20];

    for (adapter = 0; adapter < MAX_ADAPTERS; ++adapter)
    {
        //snprintf(filename, 20, "/dev/i2c-%d", adapter);
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

    
    gFile = open(filename, O_RDWR);
    if (gConfigure(gFile))
    {
        fprintf(stderr, "Failed to initialize gyroscope\n");
        return closeAndExit(-1);
    }

    ADXL345_ReadData(aFile, &aX.data, &aY.data, &aZ.data);
    ITG3200_ReadData(gFile, &gX.data, &gY.data, &gZ.data);
    ITG3200_ReadTemperature(gFile, &gT.data);

    initCoordinate(&aX);
    initCoordinate(&aY);
    initCoordinate(&aZ);
    initCoordinate(&gX);
    initCoordinate(&gY);
    initCoordinate(&gZ);
    initCoordinate(&gT);

    running = 1;
    while (running)
    {
        ADXL345_ReadData(aFile, &aX.data, &aY.data, &aZ.data);
        ITG3200_ReadData(gFile, &gX.data, &gY.data, &gZ.data);
        ITG3200_ReadTemperature(gFile, &gT.data);

	printf("%16s: %8d X %8d Y %8d Z\n", "Accelerometer", aX.data, aY.data, aZ.data);
	printf("%16s: %8d X %8d Y %8d Z\n", "Gyroscope", gX.data, gY.data, gZ.data);
	printf("%16s: %8d %8.1f °C\n", "Temperature", gT.data, ITG3200_ConvertTemperature(gT.data));

        usleep(DELAY);
    }

	
    close(aFile);
    close(gFile);

    return 0;
}
