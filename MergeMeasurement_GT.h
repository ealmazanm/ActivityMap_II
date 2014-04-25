#pragma once
#include <stdio.h>
#include <iostream>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <ctype.h>
#include "XnCppWrapper.h"
#include "BackgroundDepthSubtraction.h"
#include <ActivityMap_Utils.h>
#include "KinectSensor.h"
#include "PplTracker_Utils_v1.h"

using namespace std;
using namespace cv;
using namespace xn;
using namespace AM;


//Merge measuerments in the RPSpace.
struct RPSDetections
{
	int idFrame;
	double area;
	int measId;
	AM::EllipseParam prmsRPS;
	AM::EllipseParam prmsMoA;
	bool merge;
};

	
static char* rpsDtPath = "d:\\Emilio\\Tracking\\DataSet\\sb125\\SecondDay\\DSet1\\mergeMeasurementsGT.txt";

class MergeMeasurement_GT
{
public:

	MergeMeasurement_GT(void);
	~MergeMeasurement_GT(void);

	void generateGT(int fromVideo, int recordOut, int tilt, int debug);
	void visualizeGT();

private:

	//void writeTrackingResults(vector<TrackInfo>& tracks);

};