#pragma once
#include <stdio.h>
#include <iostream>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/calib3d/calib3d.hpp"
#include <ctype.h>
#include "XnCppWrapper.h"
#include "BackgroundDepthSubtraction.h"
#include <ActivityMap_Utils.h>
#include "KinectSensor.h"
#include "PplTracker_Utils_v14.h"
//#include <XnUSB.h> 

using namespace std;
using namespace cv;
using namespace xn;
using namespace AMv14;

class PplTracker_v14
{
public:
	PplTracker_v14(void);
	~PplTracker_v14(void);

	void trackingMoA(int fromVideo, int recordOut, int tilt, int debug);
	void generateCalibrationImgs(int fromVideo, int recordOut, int tilt, int debug);

private:
	void writeTrackingResults(vector<AMv14::TrackInfo>& tracks);
	void pointSelectionHeight_onMouse(int event, int x, int y, int flags, void* param);

	XnPoint3D p;
	int kin;

};

