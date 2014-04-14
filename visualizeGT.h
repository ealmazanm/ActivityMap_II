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
#include "PplTracker_Utils_v1.h"
//#include <XnUSB.h> 

using namespace std;
using namespace cv;
using namespace xn;
using namespace AM;


class visualizeGT
{
public:
	visualizeGT(void);
	~visualizeGT(void);

	void visualize(int tilt, int debug);
};

