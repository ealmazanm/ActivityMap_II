#pragma once
#include "PplDetection_v1.h"
#include "PplTracker_v1.h"
#include "PplTracker_v14.h"
#include "MergeMeasurement_GT.h"
#include "visualizeGT.h"
#include <vld.h>

/*
Arg 1: 0:Video; 1:live
Arg 2: 0:No Record; 1:Record
*/
int main(int argc, char* argv[])
{	
	bool saved = false;
	int fromVideo = 1;
	int recordOut = 0;
	int tilt = 0;
	int debug  = -1;
	AM::tiltTXT >> tilt;
	cout << "Start" << endl;
	if (argc ==4)
	{
		sscanf(argv[1], "%d", &fromVideo);
		sscanf(argv[2], "%d", &recordOut);
		sscanf(argv[3], "%d", &debug);
	}

	cout << "Tilt: " << tilt;
	if (fromVideo)
		cout << ". Live execution";
	else
		cout << ". Video execution";

	if (!recordOut)
		cout << ". No record" << endl;
	else
		cout << ". Record" << endl;
	if (debug == AM::DEBUG_NONE)
		cout << ". Debug none" << endl;
	else if (debug == AM::DEBUG_LOW)
		cout << ". Debug low" << endl;
	else if (debug == AM::DEBUG_HIGH)
		cout << ". Debug high" << endl;

	//PplDetection_v1 pplDetc;
	//pplDetc.detection(fromVideo, recordOut, tilt, debug);

	//VLDEnable();
	//PplTracker_v1 pplTrack;
	//pplTrack.trackingMoA(fromVideo, recordOut, tilt, debug);
	//VLDReportLeaks();

	PplTracker_v14 pplTrack14;
	pplTrack14.trackingMoA(fromVideo, recordOut, tilt, debug);

	//To ground truth merge measurements
	 //MergeMeasurement_GT mmGt;
	 //mmGt.generateGT(fromVideo, recordOut, tilt, debug);
	 //mmGt.visualizeGT();
	//For aggregated images
	//visualizeGT vis;
	//vis.visualize(tilt,debug);
	return 0;
}