#include "PplDetection_v2.h"
#include <vld.h>



PplDetection_v2::PplDetection_v2(void)
{
}


PplDetection_v2::~PplDetection_v2(void)
{
}

 void pointSelectionBBox_onMouse(int event, int x, int y, int flags, void* param)
{
	if (event == CV_EVENT_FLAG_LBUTTON)
	{
		Mat* img = (Mat*)param;
		if (x != -1 && y != -1)
		{
			/*rectangle(*img8, Rect(x,y, 10, 10), Scalar::all(255));
			int maxY = y + 10;
			int maxX = x + 10;
			int ttlProj = 0;
			int ttlPnts = 100;
			for (int i = y; y < maxY; y++)
			{
				ushort* ptr = img->ptr<ushort>(i);
				for (int j = x; j < maxX; j++)
				{
					ttlProj = ptr[j];
				}
			}
			float avg = ttlProj/ttlPnts;
			int meanX = x + 5;
			int meanY = y + 5;*/
			

			int val = img->ptr<ushort>(y)[x];

			float X = x*ActivityMap_Utils::X_STEP + ActivityMap_Utils::MIN_X;
			float Z = (img->rows - y)*ActivityMap_Utils::Z_STEP;
			float range = sqrtf(X*X + Z*Z);

			outPProjec << range << " " << val << endl;
			cout << "Range: " << range << "- > Proj at (" << x << ", " << y << "): " << val << endl;
		}
	}
}



void PplDetection_v2::writeTrackingResults(vector<TrackInfo>& tracks)
{
	ofstream outGt ("d:\\Emilio\\Tracking\\DataSet\\sb125\\SecondDay\\DSet1\\Detections_ellipses_v1.txt");
	//write on files the tracks for posterior evaluation
	for (int i = 0; i < tracks.size(); i++)
	{
		TrackInfo trck = tracks[i];	
		
		int totalFrames = trck.trajectory.size();
		for (int posId = 0; posId < totalFrames; posId++)
		{
			Position pos = trck.trajectory[posId];

			outGt << trck.id << " " << pos.mean.x << " " << pos.mean.y << " " << pos.covX << " " << pos.covY << " " << pos.covXY << " " << pos.frameId << " " << 0 << endl;		
			
		}
		
	}

}


void PplDetection_v2::detection(int fromVideo, int recordOut, int tilt, int debug)
{

	vector<TrackInfo> tracks;

	bool saved = false;

	char* paths[3];
	paths[0] = "d:/Emilio/Tracking/DataSet/sb125/SecondDay/DSet1/kinect0_calib.oni";
	paths[1] = "d:/Emilio/Tracking/DataSet/sb125/SecondDay/DSet1/kinect1_calib.oni";
	paths[2] = "d:/Emilio/Tracking/DataSet/sb125/SecondDay/DSet1/kinect2_calib.oni";

	//Initialize resolutions MoA and Remap Polar space
	ActivityMap_Utils actMapCreator(DEPTH_SCALE, NUM_SENSORS);


	KinectSensor kinects[NUM_SENSORS];
	const XnDepthPixel* depthMaps[NUM_SENSORS];
	const XnRGB24Pixel* rgbMaps[NUM_SENSORS];

	for (int i = 0; i < NUM_SENSORS; i++)
	{
		if (fromVideo == 0)
			kinects[i].initDevice(i, REF_CAM, true, paths[i]);
		else
			kinects[i].initDevice(i, REF_CAM, true);

		kinects[i].startDevice();
		kinects[i].tilt(tilt);
	}


	KINECTS_DISPLACEMENT = max(abs(kinects[0].translation(0)), abs(kinects[2].translation(0))); //MAXIMUM TRANSLATION IN THE HORIZONTAL AXIS
	MAX_RANGE = ActivityMap_Utils::MAX_Z_TRANS + KINECTS_DISPLACEMENT; 
	
	Mat *activityMap, *activityMap_Back;
	Mat whiteBack, colorMap;
	Mat background = Mat(actMapCreator.getResolution(), CV_8UC1);
	Mat backgroundPolar = Mat(actMapCreator.getResolution().height+150, 181, CV_8UC3);

	//flags
	bool bShouldStop = false;
	bool trans = true;
	bool bgComplete = true;
	bool deleteBG = true;

	Mat foreImages[NUM_SENSORS]; //for debugging for the foreground points
	Mat depthImages[NUM_SENSORS];
	Mat rgbImages[NUM_SENSORS];
	Mat depthMat[NUM_SENSORS];
	Mat masks[NUM_SENSORS];
	Mat grey;

	BackgroundDepthSubtraction subtractors[NUM_SENSORS];
	int numberOfForegroundPoints[NUM_SENSORS];

	XnPoint3D* pointsFore2D [NUM_SENSORS];
	XnPoint3D* points3D[NUM_SENSORS];

	for (int i = 0; i < NUM_SENSORS; i++)
	{
		foreImages[i] = Mat::zeros(XN_VGA_Y_RES, XN_VGA_X_RES, CV_8UC1);
		depthImages[i] = Mat(XN_VGA_Y_RES, XN_VGA_X_RES, CV_8UC3);
		rgbImages[i] = Mat(XN_VGA_Y_RES, XN_VGA_X_RES, CV_8UC3);
		depthMat[i] = Mat(XN_VGA_Y_RES, XN_VGA_X_RES, CV_16U);
		pointsFore2D[i] = new XnPoint3D[MAX_FORGROUND_POINTS];
		numberOfForegroundPoints[i] = 0;
	}

	bool first = true;

	Mat polar = Mat(RANGE_ROWS,181, CV_16UC1);
	Mat polarAlt= Mat(RANGE_ROWS,181, CV_16UC1);
	Mat polarAlt_smooth= Mat(RANGE_ROWS,181, CV_16UC1);
	Mat polarAlt_ = Mat(RANGE_ROWS, 181, CV_8UC1);
	Mat polarAlt_smooth_ = Mat(RANGE_ROWS, 181, CV_8UC1);
	Mat polar_ = Mat(RANGE_ROWS, 181, CV_8UC1);
	Mat m = Mat(polarAlt_smooth_.size(), CV_8UC3);
	Mat* outMoA;
	Mat outMoAScaled;

	
	int waitTime = 1;

	
	VideoWriter w, w1;
	recordOut = 0;
	if (recordOut == 1)
	{
		w.open("d:\\Emilio\\Tracking\\DataSet\\sb125\\SecondDay\\DSet1\\RemapPolarSpace_Detection.avi",CV_FOURCC('P','I','M','1'), 20.0, polarAlt_smooth_.size(), true);
		w1.open("d:\\Emilio\\Tracking\\DataSet\\sb125\\SecondDay\\DSet1\\MoA_tracking_V1.mpg",CV_FOURCC('P','I','M','1'), 20.0, actMapCreator.getResolution(), true);
	}	

	//Size of kernel: smooth rps;
	Mat kernel = Mat::ones(Size(10,10), CV_32F);

	char* nWindows[NUM_SENSORS];
	nWindows[0] =  "rgb 0";
	nWindows[1] =  "rgb 1";
	nWindows[2] =  "rgb 2";

	char* windMoA = "Activity Map";
	clock_t startTime = clock();
	clock_t startTotalTime = clock();
	clock_t startTime_tmp;
	int nPoints = 0;
	int frames = 0;
	int debugFrame = -1;

	vector<PointMapping>* pntsMap2 = new vector<PointMapping>[polarAlt.rows*polarAlt.cols];
	while (!bShouldStop && frames < 1000)
	{		
		if (frames == 30)
			cout << "Stop" << endl;
		printf("\rFrame %d", frames);

		if (frames == 5) 
			bgComplete = true;
		
		if (debug >= DEBUG_MED)
		{
			cout << "Frames " << frames << endl;			
			if (frames%10 == 0)
			{
				clock_t endTime = clock();
				double fps = 1/(double(endTime-startTime)/(double(CLOCKS_PER_SEC)*10));
				if (debug == DEBUG_OUT)
					fpsOut << frames << " " << fps << " " << nPoints << endl;
			
				cout << "Frame " << frames << ": " << fps << " fps. (" << nPoints << ")" << endl;
				startTime = clock();
			}
		}

		Utils::initMat1s(polarAlt, 0);
		Utils::initMat1s(polar, 0);

		for (int i = 0; i < NUM_SENSORS; i++)
			kinects[i].waitAndUpdate();
		
		for (int i = 0; i < NUM_SENSORS;  i++)
		{
			depthMaps[i] = kinects[i].getDepthMap();
			rgbMaps[i] = kinects[i].getRGBMap();
			//new part
			kinects[i].getDepthImage(depthImages[i]);
			kinects[i].getRGBImage(rgbImages[i]);
			
			//Creates a matrix with depth values (ushort)
			createDepthMatrix(depthMaps[i], depthMat[i]);

			//to create a mask for the noise (depth img is threhold)
			cvtColor(depthImages[i],grey,CV_RGB2GRAY);
			masks[i] = grey > 250; //mask that identifies the noise (1)
		}
		
		nPoints = 0;
		if (bgComplete && trans)// && frames > 20) //Trans must be true
		{
			if (first)
			{
				whiteBack = Mat::zeros(actMapCreator.getResolution(), CV_8UC1) + 255;
				activityMap = new Mat(actMapCreator.getResolution(), CV_8UC1);
				activityMap_Back = new Mat(actMapCreator.getResolution(), CV_8UC1);
				first = false;
			}
			whiteBack.copyTo(*activityMap);
			background.copyTo(*activityMap_Back);
			for (int i = 0; i < NUM_SENSORS; i++)
			{
				startTime_tmp = clock(); //time debuggin
				numberOfForegroundPoints[i] = subtractors[i].subtraction(pointsFore2D[i], &(depthMat[i]), &(masks[i]));
				if (debug >= DEBUG_HIGH)//to show the foreground points on the image plane
					foreImages[i] = Mat::zeros(XN_VGA_Y_RES, XN_VGA_X_RES, CV_8UC1);
					updateForegroundImg(foreImages[i], pointsFore2D[i], numberOfForegroundPoints[i]);
				totalIntervals[BSUB_ID] += clock() - startTime_tmp; //time debugging
				nPoints += numberOfForegroundPoints[i];
			}
			if (nPoints > 0)
			{
				Mat acMoA = Mat::zeros(activityMap->size(), CV_16U);
				Person* dtctPpl = new Person[MAX_PEOPLE];
				int ttl_dtctPpl = 0;

				int ttl = RANGE_ROWS*RANGE_COLS;
				for (int i = 0; i < ttl; i++)
				{
					if (!pntsMap2[i].empty())
						pntsMap2[i].clear();
				}
				
				int ttlPnts = 0;
				for (int i = 0; i < NUM_SENSORS; i++)
				{
					startTime_tmp = clock();
					points3D[i] = new XnPoint3D[numberOfForegroundPoints[i]];
					kinects[i].arrayBackProject(pointsFore2D[i], points3D[i], numberOfForegroundPoints[i]);
					totalIntervals[BACKPR_ID] += clock() - startTime_tmp; //time debugging

					startTime_tmp = clock();
					kinects[i].transformArray(points3D[i], numberOfForegroundPoints[i]);
					totalIntervals[PTRANS_ID] += clock() - startTime_tmp; //time debugging

					
					
					//Create alternative representation
					//startTime_tmp = clock();
					//updatePolarAlternateive(&polarAlt, &polar, pntsMap2, ttlPnts, points3D[i], pointsFore2D[i], rgbMaps[i], numberOfForegroundPoints[i], debug, i);	
					//totalIntervals[RPSPACE_ID] += clock() - startTime_tmp; //time debugging

				
					startTime_tmp = clock();
						updateActivityMap(acMoA, points3D[i], numberOfForegroundPoints[i]);
						
						if (i == 2)
						{
							Utils::convert16to8(&acMoA, *activityMap);
							Utils::convert16to8(&acMoA, *activityMap_Back);
							//threshold(*activityMap, *activityMap, 254, 255, THRESH_BINARY);
						}
					totalIntervals[MOA_ID] += clock() - startTime_tmp; //time debugging
					
				}
		
				//Todo: Create a method detection(polarAlt, moAPeople)
				startTime_tmp = clock();
				Mat acMoA_Smooth;
				cv::filter2D(acMoA, acMoA_Smooth, -1, kernel);

				if (debug >= DEBUG_HIGH && frames > 230)
				{
					Mat acMoASmooth_8;
					Utils::convert16to8(&acMoA_Smooth, acMoASmooth_8);
					rectangle(acMoASmooth_8, Rect(10,10, 10,10), Scalar::all(0)); //show the dimensions of the kernel


					namedWindow("AcSmooth", 0);
					cvSetMouseCallback("AcSmooth", pointSelectionBBox_onMouse, &acMoA_Smooth);

					imshow("AcSmooth", acMoASmooth_8);
					imshow("Ac-noSmooth", *activityMap);
					waitKey(0);
				}

				totalIntervals[SMOOTH_ID] += clock() - startTime_tmp; //time debugging

				
				startTime_tmp = clock();
				ccDetection(acMoA_Smooth, dtctPpl, ttl_dtctPpl, pntsMap2, ttlPnts, debug, frames, debugFrame, false); //Connected component detection
				totalIntervals[DET_ID] += clock() - startTime_tmp; //time debugging
		
				
				//generate tracks history
				generateTrackHistory(tracks, dtctPpl, ttl_dtctPpl, frames);				

				//For display purposes
				if (debug >= DEBUG_MED)
				{
					Utils::convert16to8(&polarAlt_smooth, polarAlt_smooth_);
					Utils::convert16to8(&polarAlt, polarAlt_);
					Utils::convert16to8(&polar, polar_);
					displayTrackersRPS(dtctPpl, ttl_dtctPpl, polarAlt_smooth_, debug);
					imshow("Polar Alt", polar_);
					imshow("Polar Alt Smooth_", polarAlt_smooth_);
					imshow("Polar Alt", polarAlt_);
				}
				Mat *tmp = activityMap;
				if (!deleteBG)
					tmp = activityMap_Back;
							
				startTime_tmp = clock();
				displayDetections(dtctPpl, ttl_dtctPpl, polarAlt_smooth_, *tmp, debug);
				totalIntervals[DISPLAY_ID] += clock() - startTime_tmp; //time debugging
				

				delete [] dtctPpl;
				ttl_dtctPpl = 0;
				for (int i = 0; i < NUM_SENSORS; i++)
					delete []points3D[i];

			}
			outMoA = activityMap;
			if (!deleteBG)
					outMoA = activityMap_Back;

			if (recordOut == 1)
			{
				w << polarAlt_smooth_;
				w1 << *activityMap;
			}
		}
		else
		{			
			startTime_tmp = clock();
			actMapCreator.createActivityMap(kinects, depthMaps, rgbMaps, trans, background, frames, MAX_RANGE, totalSubIntervalsMOA, MODEL_MAX_HEIGHT, MODEL_MIN_HEIGHT); 
			totalIntervals[MOA_ID] += clock() - startTime_tmp; //time debugging
			
			outMoA = &background;
		}

		if (DEPTH_SCALE < 1)
		{			
			resize(*outMoA, outMoAScaled, Size(outMoA->cols/DEPTH_SCALE, outMoA->rows/DEPTH_SCALE), 0,0, INTER_LINEAR);
			outMoA = &outMoAScaled;
		}
		imshow(windMoA, *outMoA);

		if (debug >= DEBUG_HIGH)
		{
			imshow(nWindows[0], rgbImages[0]);
			imshow(nWindows[1], rgbImages[1]);
			imshow(nWindows[2], rgbImages[2]);
		}

		int c = waitKey(waitTime);
		if (c == 13)
		{
			waitTime = !waitTime;

			imwrite("d:/Emilio/Tracking/DataSet/sb125/SecondDay/DSet1/captures/fore0.jpg", foreImages[0]);
			imwrite("d:/Emilio/Tracking/DataSet/sb125/SecondDay/DSet1/captures/fore1.jpg", foreImages[1]);
			imwrite("d:/Emilio/Tracking/DataSet/sb125/SecondDay/DSet1/captures/fore2.jpg", foreImages[2]);
			imwrite("d:/Emilio/Tracking/DataSet/sb125/SecondDay/DSet1/captures/MoA.jpg", *outMoA);

		}
		else if (c == 27)
			bShouldStop = true;

		frames++;
	}

	delete []pntsMap2;

	for (int i = 0; i < NUM_SENSORS; i++)
		delete []pointsFore2D[i];


	delete activityMap;
	delete activityMap_Back;

	writeTrackingResults(tracks);


	totalIntervals[TOT_ID] = clock() - startTotalTime;
	//BUILD REPORT
	outDebugFile << endl << endl << "EXECUTION TIME REPORT" << endl;
	for (int i = 0; i < TOTAL_INTERVALS-1; i++)
	{
		float time_p = totalIntervals[i]*100/totalIntervals[TOT_ID];
		outDebugFile << titles[i] << ": " << time_p << " %" << endl;
	}
	outDebugFile << endl;

	double trackSecs = (double(totalIntervals[TRACK_ID])/(double(CLOCKS_PER_SEC)));
	double tracksAvs = trackSecs/frames;
	outDebugFile << "Total time tracking (secs): " << trackSecs << endl;
	outDebugFile << "Avg time tracking (secs): " << tracksAvs << endl;

	outDebugFile << "PARTIAL EXECUTION PEOPLE DETECTION" << endl;
	for (int i = 0; i < TOTAL_SUBINTERVAL_DETECTION; i++)
	{
		float timpe_p = totalSubIntervalsDetection[i]*100/totalIntervals[DET_ID];
		outDebugFile << titles_subDet[i] << ": " << timpe_p << " %" << endl;
	}

//	outDebugFile << endl;
	//outDebugFile << "PARTIAL pARTIAL EXECUTION BUILD APPERANCE MODEL" << endl;
	//for (int i = 0; i < TOTAL_SUBSUBINTERVAL_APP; i++)
	//{
	//	float timpe_p = totalSubSubIntevalsAPP[i]*100/totalSubIntervalsDetection[BUILDAPPEARANCE_ID];
	//	outDebugFile << titles_subsubAPP[i] << ": " << timpe_p << " %" << endl;
	//}

	//outDebugFile << "PARTIAL EXECUTION TIME REPORT (MoA)" << endl;
	//for (int i = 0; i < TOTAL_SUBINTERVAL_MOA; i++)
	//{
	//	float time_p = totalSubIntervalsMOA[i]*100/totalIntervals[MOA_ID];
	//	outDebugFile << titles_subMOA[i] << ": " << time_p << " %" << endl;
	//}


	//outDebugFile << "PARTIAL EXECUTION TIME REPORT (UPDATE POLAR ALTERNATIVE)" << endl;
	//for (int i = 0; i < TOTAL_SUBINTERVAL_RPS; i++)
	//{
	//	float time_p = totalSubIntervalsRPS[i]*100/totalIntervals[RPSPACE_ID];
	//	outDebugFile << titles_subRPS[i] << ": " << time_p << " %" << endl;
	//}


	double fps = frames/(double(totalIntervals[TOT_ID])/(double(CLOCKS_PER_SEC)));
	outDebugFile << "Total frames processed: " << frames << ". fps: " << fps << endl;
	
	for (int i = 0; i < NUM_SENSORS; i++)
	{
		kinects[i].stopDevice();
  		kinects[i].shutDown();
	}
}