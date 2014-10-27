
/*
PEOPLE DETECTION: IMAGE PLANE
*/

#include "PplDetection_v3.h"
//#include <vld.h>



PplDetection_v3::PplDetection_v3(void)
{
}


PplDetection_v3::~PplDetection_v3(void)
{
}


void PplDetection_v3::writeTrackingResults(vector<TrackInfo>& tracks)
{
	//ofstream outGt ("d:\\Emilio\\Tracking\\DataSet\\sb125\\SecondDay\\DSet1\\Detections_ellipses_v3.txt");
	ofstream outGt ("c:\\Dropbox\\PhD\\Matlab\\DetectionEval\\IPS\\Results\\Detections_ellipses_IPS_NoDepth.txt");
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

void pointSelection_onMouse(int event, int x, int y, int flags, void* param)
{
	
	if (event == CV_EVENT_FLAG_LBUTTON)
	{
		//Mat* img = (Mat*)param;
		const XnDepthPixel** p = (const XnDepthPixel**)param;
		if (x != -1 && y != -1)
		{
			int d = (*p)[y*XN_VGA_X_RES+ x];
			cout << "Point selected. x: " << x << ". y: " << y << ". Depth:" << d << endl;;
			
		}
	}
}



void PplDetection_v3::detection(int fromVideo, int recordOut, int tilt, int debug)
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
	Mat background = Mat(actMapCreator.getResolution(), CV_8UC3);

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

	//calculate mask out regions in middle kinect
	Rect rLeft, rRight;
	rLeft = calculateRect(&kinects[0], &kinects[1], true);
	rRight = calculateRect(&kinects[2], &kinects[1], false);

	bool first = true;

	Mat* outMoA;
	Mat outMoAScaled;

	
	int waitTime = 1;

	
	VideoWriter w1;
	recordOut = 0;
	if (recordOut == 1)
	{
		w1.open("d:\\Emilio\\Tracking\\DataSet\\sb125\\SecondDay\\DSet1\\MoA_tracking_V1.mpg",CV_FOURCC('P','I','M','1'), 20.0, actMapCreator.getResolution(), true);
	}	

	char* nWindows[NUM_SENSORS];
	nWindows[0] =  "rgb 0";
	nWindows[1] =  "rgb 1";
	nWindows[2] =  "rgb 2";

	char* fWindows[NUM_SENSORS];
	fWindows[0] =  "fore 0";
	fWindows[1] =  "fore 1";
	fWindows[2] =  "fore 2";

	char* windMoA = "Activity Map";
	clock_t startTime = clock();
	clock_t startTotalTime = clock();
	clock_t startTime_tmp;
	int nPoints = 0;
	int frames = 0;
	int debugFrame = -1;

	//DEbug for the detection part
	bool print = false; //used to control when to print
	bool trainingDset = false; //Print out CC values
	vector<int> trainDsetValues(50);

	while (!bShouldStop && frames < 1000)
	{		
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
				whiteBack = Mat::zeros(actMapCreator.getResolution(), CV_8UC3);
				activityMap = new Mat(actMapCreator.getResolution(), CV_8UC3);
				activityMap_Back = new Mat(actMapCreator.getResolution(), CV_8UC3);
				Utils::initMat3u(whiteBack, 255);
				first = false;
			}
			whiteBack.copyTo(*activityMap);
			background.copyTo(*activityMap_Back);

			
			PersonIPS dtctPpl[NUM_SENSORS][MAX_PEOPLE];
			int ttl_dtctPpl[NUM_SENSORS] = {0,0,0};

			for (int i = 0; i < NUM_SENSORS; i++)
			{
				startTime_tmp = clock(); //time debuggin
				numberOfForegroundPoints[i] = subtractors[i].subtraction(pointsFore2D[i], &(depthMat[i]), &(masks[i]));
				foreImages[i] = Mat::zeros(XN_VGA_Y_RES, XN_VGA_X_RES, CV_8UC1);
				updateForegroundImg(foreImages[i], pointsFore2D[i], numberOfForegroundPoints[i]);
				totalIntervals[BSUB_ID] += clock() - startTime_tmp; //time debugging
				nPoints += numberOfForegroundPoints[i];

				std::vector < std::vector<XnPoint3D> > blobs;
				threshold(foreImages[i], foreImages[i], 0, 1, THRESH_BINARY);
				findBlobsII(foreImages[i], blobs, depthMaps[i]);
				std::vector < std::vector<XnPoint3D> > blobsFilter;
				thresholdSmallBlobs(blobs, blobsFilter);
				//for debugging purposes
				foreImages[i] = Mat::zeros(XN_VGA_Y_RES, XN_VGA_X_RES, CV_8UC3);
					
				if (debug >= DEBUG_HIGH && (frames == 291 || frames == 294) && i == 0)
				{
					updateForegroundImgII(foreImages[i], blobsFilter, depthMaps[i], print);
					//save foeground image and RGB
					if (frames == 291)
					{
						imwrite("c:\\Dropbox\\PhD\\Individual Studies\\PhD\\Thesis\\Chapter4\\imgs\\IPS_seg\\fore0_Right_291_Bef.jpg", foreImages[i]);
						imwrite("c:\\Dropbox\\PhD\\Individual Studies\\PhD\\Thesis\\Chapter4\\imgs\\IPS_seg\\RGB0_Right_291_Bef.jpg", rgbImages[i]);
					}
					else if (frames == 294)
					{
						imwrite("c:\\Dropbox\\PhD\\Individual Studies\\PhD\\Thesis\\Chapter4\\imgs\\IPS_seg\\fore0_Miss_294_Bef.jpg", foreImages[i]);
						imwrite("c:\\Dropbox\\PhD\\Individual Studies\\PhD\\Thesis\\Chapter4\\imgs\\IPS_seg\\RGB0_Miss_294_Bef.jpg", rgbImages[i]);
					}

					imshow(fWindows[i], foreImages[i]);
					waitKey(0);
				}

				for (int k = 0; k < blobsFilter.size(); k++)
				{
					std::vector<XnPoint3D> *blob = &(blobsFilter[k]);
					int ttlPnts = blob->size();
					XnPoint3D* p2dD = &((*blob)[0]);
					XnPoint3D* p3d = new XnPoint3D[ttlPnts];
					kinects[i].arrayBackProject(p2dD, p3d, ttlPnts);
					kinects[i].tiltCorrection(p3d, ttlPnts);
					std::vector < std::vector<XnPoint3D> > subBlobs2dD;
					std::vector < std::vector<XnPoint3D> > subBlobs3d;
					//any == true if there are at leat one blob
					bool any = look4MergesBlob(p2dD, p3d, ttlPnts, subBlobs2dD, subBlobs3d, false, k, frames, i);
					//bool any = true;
					int ttlSubB = subBlobs2dD.size();
					int idP = 0;
					if (any && ttlSubB > 0)
					{
						for (int s = 0; s < ttlSubB; s++)
						{
							XnPoint3D* sb2dD = &(subBlobs2dD[s][0]);
							XnPoint3D* sb3d = &(subBlobs3d[s][0]);
							int ttlSubPnts = subBlobs2dD[s].size();
							detectPeople(depthMaps[i], sb2dD, sb3d, ttlSubPnts, kinects[i], dtctPpl[i], ttl_dtctPpl[i], i, foreImages[i], false, idP, rLeft, rRight);
						}
					}
					else if (any && ttlSubB == 0) //if there are at least 1
					{
						//treat it as single blob
						detectPeople(depthMaps[i], p2dD, p3d, ttlPnts, kinects[i], dtctPpl[i], ttl_dtctPpl[i], i, foreImages[i], false, idP, rLeft, rRight);
					}

					//generate tracks history
					//generateTrackHistoryIPS(tracks, dtctPpl[i], ttl_dtctPpl[i] , frames);	

				}
				//debug: images post-processed (in the detectPeople function tenth parameter true
				//if (frames == 264 && i == 2)
				//{
				//	//save foeground image and RGB
				//	imwrite("c:\\Dropbox\\PhD\\Individual Studies\\PhD\\Thesis\\Chapter4\\imgs\\IPS_seg\\sreenshots\\Example6\\fore0_Fail_264_Aft.jpg", foreImages[i]);
				//	imwrite("c:\\Dropbox\\PhD\\Individual Studies\\PhD\\Thesis\\Chapter4\\imgs\\IPS_seg\\sreenshots\\Example6\\RGB0_Right_264_Aft.jpg", rgbImages[i]);
				//}
				//else if (frames == 294)
				//{
				//	//save foeground image and RGB
				//	//imwrite("c:\\Dropbox\\PhD\\Individual Studies\\PhD\\Thesis\\Chapter4\\imgs\\IPS_seg\\fore0_Miss_294_Aft.jpg", foreImages[i]);
				//	//imwrite("c:\\Dropbox\\PhD\\Individual Studies\\PhD\\Thesis\\Chapter4\\imgs\\IPS_seg\\RGB0_Miss_294_Aft.jpg", rgbImages[i]);
				//}
				//imshow(fWindows[i], foreImages[i]);
				//waitKey(0);
			}
			if (debug >= DEBUG_NONE && nPoints > 0)
			{
				for (int i = 0; i < NUM_SENSORS; i++)
				{
					startTime_tmp = clock();
					points3D[i] = new XnPoint3D[numberOfForegroundPoints[i]];
					kinects[i].arrayBackProject(pointsFore2D[i], points3D[i], numberOfForegroundPoints[i]);
					totalIntervals[BACKPR_ID] += clock() - startTime_tmp; //time debugging

					startTime_tmp = clock();
					kinects[i].transformArray(points3D[i], numberOfForegroundPoints[i]);
					totalIntervals[PTRANS_ID] += clock() - startTime_tmp; //time debugging

				
					startTime_tmp = clock();
					updateActivityMapII(*activityMap, *activityMap_Back, &actMapCreator, points3D[i], numberOfForegroundPoints[i], pointsFore2D[i]);
					totalIntervals[MOA_ID] += clock() - startTime_tmp; //time debugging

					displayDetectedPplIPS(dtctPpl[i], ttl_dtctPpl[i], rgbImages[i], *activityMap);

					delete []points3D[i];
				}
			}
			
			outMoA = activityMap;
			if (!deleteBG)
					outMoA = activityMap_Back;

			if (recordOut == 1)
				w1 << *activityMap;

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

		if (debug >= DEBUG_NONE)
		{
			imshow(nWindows[0], rgbImages[0]);
			imshow(nWindows[1], rgbImages[1]);
			imshow(nWindows[2], rgbImages[2]);
		}

		/*if (frames == 264)
			imwrite("c:\\Dropbox\\PhD\\Individual Studies\\PhD\\Thesis\\Chapter4\\imgs\\IPS_seg\\sreenshots\\Example6\\RGB0_Right_264_Ellipses.jpg", rgbImages[2]);
*/

		int c = waitKey(waitTime);
		if (c == 13)
		{
			waitTime = !waitTime;

			//imwrite("d:/Emilio/Tracking/DataSet/sb125/SecondDay/DSet1/captures/fore0.jpg", foreImages[0]);
			//imwrite("d:/Emilio/Tracking/DataSet/sb125/SecondDay/DSet1/captures/fore1.jpg", foreImages[1]);
			//imwrite("d:/Emilio/Tracking/DataSet/sb125/SecondDay/DSet1/captures/fore2.jpg", foreImages[2]);
			//imwrite("d:/Emilio/Tracking/DataSet/sb125/SecondDay/DSet1/captures/MoA.jpg", *outMoA);

		}
		else if (c == 27)
			bShouldStop = true;
		else if (c == 99) // c
		{
			imwrite("c:\\Dropbox\\PhD\\Individual Studies\\PhD\\Thesis\\Chapter4\\imgs\\IPS_seg\\RGB_ellipses0.jpg", rgbImages[0]);
			imwrite("c:\\Dropbox\\PhD\\Individual Studies\\PhD\\Thesis\\Chapter4\\imgs\\IPS_seg\\RGB_ellipses1.jpg", rgbImages[1]);
			imwrite("c:\\Dropbox\\PhD\\Individual Studies\\PhD\\Thesis\\Chapter4\\imgs\\IPS_seg\\RGB_ellipses2.jpg", rgbImages[2]);
			imwrite("c:\\Dropbox\\PhD\\Individual Studies\\PhD\\Thesis\\Chapter4\\imgs\\IPS_seg\\MoA_ellipses.jpg", *outMoA);
		}

		frames++;
	}

	for (int i = 0; i < NUM_SENSORS; i++)
		delete []pointsFore2D[i];


	delete activityMap;
	delete activityMap_Back;

	//writeTrackingResults(tracks);


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