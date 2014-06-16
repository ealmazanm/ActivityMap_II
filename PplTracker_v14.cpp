#include "PplTracker_v14.h"
#include <vld.h>


PplTracker_v14::PplTracker_v14(void)
{
	kin = -1;
}


PplTracker_v14::~PplTracker_v14(void)
{
}

void PplTracker_v14::pointSelectionHeight_onMouse(int event, int x, int y, int flags, void* param)
{
	if (event == CV_EVENT_FLAG_LBUTTON)
	{
		int cam = *((int*)param);
		if (x != -1 && y != -1)
		{
			kin = cam;
			p.X = x;
			p.Y = y;
			/*int depth = depthMaps[cam][y*XN_VGA_X_RES+x];
			if (depth != 0)
			{
				XnPoint3D p2D;
				XnPoint3D p3D;
				p2D.X = x; p2D.Y = y; p2D.Z = depth;
				kinects[cam].arrayBackProject(&p2D, &p3D, 1);
				kinects[cam].transformArray(&p3D,1);
				cout << "Height: " << p3D.Y << endl;
			}*/
		}
	}
}


void PplTracker_v14::writeTrackingResults(vector<TrackInfo>& tracks)
{
	//ofstream outGt ("d:\\Emilio\\Tracking\\DataSet\\sb125\\SecondDay\\DSet1\\Tracks_ellipses_v14.txt");
	ofstream outGt ("c:\\Dropbox\\PhD\\Matlab\\TrackingEval\\KingstonEvalTool\\TBE_Ellipse\\ST_V.1.4\\Results\\Spatial\\Update0\\Tracks_ellipses_SNN_spatial.txt");
	//ofstream outGt ("c:\\Dropbox\\PhD\\Matlab\\TrackingEval\\KingstonEvalTool\\TBE_Ellipse\\ST_V.1.4\\Results\\Tracks_ellipses_Random.txt");
	
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


void PplTracker_v14::trackingMoA(int fromVideo, int recordOut, int tilt, int debug)
{

	ofstream outDtcAreas; //( "c:\\Dropbox\\PhD\\Matlab\\DataAssociation\\Dset1\\AreaProxResults\\mergeMeasurementsST_115.txt");

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
	
	//RANGE_STEP = cei;
	//SCALE_RANGE_ALT = MAX_RANGE/(((0.33*logf(10+3*MAX_RANGE/1000))-0.80597));



	//namedWindow(windMoA);
	Mat *activityMap, *activityMap_Back;
	Mat whiteBack, colorMap;
	Mat background = Mat(actMapCreator.getResolution(), CV_8UC3);
	Mat backgroundPolar = Mat(actMapCreator.getResolution().height+150, 181, CV_8UC3);

	//flags
	bool bShouldStop = false;
	bool trans = true;
	bool bgComplete = true;
	bool deleteBG = true;

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
		depthImages[i] = Mat(XN_VGA_Y_RES, XN_VGA_X_RES, CV_8UC3);
		rgbImages[i] = Mat(XN_VGA_Y_RES, XN_VGA_X_RES, CV_8UC3);
		depthMat[i] = Mat(XN_VGA_Y_RES, XN_VGA_X_RES, CV_16U);
		pointsFore2D[i] = new XnPoint3D[MAX_FORGROUND_POINTS];
		numberOfForegroundPoints[i] = 0;
	}

	bool first = true;

	int TotalFrames_4 = 1670;
	int TotalFrames_2 = 850;

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
	//w1.open("d:\\Emilio\\Tracking\\DataSet\\sb125\\SecondDay\\DSet1\\MoA_tracking_V14_SNN_APP.mpg",CV_FOURCC('P','I','M','1'), 20.0, actMapCreator.getResolution(), true);
	recordOut = 0;
	if (recordOut == 1)
	{
		w.open("d:\\Emilio\\Tracking\\DataSet\\sb125\\SecondDay\\DSet2\\RemapPolarSpace_Detection.avi",CV_FOURCC('P','I','M','1'), 20.0, polarAlt_smooth_.size(), true);
		w1.open("d:\\Emilio\\Tracking\\DataSet\\sb125\\SecondDay\\DSet2\\MoA_tracking_V1.mpg",CV_FOURCC('P','I','M','1'), 20.0, actMapCreator.getResolution(), true);
	}	

	//Size of kernel: smooth rps;
	Mat kernel = Mat::ones(Size(5,27), CV_32F);

	Rect rLeft, rRight;
	rLeft = calculateRect(&kinects[0], &kinects[1], true);
	rRight = calculateRect(&kinects[2], &kinects[1], false);


	Person* trckPpl = new Person[MAX_PEOPLE];
	int ttl_trckPpl = 0;

	char* nWindows[NUM_SENSORS];
	nWindows[0] =  "rgb 0";
	nWindows[1] =  "rgb 1";
	nWindows[2] =  "rgb 2";

	//namedWindow(nWindows[0]);
	//namedWindow(nWindows[1]);
	//namedWindow(nWindows[2]);
	//int cam0 = 0;
	//int cam1 = 1;
	//int cam2 = 2;
	//cvSetMouseCallback(nWindows[0], pointSelectionHeight_onMouse, &cam0);
	//cvSetMouseCallback(nWindows[1], pointSelectionHeight_onMouse, &cam1);
	//cvSetMouseCallback(nWindows[2], pointSelectionHeight_onMouse, &cam2);


	char* windMoA = "Activity Map";
	clock_t startTime = clock();
	clock_t startTotalTime = clock();
	clock_t startTime_tmp;
	int nPoints = 0;
	int frames = 0;
	int debugFrame = -1;

	vector<PointMapping>* pntsMap2 = new vector<PointMapping>[polarAlt.rows*polarAlt.cols];
	vector<DCF> DCFs;
	bool merged = false;
	while (!bShouldStop && frames < 1000)
	{		
		if (frames == 30)
			cout << "stop" << endl;

		printf("\rFram %d", frames);

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
				whiteBack = Mat::Mat(actMapCreator.getResolution(), CV_8UC3);
				activityMap = new Mat(actMapCreator.getResolution(), CV_8UC3);
				activityMap_Back = new Mat(actMapCreator.getResolution(), CV_8UC3);
				Utils::initMat3u(whiteBack, 255);
				first = false;
			}
			whiteBack.copyTo(*activityMap);
			background.copyTo(*activityMap_Back);
			for (int i = 0; i < NUM_SENSORS; i++)
			{
				startTime_tmp = clock(); //time debuggin
				numberOfForegroundPoints[i] = subtractors[i].subtraction(pointsFore2D[i], &(depthMat[i]), &(masks[i]));
				totalIntervals[BSUB_ID] += clock() - startTime_tmp; //time debugging
				nPoints += numberOfForegroundPoints[i];
				if (debug >= DEBUG_HIGH)//Draw the output of the foreground detection
				{
					if (frames == -1 && i == 1)
						imwrite("c:/Dropbox/Phd/Matlab/Model/rgb_1_Orig.jpg", rgbImages[1]);
					for (int c = 0; c < numberOfForegroundPoints[i]; c++)
					{
						XnPoint3D* p = &(pointsFore2D[i][c]); 
						uchar* ptr = rgbImages[i].ptr<uchar>(p->Y);
						ptr[3*(int)p->X] = 0;
						ptr[(3*(int)p->X)+1] = 0;
						ptr[(3*(int)p->X)+2] = 255;

					}
					if (frames == -1 && i == 1)
						imwrite("c:/Dropbox/Phd/Matlab/Model/rgb_1_Red.jpg", rgbImages[1]);
				}
			}
			if (nPoints > 0)
			{
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
					startTime_tmp = clock();
					updatePolarAlternateive(&polarAlt, &polar, pntsMap2, ttlPnts, points3D[i], pointsFore2D[i], rgbMaps[i], numberOfForegroundPoints[i], debug, i);	
					totalIntervals[RPSPACE_ID] += clock() - startTime_tmp; //time debugging

					if (debug >= DEBUG_NONE)
					{
						startTime_tmp = clock();
						updateActivityMap(*activityMap, *activityMap_Back, &actMapCreator, points3D[i], numberOfForegroundPoints[i], pointsFore2D[i]);
						totalIntervals[MOA_ID] += clock() - startTime_tmp; //time debugging
					}
					
				}
		
				//Todo: Create a method detection(polarAlt, moAPeople)
				startTime_tmp = clock();
				cv::filter2D(polarAlt, polarAlt_smooth, -1, kernel);
				totalIntervals[SMOOTH_ID] += clock() - startTime_tmp; //time debugging

				startTime_tmp = clock();
				ccDetection(polarAlt_smooth, dtctPpl, ttl_dtctPpl, pntsMap2, ttlPnts, debug, frames, debugFrame); //Connected component detection
				totalIntervals[DET_ID] += clock() - startTime_tmp; //time debugging


				startTime_tmp = clock();
				tracking_GNN(trckPpl, ttl_trckPpl, dtctPpl, ttl_dtctPpl, activityMap, DCFs, debug, frames, outDtcAreas);
				totalIntervals[TRACK_ID] += clock() - startTime_tmp; //time debugging

				//generate tracks history
				generateTrackHistory(tracks, trckPpl, ttl_trckPpl, frames);				

				//For display purposes
				if (debug >= DEBUG_MED)
				{
					Utils::convert16to8(&polarAlt_smooth, polarAlt_smooth_);
					Utils::convert16to8(&polarAlt, polarAlt_);
					Utils::convert16to8(&polar, polar_);
					displayTrackersRPS(trckPpl, ttl_trckPpl, polarAlt_smooth_, debug);
					imshow("Polar Alt", polar_);
					imshow("Polar Alt Smooth_", polarAlt_smooth_);
					imshow("Polar Alt", polarAlt_);
				}
				Mat *tmp = activityMap;
				if (!deleteBG)
					tmp = activityMap_Back;
				

				if (debug >= DEBUG_MED && ttl_trckPpl > 0)
				{
					Person* prs =  &trckPpl[0];
					int c = 1;
					while (prs->id != 3 && c < ttl_trckPpl)
					{
						prs = &trckPpl[c];
						c++;
					}
					if (prs->id == 0)
					{
						//outDebugFile << "Frame: " << frames << endl;
						//outDebugFile << "SigmaX,Y_rps: " << prs->sigmaX_RPS << ", " << prs->sigmaY_RPS << endl;
						//Utils::printValuesF(&prs->covMoA, "Cov MoA", outDebugFile);
						//printValuesF(&prs->A, "Motion Model personid 0", outDebugFile);
						//printValuesF(&prs->stateMoA, "Person state id 0", outDebugFile);
						//printValuesF(&prs->stateUncertainty, "Uncertainty of the state(updated)", outDebugFile);
						//printValuesF(&prs->Q, "Motion model error prs Id 0", outDebugFile);
						//printValuesF(&prs->R, "Person measurement error id 0", outDebugFile);
						//printValuesF(&prs->K, "Kalman Gain prs id 0", outDebugFile);
					}
				}
				startTime_tmp = clock();
				displayTrackersMoA(trckPpl, ttl_trckPpl, *tmp, debug, frames);
				//displayPredictionsTrackersMoA(trckPpl, ttl_trckPpl, *tmp, debug, frames);
				//displayDetections(dtctPpl, ttl_dtctPpl, polarAlt_smooth_, *tmp, debug);
				//merged = displayMergeMeasurements(trckPpl, ttl_trckPpl, dtctPpl, ttl_dtctPpl, *tmp, debug, frames);
				totalIntervals[DISPLAY_ID] += clock() - startTime_tmp; //time debugging
				
				if (debug >= DEBUG_HIGH)//shows the apperance model
				{
						Person* trgt = NULL;
						bool found = false;
						int i = 0;
						while ( i < ttl_trckPpl && !found)
						{
							trgt =  &(trckPpl[i]);
							found = trgt->id == 19;
							i++;
						}
						if (found)
						{
							updateAppearanceImg(trgt);
							imshow("AppearanceMdl", trgt->apperanceRed);
						}
					
				}

				delete [] dtctPpl;
				ttl_dtctPpl = 0;
				for (int i = 0; i < NUM_SENSORS; i++)
				{
					delete []points3D[i];
				}
			}
			outMoA = activityMap;
			if (!deleteBG)
					outMoA = activityMap_Back;

			if (debug >= DEBUG_HIGH)
				w1 << *activityMap;

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
			//if (recordOut == 1 && trans)
			//		w << background;
		}

		if (DEPTH_SCALE < 1)
		{			
			resize(*outMoA, outMoAScaled, Size(outMoA->cols/DEPTH_SCALE, outMoA->rows/DEPTH_SCALE), 0,0, INTER_LINEAR);
			outMoA = &outMoAScaled;
		}
		imshow(windMoA, *outMoA);

		if (debug >= DEBUG_NONE)
		{
			//imshow("Polar Alt", polar_);
			//imshow("Polar Alt Smooth_", polarAlt_smooth_);
			//imshow("Polar Alt", polarAlt_);
			imshow(nWindows[0], rgbImages[0]);
			imshow(nWindows[1], rgbImages[1]);
			imshow(nWindows[2], rgbImages[2]);
		}

		//if (frames == 397)
		//	waitTime = 0;
		
		if (debug >= DEBUG_HIGH)
		{
			const Person* p = NULL;
			bool found = false;
			int i = 0;
			while ( i < ttl_trckPpl && !found)
			{
				p =  &(trckPpl[i]);
				found = p->id == 19;
				i++;
			}
			if (found && p != NULL)
			{
				outDebugFile << "Frame " << frames << ". Trgt Id: " << p->id << ". Dtct Id: " << p->idDetection << endl;
			}
		}


		if(debug >= DEBUG_HIGH && frames > 710 && frames < 735)
		{
				char* common = "c:\\Dropbox\\PhD\\Matlab\\Model\\Appearance\\GNN\\UPDATE\\";
				//Capture Map of Activity
				char path[150];
				strcpy(path, common);
				char frStr[15];
				itoa(frames, frStr, 10);
				strcat(path, "MoA_K0");
				strcat(path, frStr);
				strcat(path, ".jpg");
				imwrite(path, *activityMap);

				//Capture RGB images
				strcpy(path, common);
				strcat(path, "rgb_K0_");
				strcat(path, frStr);
				strcat(path, ".jpg");
				imwrite(path, rgbImages[0]);

				////capture modles 43
				//const Person* p = NULL;
				//bool found = false;
				//int i = 0;
				//while ( i < ttl_trckPpl && !found)
				//{
				//	p =  &(trckPpl[i]);
				//	found = p->id == 42;
				//	i++;
				//}
				//if (found && p != NULL)
				//{
				//	strcpy(path, common);
				//	strcat(path, "rgbApp_");
				//	strcat(path, frStr);	
				//	strcat(path, "id43.jpg");
				//	imwrite(path, p->apperanceRed);

				//	strcpy(path, common);
				//	strcat(path, "rgbApp_");
				//	strcat(path, frStr);	
				//	strcat(path, "id43_dtc.jpg");
				//	imwrite(path, p->apperanceRed_Dtc);
				//}
				////capture model tgt 38
				//p = NULL;
				//found = false;
				//i = 0;
				//while ( i < ttl_trckPpl && !found)
				//{
				//	p =  &(trckPpl[i]);
				//	found = p->id == 38;
				//	i++;
				//}
				//if (found && p != NULL)
				//{
				//	strcpy(path, common);
				//	strcat(path, "rgbApp_");
				//	strcat(path, frStr);	
				//	strcat(path, "id38.jpg");
				//	imwrite(path, p->apperanceRed);

				//	strcpy(path, common);
				//	strcat(path, "rgbApp_");
				//	strcat(path, frStr);	
				//	strcat(path, "id38_dtc.jpg");
				//	imwrite(path, p->apperanceRed_Dtc);
				//}
				
		}

		int c = waitKey(waitTime);
		
		switch (c)
		{
		case 115: //s (screen capture)
			{
					cout << "sdfa" << endl;

				//Register the similarity matrix
			}
		case 32:
			{
				saved = true;;
				break;
			}
		case 27: //esc
			{
				bShouldStop = true;
				break;
			}

		case 99: //c
			{
				//bgComplete = true;
				if (debug > DEBUG_HIGH)
				{
					const Person* p = NULL;
					bool found = false;
					int i = 0;
					while ( i < ttl_trckPpl && !found)
					{
						p =  &(trckPpl[i]);
						found = p->id == 19;
						i++;
					}
					if (found && p != NULL)
					{
						imwrite("c:\\Dropbox\\PhD\\Matlab\\Model\\Appearance\\GNN\\WRONG\\rgb.jpg", rgbImages[1]);
						imwrite("c:\\Dropbox\\PhD\\Matlab\\Model\\Appearance\\GNN\\WRONG\\rgb_Mdl.jpg", p->apperanceRed);
						for (int i = 0; i < MODEL_NBINS; i++)
						{
							int v = p->heightModel[i];
							if (v > MIN_NUM_POINTS_BIN)
							{
								gaussianParam gp = p->colourModel[i];
								modelOut << frames << " " << p->id << " " << i << " " << v << " " << gp.mean.val[0] << " " << gp.mean.val[1] << " " << gp.mean.val[2] << " ";
					
								modelOut << gp.cov.at<float>(0,0) << " " << gp.cov.at<float>(0,1) << " " << gp.cov.at<float>(0,2) << " " << gp.cov.at<float>(1,1) <<
										" " << gp.cov.at<float>(1,2) << " " << gp.cov.at<float>(2,2) << endl;
							}
							else
							{
								modelOut << frames << " " << p->id << " " << i << " " << -1 << " " << -1 << " " << -1 << " " << -1 << " " << -1 << " " << -1 << " " << -1 << " " << -1 << " " << -1 << " " << -1 << endl;
							}
				
						}
					}
				}
					
				break;
			}
		case 116: //t
			{
				//trans = !trans;
				if (debug >= DEBUG_HIGH)
				{
					const Person* p = NULL;
					bool found = false;
					int i = 0;
					while ( i < ttl_trckPpl && !found)
					{
						p =  &(trckPpl[i]);
						found = p->id == 0;
						i++;
					}
					if (found && p != NULL)
					{
						imwrite("c:\\Dropbox\\PhD\\Matlab\\Model\\rgb_blob.jpg", rgbImages[0]);
						imwrite("c:\\Dropbox\\PhD\\Matlab\\Model\\BW_blob.jpg", p->bwImg);
						imwrite("c:\\Dropbox\\PhD\\Matlab\\Model\\MoA_Detection.jpg", *activityMap);
					}
				}

				break;		
			}
		case 100: //d
			{
				deleteBG = !deleteBG;
			}
		case 13: //enter
			{
				//debugFrame = frames + 1;
				//imwrite("c:/Dropbox/Phd/Individual Studies/Problems/MoA_Tilting.jpg", *outMoA);
				//imwrite("c:/Dropbox/Phd/Matlab/Model/rgb_1.jpg", rgbImages[1]);
				//waitTime = 1;
				if (waitTime == 0)
					waitTime=1;
				else
				//{
				//	//imwrite("c:/Dropbox/Phd/Individual Studies/KinectDepthSensor/AlternativeSpace/MoA_Detection_Good.jpg", *activityMap);
					waitTime=0;
				//}
				

				
				break;
			}
		}

		//save current people
		//for (int i = 0; i < ttl_dtctPpl; i++)
		//{
		//	ttlPastPpl = ttl_dtctPpl;
		//	copyPerson(pastPpl[i], &dtctPpl[i]);
			//pastPpl[i] = dtctPpl[i];
		//}
		//if (frames == 465)
		//{
			//bShouldStop = true;
		//	debugFrame = frames;
			//waitTime = 0;
		//}
		frames++;
	}
	//Generate the occlusion periods
	//writeXMLDCFs(DCFs);

	//list<Merge> mergeList, mergeListPost;
	//detectMerges(DCFs, mergeList);
	//writeXMLMerges(mergeList);

	delete []pntsMap2;

	for (int i = 0; i < NUM_SENSORS; i++)
	{
		delete []pointsFore2D[i];
	}

	delete activityMap;
	delete activityMap_Back;

	writeTrackingResults(tracks);

	////write on files the tracks for posterior evaluation
	//char* path = "c:/Dropbox/PhD/Matlab/TrackingEval/CLEAT-MOT-script/ST/st";
	//char idStr[15];
	//char pathTmp[150];	
	//for (int i = 0; i < tracks.size(); i++)
	//{
	//	Track trck = tracks[i];	

	//	itoa(trck.id, idStr, 2);
	//	strcpy(pathTmp, path);
	//	strcat(pathTmp, idStr);
	//	strcat(pathTmp, ".txt");
	//	ofstream outTrack(pathTmp);
	//	
	//	int totalFrames = trck.trajectory.size();
	//	for (int posId = 0; posId < totalFrames; posId++)
	//	{
	//		Position pos = trck.trajectory[posId];

	//		outTrack << pos.frameId << " " << pos.bbox.x << " " << pos.bbox.y << " " << pos.bbox.width << " " << pos.bbox.height << endl;
	//	}
	//	
	//}



	delete[] trckPpl;

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

	

	//outDtcAreas.close();
	for (int i = 0; i < NUM_SENSORS; i++)
	{
		kinects[i].stopDevice();
  		kinects[i].shutDown();
	}
}