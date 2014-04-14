#include "PplDetection_v1.h"


PplDetection_v1::PplDetection_v1(void)
{
}


PplDetection_v1::~PplDetection_v1(void)
{
}


void PplDetection_v1::detection(int fromVideo, int recordOut, int tilt, int debug)
{
	bool saved = false;
	if (fromVideo)
		cout << ". Live execution";
	else
		cout << ". Video execution";

	if (!recordOut)
		cout << ". No record" << endl;
	else
		cout << ". Record" << endl;
	if (debug == DEBUG_NONE)
		cout << ". Debug none" << endl;
	else if (debug == DEBUG_LOW)
		cout << ". Debug low" << endl;
	else if (debug == DEBUG_HIGH)
		cout << ". Debug high" << endl;

	ofstream outDtcAreas ("d:\\Emilio\\Tracking\\DataSet\\sb125\\SecondDay\\DSet2\\mergeMeasurementsST_85_det.txt");
	char* paths[3];
	paths[0] = "d:/Emilio/Tracking/DataSet/sb125/SecondDay/DSet2/kinect0_calib.oni";
	paths[1] = "d:/Emilio/Tracking/DataSet/sb125/SecondDay/DSet2/kinect1_calib.oni";
	paths[2] = "d:/Emilio/Tracking/DataSet/sb125/SecondDay/DSet2/kinect2_calib.oni";

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
	Mat backgroundPolar = Mat(actMapCreator.getResolution().height+150, 181, CV_8UC3);

	//flags
	bool bShouldStop = false;
	bool trans = true;
	bool bgComplete = false;
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
	//int TotalFrames_2 = 1790;
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
	recordOut = 0;
	if (recordOut == 1)
	{
		w.open("c:/Dropbox/Phd/Individual Studies/KinectDepthSensor/AlternativeSpace/RemapPolarSpace_Detection.mpg",CV_FOURCC('P','I','M','1'), 20.0, polarAlt_smooth_.size(), true);
		w1.open("c:/Dropbox/Phd/Individual Studies/KinectDepthSensor/AlternativeSpace/MoA_Detection.mpg",CV_FOURCC('P','I','M','1'), 20.0, actMapCreator.getResolution(), true);
	}	
	
	Rect rLeft, rRight;
	rLeft = calculateRect(&kinects[0], &kinects[1], true);
	rRight = calculateRect(&kinects[2], &kinects[1], false);

	//Size of kernel: smooth rps;
	Mat kernel = Mat::ones(Size(5,27), CV_32F);

	//list<Person> people;
	Person* dtctPpl = new Person[MAX_PEOPLE];
	int ttl_dtctPpl = 0;
	Person** trckPpl;
	int ttl_trckPpl = 0;
	//for detection to estimate the orientation of the ellipse
	Person* pastPpl = new Person[MAX_PEOPLE];
	int ttlPastPpl = 0;



	clock_t startTime = clock();
	clock_t startTotalTime = clock();
	clock_t startTime_tmp;
	int nPoints = 0;
	int frames = 0;
	int debugFrame = -1;
	while (!bShouldStop && frames < 1000)
	{		
		if (debug != DEBUG_NONE)
		{
			//outDebugFile << "Frame " << frames << endl;
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
			
			//Creates a matrxi with depth values (ushort)
			createDepthMatrix(depthMaps[i], depthMat[i]);

			//if (i == REF_CAM)
			//{
			//	startTime_tmp = clock(); //time debuggin
			//	//Mask out the complete ROI
			//	//maskOutOverlapping(depthMat[i], rLeft, rRight);
			//	//Selective Points Mask out 
			//	maskOutOverlappingPointSel(depthMat[i], rLeft, rRight, kinects);
			//	//Selective Mask out
			//	//maskOutOverlappingSelective(depthMat[i], rLeft, rRight, kinects);
			//	//updateDepthImage(depthImages[i], depthMat[i]);
			//	totalIntervals[MASKOVERLAPPING_ID] += clock() - startTime_tmp; //time debugging
			//}

			//to create a mask for the noise (depth img is threhold)
			cvtColor(depthImages[i],grey,CV_RGB2GRAY);
			masks[i] = grey > 250; //mask that identifies the noise (1)
		}

		nPoints = 0;
		if (bgComplete && trans) //Trans must be true
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
				if (debug >= DEBUG_MED)//Draw the output of the foreground detection
				{
					if (frames == debugFrame && i == 1)
						imwrite("c:/Dropbox/Phd/Matlab/Model/rgb_1_Orig.jpg", rgbImages[1]);
					for (int c = 0; c < numberOfForegroundPoints[i]; c++)
					{
						XnPoint3D* p = &(pointsFore2D[i][c]); 
						uchar* ptr = rgbImages[i].ptr<uchar>(p->Y);
						ptr[3*(int)p->X] = 0;
						ptr[(3*(int)p->X)+1] = 0;
						ptr[(3*(int)p->X)+2] = 255;
					}
					if (frames == debugFrame && i == 1)
						imwrite("c:/Dropbox/Phd/Matlab/Model/rgb_1_Red.jpg", rgbImages[1]);
				}
			}
			if (nPoints > 0)
			{
				//list of 3D points + colour map to the RP space
				PointMapping* pntsMap = new PointMapping[nPoints];
				int ttlPnts = 0;

				for (int i = 0; i < NUM_SENSORS; i++)
				{
					startTime_tmp = clock();
					points3D[i] = kinects[i].arrayBackProject(pointsFore2D[i], numberOfForegroundPoints[i]);
					totalIntervals[BACKPR_ID] += clock() - startTime_tmp; //time debugging

					if (debug == DEBUG_HIGH && frames == debugFrame)
					{
						int total = numberOfForegroundPoints[i];
						ofstream* out;
						if (i == 0)
							out = &outFPK0;
						else if (i == 1)
							out = &outFPK1;
						else
							out = &outFPK2;

						for (int j = 0; j < total; j++)
						{
							*out << (float)points3D[i][j].X << " " << (float)points3D[i][j].Y << " " << (float)points3D[i][j].Z << endl; 
							
						}
					}

					startTime_tmp = clock();
					kinects[i].transformArray(points3D[i], numberOfForegroundPoints[i]);
					totalIntervals[PTRANS_ID] += clock() - startTime_tmp; //time debugging
					
					//Create alternative representation
					startTime_tmp = clock();
					updatePolarAlternateive(&polarAlt, &polar, pntsMap, ttlPnts, points3D[i], pointsFore2D[i], rgbMaps[i], numberOfForegroundPoints[i], debug);	
					totalIntervals[RPSPACE_ID] += clock() - startTime_tmp; //time debugging
					
					startTime_tmp = clock();
					updateActivityMap(*activityMap, *activityMap_Back, &actMapCreator, points3D[i], numberOfForegroundPoints[i], pointsFore2D[i]);
					totalIntervals[MOA_ID] += clock() - startTime_tmp; //time debugging
				}
		
				//Todo: Create a method detection(polarAlt, moAPeople)
				startTime_tmp = clock();
				cv::filter2D(polarAlt, polarAlt_smooth, -1, kernel);
				totalIntervals[SMOOTH_ID] += clock() - startTime_tmp; //time debugging

				//outDebugFile << "Before detection" << endl;
				startTime_tmp = clock();
				ccDetection(polarAlt_smooth, dtctPpl, ttl_dtctPpl, pntsMap, ttlPnts, debug, frames, debugFrame); //Connected component detection
				totalIntervals[DET_ID] += clock() - startTime_tmp; //time debugging
				//outDebugFile << "After detection" << endl;

				if (debug >= DEBUG_NONE)
				{
					for (int i = 0; i < ttl_dtctPpl; i++)
					{
						Person rpsDtc = dtctPpl[i];
						float area = (2*rpsDtc.sigmaX_RPS) * (2*rpsDtc.sigmaY_RPS);
												
						Point meanRPS;
						meanRPS = rpsDtc.mean_RPS;
						double covX = rpsDtc.sigmaX_RPS*rpsDtc.sigmaX_RPS;
						double covY = rpsDtc.sigmaY_RPS*rpsDtc.sigmaY_RPS;
						double covXY = 0;
						Point meanMoA;
						meanMoA.x = rpsDtc.stateMoA.at<float>(0, 0);
						meanMoA.y = rpsDtc.stateMoA.at<float>(1, 0);
						double covXMoA = rpsDtc.covMoA_points.at<float>(0,0);
						double covYMoA = rpsDtc.covMoA_points.at<float>(1,1);
						double covXYMoA = rpsDtc.covMoA_points.at<float>(0,1);
						int merge = 0;
						if (area  > AREA_THRESHOLD)
							merge = 1;

						outDtcAreas << rpsDtc.id << " " << frames << " " << area << " " << meanRPS.x << " " << meanRPS.y << " "
							<< covX << " " << covY << " " << merge << " " << meanMoA.x << " " << meanMoA.y << " " 
							<< covXMoA << " " << covYMoA << " " << covXYMoA << endl;


					}
				}

				if (debug == DEBUG_HIGH)
				{
					for (int i = 0; i < ttl_trckPpl; i++)
					{
						Person*  p = trckPpl[i];
						printPerson(p);
					}
				}


				//For display purposes
				if (debug >= DEBUG_LOW)
				{
					Utils::convert16to8(&polarAlt_smooth, polarAlt_smooth_);
					Utils::convert16to8(&polarAlt, polarAlt_);
					Utils::convert16to8(&polar, polar_);
				}
				Mat *tmp = activityMap;
				if (!deleteBG)
					tmp = activityMap_Back;
				
							
				if (debug >= DEBUG_NONE && ttl_dtctPpl > 0)
				{
					Person* prs =  &dtctPpl[0];
					int c = 1;
					while (prs->id != 0)
					{
						prs = &dtctPpl[c];
						c++;
					}
					if (prs->id == 0)
					{
						outDebugFile << "Frame: " << frames << endl;
						outDebugFile << "SigmaX,Y_rps: " << prs->sigmaX_RPS << ", " << prs->sigmaY_RPS << endl;
						Utils::printValuesF(&prs->covMoA, "Cov MoA", outDebugFile);
					}
				}

				startTime_tmp = clock();
				//displayDetections(dtctPpl, ttl_dtctPpl, polarAlt_smooth_, pastPpl, ttlPastPpl, *tmp, debug);
				displayDetections(dtctPpl, ttl_dtctPpl, polarAlt_smooth_, *tmp, debug);
				totalIntervals[DISPLAY_ID] += clock() - startTime_tmp; //time debugging
				

				if (debug >= DEBUG_MED)//Show the comparison between the smoothed and the original remap polar space
				{
					Mat imgDebug = Mat(polarAlt_smooth_.size(), CV_8UC3);
					for (int i = 0; i < polarAlt_smooth_.rows; i++)
					{
						uchar* ptrDebug = imgDebug.ptr<uchar>(i);
						uchar* ptrPolarAlt = polarAlt_.ptr<uchar>(i);
						uchar* p = polarAlt_smooth_.ptr<uchar>(i);
						for (int j = 0; j < polarAlt_smooth_.cols; j++)
						{
							if (ptrPolarAlt[j] < 255 && p[j] < 255)
							{
								ptrDebug[j*3] = 0; ptrDebug[j*3+1] = 255; ptrDebug[j*3+2] = 0;
							}
							else if (ptrPolarAlt[j] < 255)
							{
								ptrDebug[j*3] = 255; ptrDebug[j*3+1] = 0; ptrDebug[j*3+2] = 0;
							}
							else if (p[j] < 255)
							{
								ptrDebug[j*3] = 0; ptrDebug[j*3+1] = 0; ptrDebug[j*3+2] = 255;
							}
						}
					}
					addGrid(polarAlt_smooth_, Size(kernel.cols, kernel.rows));
					imshow("Smooth comparison", imgDebug);
				}
				
				//free memory for the mapping
				delete []pntsMap;
			}
			outMoA = activityMap;
			if (!deleteBG)
					outMoA = activityMap_Back;

			if (recordOut == 1)
			{
				w << m;
				w1 << *activityMap;
			}
		}
		else
		{			
			startTime_tmp = clock();
			actMapCreator.createActivityMap(kinects, depthMaps, rgbMaps, trans, background, frames, MAX_RANGE, totalSubIntervalsMOA, MODEL_MAX_HEIGHT, MODEL_MIN_HEIGHT); 
			totalIntervals[MOA_ID] += clock() - startTime_tmp; //time debugging
			
			outMoA = &background;
			if (recordOut == 1 && trans)
					w << background;
		}

		if (DEPTH_SCALE < 1)
		{			
			resize(*outMoA, outMoAScaled, Size(outMoA->cols/DEPTH_SCALE, outMoA->rows/DEPTH_SCALE), 0,0, INTER_LINEAR);
			outMoA = &outMoAScaled;
		}
		imshow("Activity map", *outMoA);
	
		if (debug >= DEBUG_LOW)
		{
			imshow("Polar", polar_);
			imshow("Polar Alt Smooth_", polarAlt_smooth_);
			imshow("Polar Alt", polarAlt_);
			imshow("rgb0", rgbImages[0]);
			imshow("rgb1", rgbImages[1]);
			imshow("rgb2", rgbImages[2]);
		}
		int c = waitKey(waitTime);
		if (frames == 15) bgComplete = true;
		switch (c)
		{
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
				bgComplete = true;
				break;
			}
		case 116: //t
			{
				trans = !trans;
				break;		
			}
		case 100: //d
			{
				deleteBG = !deleteBG;
			}
		case 13: //enter
			{
				debugFrame = frames + 1;
				//imwrite("c:/Dropbox/Phd/Individual Studies/Problems/MoA_Tilting.jpg", *outMoA);
				//imwrite("c:/Dropbox/Phd/Matlab/Model/rgb_1.jpg", rgbImages[1]);
				
				//if (waitTime == 0)
				//	waitTime=1;
				//else
				//{
				//	//imwrite("c:/Dropbox/Phd/Individual Studies/KinectDepthSensor/AlternativeSpace/MoA_Detection_Good.jpg", *activityMap);
				//	waitTime=0;
				//}
				break;
			}
		}

		//save current people
		for (int i = 0; i < ttl_dtctPpl; i++)
		{
			ttlPastPpl = ttl_dtctPpl;
			//pastPpl[i].stateMoA.copyTo(dtctPpl[i].stateMoA);
			copyPerson(pastPpl[i], &dtctPpl[i]);
		}
		
		ttl_dtctPpl = 0;

	
		//bShouldStop = frames == 244;
		frames++;
	}

	for (int i = 0; i < ttl_trckPpl; i++)
		delete trckPpl[i];

	for (int i = 0; i < NUM_SENSORS; i++)
	{
		delete []pointsFore2D[i];
	}
	delete[] pastPpl;
	delete[] dtctPpl;
	delete activityMap;
	delete activityMap_Back;

	//delete[] trckPpl;

	totalIntervals[TOT_ID] = clock() - startTotalTime;
	//BUILD REPORT
	outDebugFile << "EXECUTION TIME REPORT" << endl;
	for (int i = 0; i < TOTAL_INTERVALS-1; i++)
	{
		float time_p = totalIntervals[i]*100/totalIntervals[TOT_ID];
		outDebugFile << titles[i] << ": " << time_p << " %" << endl;
	}

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