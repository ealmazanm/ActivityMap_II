#include "PplTracker_v2.h"
//#include <vld.h>

//Meanshift tracker
PplTracker_v2::PplTracker_v2(void)
{
	kin = -1;
}


PplTracker_v2::~PplTracker_v2(void)
{
}

void PplTracker_v2::writeTrackingResults(vector<TrackInfo>& tracks)
{
	//ofstream outGt ("d:\\Emilio\\Tracking\\DataSet\\sb125\\SecondDay\\DSet1\\Tracks_ellipses_v2.txt");
	ofstream outGt ("c:\\Dropbox\\PhD\\Matlab\\TrackingEval\\KingstonEvalTool\\TBE_Ellipse\\ST_V.2\\Results\\NoCleanAll\\Tracks_ellipses_MS_order.txt");
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


void PplTracker_v2::trackingMoA(int fromVideo, int recordOut, int tilt, int debug)
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
	bool* activePoints[NUM_SENSORS];
	Point* moa2DPoint[NUM_SENSORS];

	for (int i = 0; i < NUM_SENSORS; i++)
	{
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
		w.open("d:\\Emilio\\Tracking\\DataSet\\sb125\\SecondDay\\DSet2\\RemapPolarSpace_Detection.avi",CV_FOURCC('P','I','M','1'), 20.0, polarAlt_smooth_.size(), true);
		w1.open("d:\\Emilio\\Tracking\\DataSet\\sb125\\SecondDay\\DSet2\\MoA_tracking_V1.mpg",CV_FOURCC('P','I','M','1'), 20.0, actMapCreator.getResolution(), true);
	}	

	//Size of kernel: smooth rps;
	Mat kernel = Mat::ones(Size(5,27), CV_32F);

	Person* trckPpl = new Person[MAX_PEOPLE];
	int ttl_trckPpl = 0;
	TermCriteria term = TermCriteria(TermCriteria::MAX_ITER+TermCriteria::EPS, 50, 0.1);

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
	int tgtIdCounter = 0; 
	while (!bShouldStop && frames < 1000)
	{		
		printf("\rFram %d", frames);
		//outDebugFile << "Frame: " << frames << endl;
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
					activePoints[i] = new bool[numberOfForegroundPoints[i]];
					moa2DPoint[i] = new Point[numberOfForegroundPoints[i]];
					kinects[i].arrayBackProject(pointsFore2D[i], points3D[i], numberOfForegroundPoints[i]);
					totalIntervals[BACKPR_ID] += clock() - startTime_tmp; //time debugging

					startTime_tmp = clock();
					kinects[i].transformArray(points3D[i], numberOfForegroundPoints[i]);
					totalIntervals[PTRANS_ID] += clock() - startTime_tmp; //time debugging

					if (debug >= DEBUG_NONE)
					{
						startTime_tmp = clock();
						updateActivityMap(*activityMap, *activityMap_Back, &actMapCreator, points3D[i], numberOfForegroundPoints[i], pointsFore2D[i]);
						totalIntervals[MOA_ID] += clock() - startTime_tmp; //time debugging
					}

				}	

				if (debug >= DEBUG_HIGH && frames == 509)
				{
					Mat moaCpy;
					activityMap->copyTo(moaCpy);
					imshow("Debug MoA", moaCpy);
					waitKey(0);
					imwrite("c:\\Dropbox\\PhD\\Individual Studies\\PhD\\Thesis\\Chapter6\\imgs\\fail\\moa_298_original.jpg", moaCpy);
				}

				getMoA2DPoints(points3D, moa2DPoint, numberOfForegroundPoints);

				//if (frames == 411)
				//	waitTime = 0;

				if (ttl_trckPpl > 0)
				{
					//sort list by range
					Point2d origin= Point2d(activityMap->cols/2, activityMap->rows); //to compute range
					sortByRange(trckPpl, ttl_trckPpl, origin);
					//for debug
					Mat MoACopy;
					activityMap->copyTo(MoACopy);


					char* common = "c:\\Dropbox\\PhD\\Individual Studies\\PhD\\Thesis\\Chapter6\\imgs\\fail\\moa_trgt_";
				
					for (int i = 0; i < ttl_trckPpl; i++)
					{
						Person* trgt = &(trckPpl[i]);
						Mat MoAp = Mat::zeros(activityMap->size(), CV_32FC1);
					
						startTime_tmp = clock();
						createMoAp(trgt, points3D, moa2DPoint, numberOfForegroundPoints, pointsFore2D, rgbMaps, MoAp, frames, activePoints);
						totalIntervals[PROBMAP_ID] += clock() - startTime_tmp; //time debugging
				
						//DEBUG: Shows the actual performance of camshift
						if (debug >= DEBUG_HIGH && frames == 509)//( (frames >= 300 && trgt->id == 6) || (frames >= 300 && trgt->id == 8) ) )
						{
							//Capture Map of Activity
							char path[150];
							strcpy(path, common);
							char IDStr[15];
							itoa(trgt->id, IDStr, 10);
							strcat(path, IDStr);
							strcat(path, ".jpg");
				

							//Mat mImg = Mat::zeros(activityMap->size(), CV_8UC1);
							//Utils::convert16to8(&MoAp, mImg);
							//Rect r = getUncertainArea(trgt->rrMoA.boundingRect(), MoAp.size(), 0.25);
						
								/*outDebugFile << "**********Target " << i << ". Frame: " << frames << endl;
								float maxProb, ttlProb;
								maxProb = ttlProb = 0;
								getProbs(maxProb, ttlProb, MoAp, r);
								outDebugFile << "TtlProb: " << ttlProb << ". MaxProb: " << maxProb << endl;*/
						
							//rectangle(*activityMap, r, Scalar::all(0), 2);
							//rectangle(mImg, r, Scalar::all(0), 2);
							//imshow("Img", mImg);
							//imshow("...", *activityMap);
							//waitKey(0);
							camShift_custom(trgt, MoAp, term);
							//rectangle(mImg, trgt->rrMoA.boundingRect(), Scalar::all(0), 2);
												
							Mat m = (*activityMap)(trgt->rrMoA.boundingRect());
							Utils::initMat3u(m, 255);
							char txt[15];
							itoa(trgt->id, txt, 10);
							drawPersonPointsCov_debug(trgt->rrMoA, activityMap, trgt->colour, 2,txt);
							imshow("Debug MoA", *activityMap);
							waitKey(0);
							imwrite(path, *activityMap);
						}
						//else
						{
							startTime_tmp = clock();
							camShift_custom(trgt, MoAp, term);
							totalIntervals[TRACK_ID] += clock() - startTime_tmp; //time debugging
						}
						if (frames%25 == 0 && trgt->lost == 0)
						{
							//if (frames == 330 && trgt->id == 7)
							//	cout << "Stop" << endl;
							startTime_tmp = clock();
							//Build appearance model for the associated detection
							updateAppearanceDtct(trgt, points3D, moa2DPoint, numberOfForegroundPoints, pointsFore2D, rgbMaps, frames, debug);
							copyModel(trgt);
							//updateMShiftModel(trgt);
							totalIntervals[UPDATE_APP] += clock() - startTime_tmp; //time debugging

							if (debug > DEBUG_MED && frames == 200)
							{
								outDebugFile << "Target after update" << endl;
								int j = 1;
								outDebugFile << "At frame: " << frames << ". Bin: " << j; 
									Scalar cMean = trgt->colourModel[j].mean;
									outDebugFile << ". Error: Height Prob: " << trgt->heightModelPdf[j] << " Colour Mean: (" << cMean.val[0] << ", " << cMean.val[1] << ", " << cMean.val[2] << "). " << endl;
									printValuesF(&trgt->colourModel[j].cov , "Colour Cov", outDebugFile);
							}

							//updateMShiftModel(trgt);
						}
						//show MoaCopy
						if (debug >= DEBUG_HIGH && waitTime == 0)
						{
							displayTrackersMoA(trgt, 1, MoACopy, debug, frames);
							imshow("MoACopy", MoACopy);
							waitKey(0);
						}
						//Clean points with high probability of belonging to the target.
						//TODO: Get a sense of typical probability values (0 - 3e^-6)
						startTime_tmp = clock();
						cleanPointsTarget(trgt, moa2DPoint, activePoints, numberOfForegroundPoints, activityMap->size(), MoAp, MoACopy, debug); //Take out of debug after
						totalIntervals[CLEAN_ID] += clock() - startTime_tmp; //time debugging

						if (debug >= DEBUG_HIGH && waitTime == 0)
						{
							imshow("MoACopy", MoACopy);
							waitKey(0);
						}


					}
				}
				
				//int w = 1;
				//if (frames > 241)
				//	w = 0;
				//if (debug >= DEBUG_NONE && ttl_trckPpl > 0)
				//{
				//	/*Mat mImg = Mat::zeros(activityMap->size(), CV_8UC1) + 255;
				//	showActivePoints(mImg, moa2DPoint, activePoints, numberOfForegroundPoints, debug);
				//	for (int i = 0; i < ttl_trckPpl; i++)
				//		rectangle(mImg, trckPpl[i].rrMoA.boundingRect(), Scalar::all(0), 2);
				//	imshow("moat", mImg);
				//	waitKey(w);

				//	if (frames == 242)
				//		cout << "Stop" << endl;*/

				//	removeLostTracks(trckPpl, ttl_trckPpl);
				//	cleanPoints(trckPpl, ttl_trckPpl, moa2DPoint, activePoints, numberOfForegroundPoints, activityMap->size()); //Take out of debug after

				//	/*showActivePoints(mImg, moa2DPoint, activePoints, numberOfForegroundPoints, debug);
				//	imshow("moat", mImg);
				//	waitKey(w);*/
				//}

				startTime_tmp = clock();
				removeLostTracks(trckPpl, ttl_trckPpl);
				totalIntervals[REMOVE_ID] += clock() - startTime_tmp; //time debugging
				//startTime_tmp = clock();
				//cleanPoints(trckPpl, ttl_trckPpl, moa2DPoint, activePoints, numberOfForegroundPoints, activityMap->size()); //Take out of debug after
				//totalIntervals[CLEAN_ID] += clock() - startTime_tmp; //time debugging
				
				

				//Search for new peiple
				for (int i = 0; i < NUM_SENSORS; i++)
				{
					//Create alternative representation
					startTime_tmp = clock();
					updatePolarAlternateive(&polarAlt, &polar, pntsMap2, ttlPnts, points3D[i], pointsFore2D[i], rgbMaps[i], activePoints[i], numberOfForegroundPoints[i], debug, i);	
					totalIntervals[RPSPACE_ID] += clock() - startTime_tmp; //time debugging
				}
	
				startTime_tmp = clock();
				cv::filter2D(polarAlt, polarAlt_smooth, -1, kernel);
				totalIntervals[SMOOTH_ID] += clock() - startTime_tmp; //time debugging

				startTime_tmp = clock();
				ccDetection(polarAlt_smooth, dtctPpl, ttl_dtctPpl, pntsMap2, ttlPnts, debug, frames, debugFrame); //Connected component detection
				totalIntervals[DET_ID] += clock() - startTime_tmp; //time debugging

				if (ttl_dtctPpl > 0)
					addNewTracks(trckPpl, ttl_trckPpl, dtctPpl, ttl_dtctPpl, tgtIdCounter);
	
				//generate tracks history
				if (ttl_trckPpl > 0)
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
				
				startTime_tmp = clock();
				displayTrackersMoA(trckPpl, ttl_trckPpl, *tmp, debug, frames);
				//displayDetections(dtctPpl, ttl_dtctPpl, polarAlt_smooth_, *tmp, debug);
				totalIntervals[DISPLAY_ID] += clock() - startTime_tmp; //time debugging


				if (debug >= DEBUG_HIGH  && ttl_trckPpl > 0)
				{
					Person* trgt = &(trckPpl[0]);
					if (frames % 2 == 0)
					{
						updateAppearanceImg(trgt);
						updateAppearanceImgDtc(trgt);
						imshow("D", trgt->apperanceRedDtc);
					}
					
					imshow("AppearanceMdl", trgt->apperanceRed);
					//imshow("A2", trgt->apperance);
				}

				delete [] dtctPpl;
				ttl_dtctPpl = 0;
				for (int i = 0; i < NUM_SENSORS; i++)
				{
					delete []points3D[i];
					delete []activePoints[i];
					delete []moa2DPoint[i];
				}
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
			//if (recordOut == 1 && trans)
			//		w << background;
		}

		if (DEPTH_SCALE < 1)
		{			
			resize(*outMoA, outMoAScaled, Size(outMoA->cols/DEPTH_SCALE, outMoA->rows/DEPTH_SCALE), 0,0, INTER_LINEAR);
			outMoA = &outMoAScaled;
		}
		imshow(windMoA, *outMoA);

		if (debug >= DEBUG_HIGH)
		{
			//imshow("Polar Alt", polar_);
			//imshow("Polar Alt Smooth_", polarAlt_smooth_);
			//imshow("Polar Alt", polarAlt_);
			imshow(nWindows[0], rgbImages[0]);
			imshow(nWindows[1], rgbImages[1]);
			imshow(nWindows[2], rgbImages[2]);
		}

		int c = waitKey(waitTime);
		if (c == 13)
			waitTime = !waitTime;
		else if (c == 27)
			bShouldStop = true;

		frames++;
	}

	delete []pntsMap2;

	for (int i = 0; i < NUM_SENSORS; i++)
	{
		delete []pointsFore2D[i];
	}

	delete activityMap;
	delete activityMap_Back;

	writeTrackingResults(tracks);

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

	outDebugFile << "PARTIAL EXECUTION CREATE PROBABILITY MAP" << endl;
	for (int i = 0; i < TOTAL_SUBINTERVAL_PROB; i++)
	{
		float timpe_p = totalSubIntervalsProb[i]*100/totalIntervals[PROBMAP_ID];
		outDebugFile << titles_subProb[i] << ": " << timpe_p << " %" << endl;
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