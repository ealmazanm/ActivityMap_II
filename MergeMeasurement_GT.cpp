#include "MergeMeasurement_GT.h"


MergeMeasurement_GT::MergeMeasurement_GT(void)
{
}


MergeMeasurement_GT::~MergeMeasurement_GT(void)
{
}

int waitTime = 1;
void detectionSelection_onMouse(int event, int x, int y, int flags, void* param)
{
	if (event == CV_EVENT_FLAG_LBUTTON)
	{
		vector<RPSDetections>* rpsDtcVct = (vector<RPSDetections>*)param;
		if (x != -1 && y != -1)
		{
			waitTime = 0;
			//search the closest detection
			int minPos;
			double minDist = -1;
			int ttlDtc = rpsDtcVct->size();
			for (int i = 0; i < ttlDtc; i++)
			{
				RPSDetections dtc = (*rpsDtcVct)[i];
				double xDiff = x-dtc.prmsRPS.mean.x;
				double yDiff = y-dtc.prmsRPS.mean.y;
				double dist = sqrtf((xDiff*xDiff) + (yDiff*yDiff));
				if (minDist == -1 || dist < minDist)
				{
					minPos = i;
					minDist = dist;
				}
			}
			if (minDist != -1)
			{
				(*rpsDtcVct)[minPos].merge = !(*rpsDtcVct)[minPos].merge ;
			}
			else
				cout << "Error" << endl;

		}
	}
}

void fillDetection(char* lineC, RPSDetections& dtc, int i)
{
		
	switch (i)
	{
	case 0:
		//id Detection
		dtc.measId = atoi(lineC);
		break;
	case 1:
		//id Frame
		dtc.idFrame = atoi(lineC);
		break;
	case 2:
		//Area detection
		dtc.area = atof(lineC);
		break;
	case 3:
		//MeanX
		dtc.prmsRPS.mean.x = atof(lineC);
		break;
	case 4:
		//MeanY
		dtc.prmsRPS.mean.y = atof(lineC);
		break;
	case 5:
		//CovX
		dtc.prmsRPS.covX = atof(lineC);
		break;
	case 6:
		//CovY
		dtc.prmsRPS.covY = atof(lineC);
		break;
	case 7:
		//Merge
		dtc.merge = atoi(lineC);
		break;
	case 8:
		dtc.prmsMoA.mean.x = atoi(lineC);
		break;
	case 9:
		dtc.prmsMoA.mean.y = atoi(lineC);
		break;
	case 10:
		dtc.prmsMoA.covX = atof(lineC);
		break;
	case 11:
		dtc.prmsMoA.covY = atof(lineC);
		break;
	case 12:
		dtc.prmsMoA.covXY = atof(lineC);
		break;
	default://for the rest of the line
		cout << "Error" << endl;
		break;
	}
}


void readSaveDetections(vector<RPSDetections>& rpsDtcns)
{
	ifstream inMerge(rpsDtPath);
		
	string line;
	while (getline(inMerge, line))
	{
		char *lineC = new char[line.length() + 1];
		strcpy(lineC, line.c_str());
		lineC = strtok(lineC, " ");
		int i = 0;
		RPSDetections dtc;
		while (lineC)
		{			
			fillDetection(lineC, dtc, i);
			lineC = strtok(NULL, " ");
			i++;
		}
		delete [] lineC;
		rpsDtcns.push_back(dtc);
	}

	inMerge.close();
}


void writeRPSDetections(vector<RPSDetections>& rpsDtcns, vector<RPSDetections>& oldrpsDtcns)
{
	int ttl = rpsDtcns.size();
	int ttlOld = oldrpsDtcns.size();
	if (ttlOld == 0 || ttlOld <= ttl)
		oldrpsDtcns = rpsDtcns;

	else
	for (int i = 0; i < ttl; i++)
	{
		oldrpsDtcns[i] = rpsDtcns[i];
	}
	

	ofstream outMerge(rpsDtPath);

	ttlOld = oldrpsDtcns.size();
	for (int i = 0; i < ttlOld; i++)
	{
		RPSDetections rpsDtc = oldrpsDtcns[i];

		outMerge << rpsDtc.measId << " " << rpsDtc.idFrame << " " << rpsDtc.area << " " << rpsDtc.prmsRPS.mean.x << " " << rpsDtc.prmsRPS.mean.y << " ";
		outMerge << rpsDtc.prmsRPS.covX << " " << rpsDtc.prmsRPS.covY << " " << (int)rpsDtc.merge << " "
			<< rpsDtc.prmsMoA.mean.x << " " << rpsDtc.prmsMoA.mean.y << " " << rpsDtc.prmsMoA.covX << " "
			<< rpsDtc.prmsMoA.covY << " " << rpsDtc.prmsMoA.covXY << endl;
	}
	outMerge.close();
}

void updateDtcDisplay(vector<RPSDetections>& rpsDtcns, Mat& img, int& waitTime)
{
	int ttl = rpsDtcns.size();
	for (int i = 0; i < ttl; i++)
	{
		RPSDetections dtct = rpsDtcns[i];
		if (dtct.merge)
		{
			float vals[] = {dtct.prmsRPS.covX, 0, 0, dtct.prmsRPS.covY};
			Mat cov = Mat(2,2, CV_32F, vals);
			drawPersonPointsCov_debug(dtct.prmsRPS.mean, cov, &img, Scalar(0, 0, 255), 1, NULL);
		}
	}
}

void updateMergesII(vector<RPSDetections>& oldDtctns, vector<RPSDetections>& rpsDtc_Frame, int frameId)
{
	int ttlOldDt = oldDtctns.size();
	bool end = false;
	int cnt = 0;
	int ttl_dtctPpl = rpsDtc_Frame.size();
	while (!end && cnt < ttlOldDt)
	{
		RPSDetections* oldDtc = &(oldDtctns[cnt]);
		if (oldDtc->idFrame == frameId)
		{
			int i = 0;
			bool found = false;
			while (!found && i < ttl_dtctPpl)
			{
				RPSDetections* d = &(rpsDtc_Frame[i]);
				if (d->measId == oldDtc->measId)
				{
					d->merge = oldDtc->merge;
					found = true;
				}
				i++;
			}
		}
		else if (oldDtc->idFrame > frameId)
			end = true;

		cnt++;
	}

}

void updateMerges(vector<RPSDetections>& oldDtctns, vector<RPSDetections>& dtctPpl, int ttl_dtctPpl, int frameId)
{
	int ttlOldDt = oldDtctns.size();
	bool end = false;
	int cnt = 0;
	while (!end && cnt < ttlOldDt)
	{
		RPSDetections oldDtc = oldDtctns[cnt];
		if (oldDtc.idFrame == frameId  && oldDtc.merge)
		{
			int i = 0;
			bool found = false;
			while (!found && i < ttl_dtctPpl)
			{
				RPSDetections* d = &(dtctPpl[i]);
				if (d->measId == oldDtc.measId)
				{
					d->merge = true;
					found = true;
				}
				i++;
			}
		}
		else if (oldDtc.idFrame > frameId)
			end = true;

		cnt++;
	}

}

void MergeMeasurement_GT::visualizeGT()
{
	vector<RPSDetections> oldDtctns;
	readSaveDetections(oldDtctns);
	int ttl = oldDtctns.size();

	char *pathRPS = "d:\\Emilio\\Tracking\\DataSet\\sb125\\SecondDay\\DSet1\\RemapPolarSpace_Detection.avi";
	VideoCapture cap(pathRPS);
	int frId = 0;
	int wait = 100;
	int cont = 0;
	bool exit = false;
	if (cap.isOpened())
	{
		while(frId < 1000)
		{
			cout << "Frame: " << frId << endl;
			Mat frame;
			cap >> frame;

			while (cont < ttl && !exit)
			{
				RPSDetections dtct = oldDtctns[cont];
				if (dtct.idFrame > frId)
				{
					exit = true;
				}
				else
				{
					cont ++;
					if (dtct.idFrame == frId)
					{
						float vals[] = {dtct.prmsRPS.covX, 0, 0, dtct.prmsRPS.covY};
						Mat cov = Mat(2,2, CV_32F, vals);
						Scalar color = Scalar(255, 0, 0);
						if (dtct.merge)
							color = Scalar(0,0,255);

						drawPersonPointsCov_debug(dtct.prmsRPS.mean, cov, &frame, color, 1, NULL);
					}
				}
			}
			exit = false;
			

			imshow("RPSpace Det Gt", frame);
			int c = waitKey(wait);
			if (c == 13)
				wait = !wait;

			frId++;
		}

	}
	cap.release();
}

void displayRPSDetections (vector<RPSDetections>& dtctns, Mat& img)
{
	int ttl = dtctns.size();
	for (int i = 0; i < ttl; i++)
	{
		RPSDetections dtct = dtctns[i];
		float vals[] = {dtct.prmsRPS.covX, 0, 0, dtct.prmsRPS.covY};
		Mat cov = Mat(2,2, CV_32F, vals);
		Scalar color = Scalar(255, 0, 0);
		if (dtct.merge)
			color = Scalar(0,0,255);
		
		drawPersonPointsCov_debug(dtct.prmsRPS.mean, cov, &img, color, 1, NULL);
	}
}

void MergeMeasurement_GT::generateGT(int fromVideo, int recordOut, int tilt, int debug)
{
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
	

	//flags
	bool bShouldStop = false;
	bool trans = true;
	bool bgComplete = true;

	Mat depthImages[NUM_SENSORS];
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
		depthMat[i] = Mat(XN_VGA_Y_RES, XN_VGA_X_RES, CV_16U);
		pointsFore2D[i] = new XnPoint3D[MAX_FORGROUND_POINTS];
		numberOfForegroundPoints[i] = 0;
	}


	Mat polar = Mat(RANGE_ROWS,181, CV_16UC1);
	Mat polarAlt= Mat(RANGE_ROWS,181, CV_16UC1);
	Mat polarAlt_smooth= Mat(RANGE_ROWS,181, CV_16UC1);
	Mat polarAlt_ = Mat(RANGE_ROWS, 181, CV_8UC1);
	Mat polarAlt_smooth_ = Mat(RANGE_ROWS, 181, CV_8UC1);
	Mat polarAltRGB = Mat(RANGE_ROWS, 181, CV_8UC3);

	VideoWriter w;
	recordOut = 0;
	if (recordOut == 1)
	{
		w.open("d:\\Emilio\\Tracking\\DataSet\\sb125\\SecondDay\\DSet1\\RemapPolarSpace_DetectionNew.avi",CV_FOURCC('P','I','M','1'), 20.0, polarAlt_smooth_.size(), true);
	}

	

	//Size of kernel: smooth rps;
	Mat kernel = Mat::ones(Size(5,27), CV_32F);

	int nPoints = 0;
	int frames = 1;
	int debugFrame = -1;
	bool first = true;
	vector<PointMapping>* pntsMap2 = new vector<PointMapping>[polarAlt.rows*polarAlt.cols];
	vector<RPSDetections> rpsDtc;
	vector<RPSDetections> rpsDtc_Frame;
	vector<RPSDetections> oldDtctns;

	readSaveDetections(oldDtctns);

	char* nameWnd = "RPSpace";
	namedWindow(nameWnd,1);
	cvSetMouseCallback(nameWnd, detectionSelection_onMouse, &rpsDtc_Frame);



	while (!bShouldStop && frames <= 1000)
	{		
		cout << "Frames: " << frames << endl;

		Utils::initMat1s(polarAlt, 0);
		Utils::initMat1s(polar, 0);

		for (int i = 0; i < NUM_SENSORS; i++)
			kinects[i].waitAndUpdate();
		
		for (int i = 0; i < NUM_SENSORS;  i++)
		{

			depthMaps[i] = kinects[i].getDepthMap();
			kinects[i].getDepthImage(depthImages[i]);
			createDepthMatrix(depthMaps[i], depthMat[i]);

			//to create a mask for the noise (depth img is threhold)
			cvtColor(depthImages[i],grey,CV_RGB2GRAY);
			masks[i] = grey > 250; //mask that identifies the noise (1)
		}
		
		nPoints = 0;
		if (bgComplete && trans)// && frames > 20) //Trans must be true
		{
			for (int i = 0; i < NUM_SENSORS; i++)
			{
				numberOfForegroundPoints[i] = subtractors[i].subtraction(pointsFore2D[i], &(depthMat[i]), &(masks[i]));
				nPoints += numberOfForegroundPoints[i];
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
					points3D[i] = new XnPoint3D[numberOfForegroundPoints[i]];
					kinects[i].arrayBackProject(pointsFore2D[i], points3D[i], numberOfForegroundPoints[i]);
			
					kinects[i].transformArray(points3D[i], numberOfForegroundPoints[i]);
					
					//Create alternative representation
					updatePolarAlternateive(&polarAlt, &polar, pntsMap2, ttlPnts, points3D[i], pointsFore2D[i], rgbMaps[i], numberOfForegroundPoints[i], debug, i);	
				}
		
				cv::filter2D(polarAlt, polarAlt_smooth, -1, kernel);
				
				ccDetection(polarAlt_smooth, dtctPpl, ttl_dtctPpl, pntsMap2, ttlPnts, debug, frames, debugFrame); //Connected component detection
				
				//Store the area of the detections in a file
				if (ttl_dtctPpl > 0)
				{
					for (int i = 0; i < ttl_dtctPpl; i++)
					{
						Person prs = dtctPpl[i];
						RPSDetections rpsd;
						rpsd.idFrame = frames;
						rpsd.measId = prs.idDetection;
						rpsd.area = (2*prs.sigmaX_RPS) * (2*prs.sigmaY_RPS);
						rpsd.merge = false;
						EllipseParam ep;
						ep.mean = prs.mean_RPS;
						ep.covX = prs.sigmaX_RPS*prs.sigmaX_RPS;
						ep.covY = prs.sigmaY_RPS*prs.sigmaY_RPS;
						ep.covXY = 0;
						rpsd.prmsRPS = ep;
						EllipseParam epMoA;
						epMoA.mean = prs.meanDtction;
						epMoA.covX = prs.covDtction.at<float>(0,0);
						epMoA.covY = prs.covDtction.at<float>(1,0);
						epMoA.covXY = prs.covDtction.at<float>(1,1);
						rpsd.prmsMoA = epMoA;
						rpsDtc_Frame.push_back(rpsd);
					}
				}
			
				delete [] dtctPpl;
				ttl_dtctPpl = 0;
				for (int i = 0; i < NUM_SENSORS; i++)
				{
					delete []points3D[i];
				}
			}
		}
		
		//w << polarAltRGB;
		Utils::convert16to8(&polarAlt_smooth, polarAlt_smooth_);
		cvtColor(polarAlt_smooth_, polarAltRGB, CV_GRAY2RGB);
		updateMergesII(oldDtctns, rpsDtc_Frame, frames); //updates rpsDtc_frame with the saved information
		
		displayRPSDetections(rpsDtc_Frame, polarAltRGB);
		imshow(nameWnd, polarAltRGB);
		int c = waitKey(waitTime);

		int wtBef = 1;
		if (c == 13)
			waitTime = !waitTime;
		else if (c == 27)
			bShouldStop = true;
		
		if (waitTime == 0)
		{
			wtBef = 0;
			waitTime = 1;
		}
		updateDtcDisplay(rpsDtc_Frame, polarAltRGB, waitTime);
		//updateMergesII(oldDtctns, rpsDtc_Frame, frames);
		imshow(nameWnd, polarAltRGB);
		c = waitKey(waitTime);
		if (wtBef == 0)
			waitTime = 0;

		if (!rpsDtc_Frame.empty())
		{
			rpsDtc.insert( rpsDtc.end(), rpsDtc_Frame.begin(), rpsDtc_Frame.end() );
			rpsDtc_Frame.clear();
		}

		frames++;
	}

	delete []pntsMap2;

	for (int i = 0; i < NUM_SENSORS; i++)
	{
		delete []pointsFore2D[i];
	}

	writeRPSDetections(rpsDtc, oldDtctns);


	
	for (int i = 0; i < NUM_SENSORS; i++)
	{
		kinects[i].stopDevice();
  		kinects[i].shutDown();
	}
}