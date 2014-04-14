#include "visualizeGT.h"


visualizeGT::visualizeGT(void)
{
}


visualizeGT::~visualizeGT(void)
{
}



Mat aggregateAll(const Mat* im1, const Mat* im2, const Mat* im3, const Mat* moa)
{
	int width = moa->cols/3;
	int height = im1->rows*width/im1->cols;
	Size szIm = Size(width, height);

	Mat im1Rsz, im2Rsz, im3Rsz;
	resize((*im1), im1Rsz, szIm);
	resize((*im2), im2Rsz, szIm);
	resize((*im3), im3Rsz, szIm);

	Mat imgs[] = {im1Rsz,im2Rsz, im3Rsz};

	Size fnalSz = Size(moa->cols, moa->rows+height);
	Mat out = Mat(fnalSz, CV_8UC3);

	for (int i = 0; i < 3; i++)
	{
		Rect r = Rect(szIm.width*i,0, szIm.width, szIm.height);
		Mat outRoi = out(r);
		imgs[i].copyTo(outRoi);
	}

	Rect r = Rect(0,szIm.height, moa->cols, moa->rows);
	Mat outRoi = out(r);
	moa->copyTo(outRoi);

	line(out, Point(0, height), Point(moa->cols,height), Scalar::all(0), 2);

	for (int i = 0; i < 3; i++)
	{
		line(out, Point(i*width,0), Point(i*width, height), Scalar::all(0), 2);
	}

	return out;

}

Mat aggregateMoaPolars(const Mat* moa, const Mat* polar, const Mat* rps)
{
	//Size of the moa
	int width = ceilf(rps->rows*moa->cols/moa->rows);
	int height = rps->rows;
	Size szMoa(width, height);
	Mat moaResize;
	resize((*moa), moaResize, szMoa);

	Mat polarColor, rpsColor;
	cvtColor((*polar), polarColor, CV_GRAY2RGB);
	cvtColor((*rps), rpsColor, CV_GRAY2RGB);

	Size detSize = Size(rps->cols*2+width, rps->rows);
	Mat out = Mat(detSize, CV_8UC3);
	
	Rect r = Rect(0, 0, width, height);
	Mat moaRoi = out(r);
	moaResize.copyTo(moaRoi);

	r = Rect(width,0, rps->size().width, rps->size().height);
	Mat rpsRoi = out(r);
	polarColor.copyTo(rpsRoi);

	r = Rect(width+rps->cols,0, rps->size().width, rps->size().height);
	rpsRoi = out(r);
	rpsColor.copyTo(rpsRoi);

	line(out, Point(width, 0), Point(width, height), Scalar::all(0), 2);
	line(out, Point(width+rps->cols, 0), Point(width+rps->cols, height), Scalar::all(0), 2);
	return out;

}

Mat aggregateDetectionImages(const Mat* rps, const Mat* moa, bool convert)
{
	int width = ceilf(rps->rows*moa->cols/moa->rows);
	int height = rps->rows;
	Size szMoa(width, height);

	Size detSize = Size(rps->cols+width, rps->rows);

	Mat out = Mat(detSize, CV_8UC3);

	//convert to 3 channels
	Mat rpsColor;
	cvtColor((*rps), rpsColor, CV_GRAY2RGB);

	//Size of the moa
	Mat moaResize;
	resize((*moa), moaResize, szMoa);
	if (convert)
	{
		Mat moaRszColor;
		cvtColor((*moa), moaRszColor, CV_GRAY2RGB);
		moaRszColor.copyTo(moaResize);
	}
	Rect r = Rect(0,0, rps->size().width, rps->size().height);
	Mat rpsRoi = out(r);
	rpsColor.copyTo(rpsRoi);
	
	r = Rect(rps->size().width, 0, szMoa.width, szMoa.height);
	Mat moaRoi = out(r);
	moaResize.copyTo(moaRoi);

//	imshow("rps color", rpsColor);
//	imshow("moa resize", moaResize);
//	imshow("final aggregation", out);
//	waitKey(0);

	return out;

}


void visualizeGT::visualize(int tilt, int debug)
{
	char* paths[3];
	paths[0] = "d:/Emilio/Tracking/DataSet/kinect0_calib.oni";
	paths[1] = "d:/Emilio/Tracking/DataSet/kinect1_calib.oni";
	paths[2] = "d:/Emilio/Tracking/DataSet/kinect2_calib.oni";

	//Initialize resolutions MoA and Remap Polar space
	ActivityMap_Utils actMapCreator(DEPTH_SCALE, NUM_SENSORS);


	KinectSensor kinects[NUM_SENSORS];
	const XnDepthPixel* depthMaps[NUM_SENSORS];
	const XnRGB24Pixel* rgbMaps[NUM_SENSORS];

	for (int i = 0; i < NUM_SENSORS; i++)
	{
		kinects[i].initDevice(i, REF_CAM, true, paths[i]);
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
	bool deleteBG = false;

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

	//Resize images from kinect
	int xSize = actMapCreator.getResolution().width/3;
	int ySize = xSize*XN_VGA_Y_RES/XN_VGA_X_RES;
	Size resizeImg (xSize, ySize);
	Size aggrSize = Size(actMapCreator.getResolution().width, actMapCreator.getResolution().height + ySize);

	//resize the moa to adjust to the height of the polar representation
	int width = ceilf(polarAlt_smooth_.rows*actMapCreator.getResolution().width/actMapCreator.getResolution().height);
	int height = polarAlt_smooth_.rows;
	Size moaSize(width, height);

	//Size of the polar+moa image
	Size detSize = Size(polarAlt_smooth_.cols+width, polarAlt_smooth_.rows);

	//Size of moa+polar+rps image
	Size MoaPRPSSize = Size(polarAlt_smooth_.cols*2+width, polarAlt_smooth_.rows);
	//Size of polars
	Size polarSize = Size(polarAlt_smooth_.cols*2, polarAlt_smooth_.rows);

	VideoWriter wMoA("d:\\Emilio\\Tracking\\DataSet\\sb125\\SecondDay\\DSet1\\MoA_tracking_V1.avi",CV_FOURCC('M','J','P','G'), 10.0, aggrSize, true);
	VideoWriter wDet("d:\\Emilio\\Tracking\\DataSet\\sb125\\SecondDay\\DSet1\\Detection_V1.avi",CV_FOURCC('M','J','P','G'), 10.0, detSize, true);
	VideoWriter wPolar("d:\\Emilio\\Tracking\\DataSet\\sb125\\SecondDay\\DSet1\\Polars_V1.avi",CV_FOURCC('M','J','P','G'), 10.0, polarSize, true);
	VideoWriter wMoaPRPS("d:\\Emilio\\Tracking\\DataSet\\sb125\\SecondDay\\DSet1\\MoAPolars_V1.avi",CV_FOURCC('M','J','P','G'), 10.0, MoaPRPSSize, true);
	//VideoWriter wDet("d:\\Emilio\\Tracking\\DataSet\\sb125\\SecondDay\\DSet1\\Detection_V1.mpg",CV_FOURCC('P','I','M','1'), 10.0, detSize, true);
	int recordOut = 0;


	//Size of kernel: smooth rps;
	Mat kernel = Mat::ones(Size(5,27), CV_32F);



	char* windMoA = "Activity Map";
	int nPoints = 0;
	int frames = 0;
	int debugFrame = -1;

	vector<PointMapping>* pntsMap2 = new vector<PointMapping>[polarAlt.rows*polarAlt.cols];

	while (!bShouldStop && frames < 1000)
	{		
		cout << "Frames: " << frames++ << endl;

		if (frames == 5) 
			bgComplete = true;

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

					updateActivityMap(*activityMap, *activityMap_Back, &actMapCreator, points3D[i], numberOfForegroundPoints[i], pointsFore2D[i]);
					
				}
		
				//Todo: Create a method detection(polarAlt, moAPeople)
				cv::filter2D(polarAlt, polarAlt_smooth, -1, kernel);

				ccDetection(polarAlt_smooth, dtctPpl, ttl_dtctPpl, pntsMap2, ttlPnts, debug, frames, debugFrame); //Connected component detection
				Utils::convert16to8(&polarAlt_smooth, polarAlt_smooth_);

				Utils::convert16to8(&polarAlt, polarAlt_);
				Utils::convert16to8(&polar, polar_);

				//displayDetections(dtctPpl, ttl_dtctPpl, polarAlt_smooth_, *activityMap, debug);

				threshold(polar_, polar_, 200, 255, THRESH_BINARY);
				threshold(polarAlt_, polarAlt_, 200, 255, THRESH_BINARY);


				Mat aggrImages = aggregateAll(&rgbImages[0], &rgbImages[1], &rgbImages[2], activityMap);

				Mat aggrDetection = aggregateDetectionImages(&polarAlt_smooth_, activityMap, false);
				Mat aggrPolar = aggregateDetectionImages(&polar_, &polarAlt_, true);
				Mat aggrMoaPolars = aggregateMoaPolars(activityMap, &polar_, &polarAlt_);

				line(aggrPolar, Point(polar_.cols, 0), Point(polar_.cols, aggrPolar.rows), Scalar::all(0), 2);
				line(aggrDetection, Point(polarAlt_smooth.cols,0), Point(polarAlt_smooth.cols, aggrDetection.rows), Scalar::all(0), 2); 

				imshow("Aggreage Detection", aggrDetection);
				imshow("Aggreage Polars", aggrPolar);
				imshow("Aggregate All", aggrImages);
				imshow("Aggregate all all", aggrMoaPolars);
				
				bShouldStop =  waitKey(1) == 27;

				wMoA << aggrImages;
				wDet << aggrDetection;
				wPolar << aggrPolar;
				wMoaPRPS << aggrMoaPolars;

				delete [] dtctPpl;
				ttl_dtctPpl = 0;
				for (int i = 0; i < NUM_SENSORS; i++)
				{
					delete []points3D[i];
				}
			}
		}
	}
	wMoA.release();
	wDet.release();
	wPolar.release();
	for (int i = 0; i < NUM_SENSORS; i++)
	{
		kinects[i].stopDevice();
  		kinects[i].shutDown();
	}
}