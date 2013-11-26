#pragma once
#include <Utils.h>

namespace AM
{
	//INTERVALS TIME EXECUTION
		static const int TOTAL_INTERVALS = 11;
		static const int BSUB_ID = 0;
		static const int RPSPACE_ID = 1;
		static const int MOA_ID = 2;
		static const int DET_ID = 3;
		static const int BACKPR_ID = 4;
		static const int PTRANS_ID = 5;
		static const int SMOOTH_ID = 6;
		static const int DISPLAY_ID = 7;
		static const int TRACK_ID = 8;
		static const int MASKOVERLAPPING_ID = 9;
		static const int TOT_ID = 10;
		static char* titles[TOTAL_INTERVALS] = {"BACKGROUND SUBTRACTION", "REMAP POLAR SPACE", "MAP OF ACTIVITY", "DETECTION", "POINTS BACKPROJECTION", "POINTS TRANSFORMATION", "SMOOTH", "DISPLAY", "TRACKING", "MASK OVERLAPPING" " TOTAL"};
		static float totalIntervals[TOTAL_INTERVALS] = {0,0,0,0,0,0,0,0,0,0,0};

		//Time interval sub-function RPS
		static const int TOTAL_SUBINTERVAL_RPS = 4;
		static const int ANGLE_ID = 0;
		static const int RANGE_ID = 1;
		static const int UPDATE_ID = 2;
		static const int FEATURE_ID = 3;
		static char* titles_subRPS[TOTAL_SUBINTERVAL_RPS] = {"ANGLE", "RANGE", "UPDATE", "FEATURE"};
		static float totalSubIntervalsRPS[TOTAL_SUBINTERVAL_RPS] = {0,0,0,0};

		//Time interval sub-function Create Activity Map
		static const int TOTAL_SUBINTERVAL_MOA = 4;
		static const int CONVERT2PONITS_ID = 0;
		static const int ARRAYBACKPROJECT_ID = 1;
		static const int TRANSFORM_ID = 2;
		static const int FINDMOA_ID = 3;
		static char* titles_subMOA[TOTAL_SUBINTERVAL_MOA] = {"CONVERT 2 POINTS", "ARRAY BACKPROJECT", "TRANSFORM", "FINDMOA_ID"};
		static float totalSubIntervalsMOA[TOTAL_SUBINTERVAL_MOA] = {0,0,0,0};

	//DEBUG_CONSTANTS
		static const int DEBUG_NONE = -1;
		static const int DEBUG_LOW = 0;
		static const int DEBUG_MED = 1;
		static const int DEBUG_HIGH = 2;
		static const int DEBUG_OUT = -2;
		static const int NUM_SENSORS = 3;
		static const float DEPTH_SCALE = 1; // decide the scaling factor with respect to MAX_RANGE
		static int KINECTS_DISPLACEMENT = 0; 
		static float MAX_RANGE;
		static const int RANGE_ROWS = 500;
		static const int RANGE_COLS = 181;
		//*IMPORTANT*//
		//THIS TWO VALUES SHOULD NOT BE CHANGED.
		//It will affect the amount of points projected in each pixel of the
		//remap polar space. The direct consecuence is that the thresholds used for
		//the detection will not be suitable.
		static const int RANGE_STEP = 24; //ceil(11650.0/RANGE_ROWS); //THIS VALUES MUST BE FIXED
		static const int SCALE_RANGE_ALT = 25896; //scale the alternative espace to match MAX_RANGE

	//PERSON MODEL SETTINGS
		static const int MODEL_MAX_HEIGHT = 300;
		static const int MODEL_MIN_HEIGHT = -2000;
		static const int MODEL_BINRANGE = 230;
		static const int MODEL_NBINS = (MODEL_MAX_HEIGHT-MODEL_MIN_HEIGHT)/MODEL_BINRANGE;
		static const int MAX_PEOPLE = 50;

		struct gaussianParam
		{
			Scalar mean;
			Mat cov;

			//Debug
			const XnRGB24Pixel** colours;
			int totalColours;
		};

		struct Person
		{
			int id;
			Mat stateMoA;
			Mat covMoA;
			Point  mean_RPS;
			int sigmaY_RPS;
			int sigmaX_RPS;
			Mat R;
			Mat Q;
			Mat A; 
	
			float heightModel[MODEL_NBINS];
			gaussianParam colourModel[MODEL_NBINS];
			int control;
			int lost;
			bool associated;
		};

		struct PointMapping
		{
			const XnPoint3D* p3D;
			const XnRGB24Pixel* colour;
			int rpX;
			int rpY;
		};

	//Tracking variables
		static const int DTC_FULL = 0;
		static const int DTC_MERGE = 1;
		static const int DTC_SPLIT = 2;
		static const int TRACKLOST_THRESHOLD = 10;
		//Update linear model
		static float valsH[] = {1, 0, 0, 0, 0, 1, 0, 0};
		static Mat H = Mat(2,4, CV_32F, valsH);
		static float const COMP_THRESHOLD = 0.5;
		//update coefficients
		static float ALPHA = 0.05;
		static float BETA = 0.01;
		static int pplId_cont = 0;

		//Data types for file storage of trackings
		struct Position
		{
			int frameId;
			Rect bbox;
		};
		struct Track
		{
			int id;
			vector<Position> trajectory;
		};
		static vector<Track> tracks;

	//Output files
		static ofstream outDebugFile("d:/Debug.txt");
		static ofstream fpsOut("c:\\Dropbox\\PhD\\Matlab\\FrameRate\\fps_BGUpdt.txt");
		static ofstream outPerson("c:\\Dropbox\\PhD\\Matlab\\PdF_person\\pdf_points.txt");
		static ofstream outHeights("c:\\Dropbox\\PhD\\Matlab\\Height_People\\heights.txt");
		static ofstream outPersModel ("c:\\Dropbox\\PhD\\Matlab\\Model\\modelInfo.txt");
		static ofstream outBboxModel ("c:\\Dropbox\\PhD\\Matlab\\BBoxes\\bboxInfo.txt");
		static ofstream outFPK0 ("C:\\Dropbox\\PhD\\Matlab\\Calibration_wks\\kinect0_full.txt");
		static ofstream outFPK1 ("C:\\Dropbox\\PhD\\Matlab\\Calibration_wks\\kinect1_full.txt");
		static ofstream outFPK2 ("C:\\Dropbox\\PhD\\Matlab\\Calibration_wks\\kinect2_full.txt");
		static ifstream tiltTXT("D:\\CameraCalibrations\\extrinsics\\tilt.txt");

	//FUNCTIONS
		//Transform the array of depths into a matrix (USHORT/CV_16U) of depths

		static int fillArrayPointSel(const Mat& roi, XnPoint3D* points, int origX)
		{
			int ttlRows = roi.rows;
			int ttlCols = roi.cols;

			ushort* d_data = (ushort*)roi.data;
			int d_step = roi.step/sizeof(ushort);

			int id = 0;
			for (int i = 0; i < ttlRows; i++)
			{
				ushort* ptr = d_data + i*d_step;
				for (int j = 0; j < ttlCols; j++)
				{
					int depth = ptr[j];
					if (depth != 0)
					{
						//int id = i*ttlCols + j;
						points[id].X = j+origX;
						points[id].Y = i;
						points[id++].Z = depth;
					}
				}
			}
			return id;
		}

		static void maskDepthImagePoint(Mat& dMat, const XnPoint3D* points3D, const XnPoint3D* points2D, int ttlP)
		{
			int ttlRows = dMat.rows;
			int ttlCols = dMat.cols;
			ushort* d_data = (ushort*)dMat.data;
			int d_step = dMat.step/sizeof(ushort);

			for (int id = 0; id < ttlP; id++)
			{
				XnPoint3D p = points3D[id];
				if (p.X < XN_VGA_X_RES && p.X >= 0 && p.Y < XN_VGA_Y_RES && p.Y >= 0)
				{
					XnPoint3D p2d = points2D[id];
					ushort* ptr = d_data + (int)p2d.Y*d_step;
					ptr[(int)p2d.X] = 0;
				}
			}
		}

		static void maskOutOverlappingPointSel(Mat& depthMat, Rect rLeft, Rect rRight, KinectSensor* kinects)
		{
			int tpLeft = rLeft.width*rLeft.height;
			int tpRight = rRight.width*rRight.height;
			//Go through al the points in the roi and create an array of XnDepth3D
			Mat leftRoi = depthMat(rLeft);
			Mat rightRoi = depthMat(rRight);
			//TODO: may be parameters of the function
			XnPoint3D* leftSide2D = new XnPoint3D[tpLeft];
			XnPoint3D* rightSide2D = new XnPoint3D[tpRight];

			int ttlLeft = fillArrayPointSel(leftRoi, leftSide2D, rLeft.x);
			int ttlRight = fillArrayPointSel(rightRoi, rightSide2D, rRight.x);

			//Back project to the 3D space
			XnPoint3D* leftSide3D = new XnPoint3D[ttlLeft];
			XnPoint3D* rightSide3D = new XnPoint3D[ttlRight];
			kinects[REF_CAM].arrayBackProject(leftSide2D, leftSide3D, ttlLeft);
			kinects[REF_CAM].arrayBackProject(rightSide2D, rightSide3D, ttlRight);

			// transform into the second CS (make sure it is calibrated the other way around)
			kinects[REF_CAM].transformArrayNoTilt_rev(leftSide3D, ttlLeft, 0);
			kinects[REF_CAM].transformArrayNoTilt_rev(rightSide3D, ttlRight, 2);

			//Project into the image plane
			XnPoint3D* leftCamP = kinects[0].arrayProject(leftSide3D, ttlLeft);
			XnPoint3D* rightCamP = kinects[2].arrayProject(rightSide3D, ttlRight);

			maskDepthImagePoint(depthMat, leftCamP, leftSide2D, ttlLeft);
			maskDepthImagePoint(depthMat, rightCamP, rightSide2D, ttlRight);

			//Free memory
			delete []leftSide2D;
			delete []rightSide2D;
			delete []leftSide3D;
			delete [] rightSide3D;
			delete [] leftCamP;
			delete [] rightCamP;
		}

		static void createDepthMatrix(const XnDepthPixel* dMap, Mat& depthMat)
		{
			ushort* d_data = (ushort*)depthMat.data;
			int d_step = depthMat.step/sizeof(ushort);

			for (int i = 0; i < XN_VGA_Y_RES; i++)
			{
				ushort* ptr = d_data + i*d_step;
				for (int j = 0; j < XN_VGA_X_RES; j++)
				{
					ptr[j] = dMap[i*XN_VGA_X_RES+j];
				}
			}
		};

		static void printPerson(const Person* p)
		{
			outDebugFile << "********** Track id: " << p->id;
			Utils::printValuesF(&p->stateMoA, "State MoA", outDebugFile);
			Utils::printValuesF(&p->covMoA, "Cov MoA", outDebugFile);
			//Utils::printValuesF(&p->gtArea, "Gating area" , outDebugFile);
			Utils::printValuesF(&p->A , "Prediction model (A)", outDebugFile);
			Utils::printValuesF(&p->Q, "Prediction error covariance (Q)", outDebugFile);
			Utils::printValuesF(&p->R, "Measurement error covariance (R)", outDebugFile);
			outDebugFile << "Lost: " << p->lost << ". Lost threshold: " << TRACKLOST_THRESHOLD << endl;
			outDebugFile << "*****************************************" << endl;
		}

		/*
		polarAlt(out): Updates the positions where the 3D points project after the remapping.
		polar(out): Points projected from the transform points into thep polar coordinate system
		pntsMap(out): List of cooridnate mappings (3D coordinate + colour + 2d rmpspace coordinates
		ttlPnts(out): Total num points that project on the rmpspace

		p3D(in): List of foreground points for a particular kinect
		points2D(in): List of 2D points of the image plane for a particular kinect
		rgbMap(in): Map of rgb colours for a particular kinect
		nP(in): Number of foreground points.
		*/
		static void updatePolarAlternateive(Mat* polarAlt, Mat* polar, PointMapping* pntsMap, int& ttlPnts, const XnPoint3D* p3D, const XnPoint3D* points2D, const XnRGB24Pixel* rgbMap, const int nP, int debug)
		{
			float toRad = 180/CV_PI;
			int pAltStep = polarAlt->step/sizeof(ushort);
			int pStep = polar->step/sizeof(ushort);

			ushort* ptrPAlt = (ushort*)polarAlt->data;
			ushort* ptrP = (ushort*)polar->data;


			clock_t startTime;
			for (int i = 0; i < nP; i++)
			{
				//XnPoint3D p = p3D[i];
				float X = p3D[i].X;
				float Y = p3D[i].Y;
				float Z = p3D[i].Z;
			
				float range = sqrtf(pow(X,2) + pow(Z,2));
				if (range > MAX_RANGE || Z > ActivityMap_Utils::MAX_Z_TRANS  || Y > ActivityMap_Utils::CEILING_THRESHOLD || Y < ActivityMap_Utils::FLOOR_THRESHOLD)
				{
					//if (debug > DEBUG_NONE)
					//	outDebugFile << "UpdatePolarAlternative: 3D Point Rejectected. Info: 3DPoint: " << p.X <<", "<< p.Y << ", " << p.Z << 
					//	". Range: " << range << ". MAX_RANGE: " << MAX_RANGE << ". MAX_Z_TRANS: " << ActivityMap_Utils::MAX_Z_TRANS << ". CEILING_THRESHOLD: " <<
					//	ActivityMap_Utils::CEILING_THRESHOLD << ". FLOOR_THRESHOLD: " << ActivityMap_Utils::FLOOR_THRESHOLD << endl;
					continue; //TODO: ADD A LOG OUTPUT
				}
		
				startTime = clock(); //time debuggin
				int angle = 90;
				if (X != 0 && Z != 0)
					if (X > 0)
						angle = (atanf(Z/abs(X)))*toRad;
					else
						angle = 180-((atanf(Z/abs(X)))*toRad);
				totalSubIntervalsRPS[ANGLE_ID] += clock() - startTime; //time debugging
	
				startTime = clock(); //time debuggin
				float range_m = range/1000;
				float range_Alt = SCALE_RANGE_ALT * ((0.33*log(10+3*range_m))-0.80597); // three distances
				totalSubIntervalsRPS[RANGE_ID] += clock() - startTime; //time debugging

				startTime = clock(); //time debuggin
				int yPos = RANGE_ROWS - ((int)range_Alt/RANGE_STEP);	
				if (yPos < 0 || yPos >= RANGE_ROWS)
				{
					if (debug >= DEBUG_NONE)
						outDebugFile << "UpdatePolarAlternative: BAD REMAPING (Remap Polar Space). Info: 3DPoint: " << X <<", "<< Y << ", " << Z << ". PolarPoint: " <<
							 range <<", "<< angle << ". RemapPolarPoint: " << range_Alt << ". RmapPolarPlanview: " << angle << ", " << yPos <<
							 ". SCALE_RANGE_ALT: " << SCALE_RANGE_ALT << ". RANGE_ROWS: " << RANGE_ROWS << ". RANGE_STEP: " << RANGE_STEP << endl;
					continue;
				}

				//int val = polarAlt->ptr<ushort>(yPos)[angle] - 1;
				//polarAlt->data * (yPos*polarAlt->step)[angle]++;
				//polarAlt->ptr<ushort>(yPos)[angle]++;
				(ptrPAlt + (yPos*pAltStep))[angle]++;
				totalSubIntervalsRPS[UPDATE_ID] += clock() - startTime; //time debugging


				startTime = clock(); //time debuggin
				//add a new mapping
				PointMapping pMap;
				pMap.p3D = p3D+i;
				pMap.rpX = angle; pMap.rpY = yPos;
				int rgbMapPos = (int)(*(points2D+i)).Y*XN_VGA_X_RES+(int)(*(points2D+i)).X;
				pMap.colour = rgbMap + rgbMapPos;
				pntsMap[ttlPnts++] = pMap;
				totalSubIntervalsRPS[FEATURE_ID] += clock() - startTime; //time debugging


				if (debug >= DEBUG_MED)
				{
					yPos = RANGE_ROWS - ((int)range/RANGE_STEP);
					if (yPos < 0 || yPos >= RANGE_ROWS)
					{
						if (debug >= DEBUG_NONE)
							outDebugFile << "UpdatePolarAlternative: BAD REMAPING (Polar Space). Info: 3DPoint: " << X <<", "<< Y << ", " << Z << ". PolarPoint: " <<
								 range <<", "<< angle << ". RANGE_ROWS: " << RANGE_ROWS << ". RANGE_STEP: " << RANGE_STEP << endl;
						continue;
					}
					//polar->ptr<ushort>(yPos)[angle]++;
					(ptrP + (yPos*pStep))[angle]++;
				}		
			}
		}


		static void findBlobs(const cv::Mat &binary, std::vector < std::vector<cv::Point2i> > &blobs)
		{
			blobs.clear();

			// Fill the label_image with the blobs
			// 0  - background
			// 1  - unlabelled foreground
			// 2+ - labelled foreground

			cv::Mat label_image;
			binary.convertTo(label_image, CV_32SC1);

			int label_count = 2; // starts at 2 because 0,1 are used already

			int lbl_Rows = label_image.rows;
			int lbl_Cols = label_image.cols;
			int* lbl_data = (int*)label_image.data;
			int lbl_step = label_image.step/sizeof(int);

			for(int y=0; y < lbl_Rows; y++) 
			{
				int *row = lbl_data + y*lbl_step;
				for(int x=0; x < lbl_Cols; x++) 
				{
					if(row[x] > 0) 
					{
						continue;
					}

					cv::Rect rect;
					cv::floodFill(label_image, cv::Point(x,y), label_count, &rect, 0, 0, 4);

					std::vector <cv::Point2i> blob;

					int maxI = rect.y + rect.height;
					int maxJ = rect.x + rect.width;
					for(int i=rect.y; i < maxI; i++) 
					{
						int *row2 = lbl_data + i*lbl_step;
						for(int j=rect.x; j < maxJ; j++) 
						{
							if(row2[j] != label_count) 
							{
								continue;
							}

							blob.push_back(cv::Point2i(j,i));
						}
					}

					blobs.push_back(blob);

					label_count++;
				}
			}
		}

		/*
		Convert a point from the remap polar space rep. to the MoA (3D)
		*/
		static Point convertBack(const Point* p)
		{
			Point out;
			//Calculate range
			float rangeAlt = (RANGE_ROWS - p->y)*RANGE_STEP + RANGE_STEP/2;
			float range = (-3.83328*(0.869577-exp( (3.0303*rangeAlt)/SCALE_RANGE_ALT) ) )*1000;
			out.x = range*cos((p->x)*CV_PI/180); //+4 there is a misplacement of around 4 degrees in the estimation (ToDo: find out why)
			out.y = range*sin((p->x)*CV_PI/180);
			return out;
		}

		/*
		Initialize the values of the bins in the height and colour model
		*/
		static void initPerson(Person* p)
		{
			//first iteration the measruemente is trusted 100%
			float valsR[] = {0,0,0,0};
			Mat m = Mat(2,2, CV_32F, valsR);
			m.copyTo(p->R);
			//Covariance of the prediction error
			//first iteration the prediction model is trusted 0%
			float valsQ[] = {1,0,0,0,   0,1,0,0,  0,0,1,0,  0,0,0,1};
			m = Mat(4,4, CV_32F, valsQ);
			m.copyTo(p->Q);
			//Prediction linear model
			float valsA[] = {1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1};
			m = Mat(4,4, CV_32F, valsA);
			m.copyTo(p->A);
		
			p->stateMoA = Mat::zeros(4,1,CV_32F);
			p->covMoA = Mat::zeros(4,4, CV_32F);
			//p->gtArea = Mat::zeros(2,2, CV_32F);
			p->control = DTC_FULL;
			p->lost = 0;
			p->associated = false;

			for (int i = 0; i < MODEL_NBINS; i++)
				p->heightModel[i] = 0;

		}

		//Projects the detected gaussian distribution (mean, variance) to the MoA (non linear)
		static void projectLocation2MoA(Person* prs, int debug, int frames, int debugFrame)
		{

			//Mean projection to MoA
			Point cMoA;
			cMoA = convertBack(&prs->mean_RPS);
			XnPoint3D p3D;
			p3D.X = cMoA.x; p3D.Y = ActivityMap_Utils::CEILING_THRESHOLD - 1; p3D.Z = cMoA.y;
			Point meanMoA = ActivityMap_Utils::findMoACoordinate(&p3D, MAX_RANGE);
			if (meanMoA.x == -1)
			{
				if (debug > DEBUG_NONE)
				{
					outDebugFile << "ProjectLocation: person located out of MoA range. INFO: RPS point: " << meanMoA.x << ", " << meanMoA.y <<
						". 3D MoA point: " << cMoA.x << ", " << cMoA.y << ". Range: " << sqrtf(pow(p3D.X,2) + pow(p3D.Z,2)) <<". MAX_RANGE: " << 
						MAX_RANGE << ". MAX_Z_TRANS: " << ActivityMap_Utils::MAX_Z_TRANS << ". CEILING_THRESH: " << ActivityMap_Utils::CEILING_THRESHOLD << 
						". FLOOR_THRESH: " << ActivityMap_Utils::FLOOR_THRESHOLD << endl;
				}
				return; //TODO: ERROR LOG
			}
			prs->stateMoA.at<float>(0, 0) = meanMoA.x;
			prs->stateMoA.at<float>(1, 0) = meanMoA.y;


			//Covariance projection to MoA
			float sigmaAlphaRad = prs->sigmaX_RPS*2*CV_PI/180;
			float sigmaRange = prs->sigmaY_RPS*2;
			float meanAlpha = prs->mean_RPS.x*CV_PI/180; //rad
			float meanRange = prs->mean_RPS.y; //mm
						
			//Jacobian matrix of partial derivatives
			float jacob_11 = (-40.8475*exp(-0.00272633*meanRange)*cosf(meanAlpha))/ActivityMap_Utils::X_STEP;
			float jacob_12 = (3333.33-14982.6*exp(-0.00272633*meanRange)*sinf(meanAlpha))/ActivityMap_Utils::X_STEP;						
			float jacob_21 = (-40.8541*exp(-0.00272652*meanRange)*sinf(meanAlpha))/ActivityMap_Utils::Z_STEP;
			float jacob_22 = (14984*exp(-0.00272652*meanRange)-3333.33*cosf(meanAlpha))/ActivityMap_Utils::Z_STEP;
			float jacobValues[4] = {jacob_11, jacob_12, jacob_21, jacob_22}; 
			Mat jacobMat = Mat(2,2, CV_32FC1, jacobValues);
			//covariance matrix in the plan view remap polar space(RPS)
			float varValues[4] = {powf(sigmaRange,2), 0,0, powf(sigmaAlphaRad,2)}; 
			Mat covPolar = Mat(2,2, CV_32FC1, varValues);
			//Covariance approximation in the plan view MoA
			Mat covCartessian = jacobMat * covPolar * jacobMat.t();

			covCartessian.copyTo(prs->covMoA(Rect(0,0,2,2)));


			//define the gate region 2x2
			Mat spaceCov = prs->covMoA(Rect(0,0, 2,2))*4;
			if (debug > DEBUG_MED)
			{
				if (frames == debugFrame)
				{
					Utils::printValuesF(&prs->covMoA, "CovMoa(I)", outDebugFile);
					Utils::printValuesF(&spaceCov, "Gate area", outDebugFile);
					Utils::printValuesF(&covCartessian, "Cov real", outDebugFile);
				}
				assert(spaceCov.rows, 2);
				assert(spaceCov.cols, 2);
				for (int i = 0; i < spaceCov.rows; i++)
				{
					for (int j = 0; j < spaceCov.cols; j++)
					{
						assert(spaceCov.at<float>(i,j), 4*prs->covMoA.at<float>(i,j));	
						assert(covCartessian.at<float>(i,j) , prs->covMoA.at<float>(i,j));
					}
				}
			}
			//usign 2 standard deviations. Probability of 95%
			//spaceCov.copyTo(prs->gtArea);

		}


		//Detect people blobs using an hysteresis threshold and a component labelling (on the RMPSpace)
		static void detectCC(Mat& bw, Person* dtctPpl, int& ttl_dtctPpl, const PointMapping* pntsMap, int ttlPnts, Mat& debugImg, int debug, int frames, int debugFrame)
		{
			Mat imgCpy = Mat(bw.size(), CV_8UC1);
			Mat cpy = Mat(bw.size(), CV_8UC1);
			Utils::convert16to8(&bw, imgCpy); //convert the image to a range between 0-255. And also inverts the values, so 0 belongs to high values in viceversa

			//Create a binary image
			threshold(imgCpy, cpy, 254, 1, THRESH_BINARY); //backround = 1, foreground = 0;

			//find connected components
			std::vector < std::vector<cv::Point2i > > blobs;
			findBlobs(cpy, blobs);

			//for each region calculate mean and covariance
			vector<vector<Point2i>>::iterator iter = blobs.begin();
			float x, y, xx, yy, ww;
			float range,alpha, rr, aa;

			ushort* bw_data =(ushort*) bw.data;
			int bw_step = bw.step/sizeof(ushort);
			int bw_cols = bw.cols;

			int idPers = 0;
			while (iter != blobs.end())
			{
				bool pass = false;
				vector<Point2i> b = *iter;
				//if (b.size() > 10) //rejection of small regions
				{
					x = y = xx = yy = ww = 0;
					int maxI = b.size();
					for (int i = 0; i < maxI; i++)
					{
						Point2i p = b[i];
					
						float rangeTmp = (bw.rows-p.y)*RANGE_STEP; //binSize
						float thresh;
						if (rangeTmp > 4333)
							thresh = -0.46*rangeTmp+5500;
						else if (rangeTmp <= 4333 && rangeTmp > 3750)
							thresh = -2*rangeTmp + 14000;
						else
							thresh = 0.6*rangeTmp+5500;
	
						int w = (bw_data + p.y*bw_step)[p.x];
						if (w > thresh) //check that there is at least one pixel that get the higher threshold
								pass = true;

						x += (p.x*w);
						y += (p.y*w);
						ww += w;
					}
					if (pass)
					{
						Person* prs = &dtctPpl[ttl_dtctPpl++];
						initPerson(prs);
						prs->mean_RPS = Point(x/ww, y/ww);
						prs->id = idPers++;
						for (int i = 0; i < maxI; i++)
						{
							int rpX = b[i].x;
							int rpY = b[i].y;
							//debug 
							if (debug > DEBUG_MED)//ONLY points that surpass the higher threshold are drawn in yellow
							{
								debugImg.ptr<uchar>(rpY)[rpX*3] = 0;
								debugImg.ptr<uchar>(rpY)[rpX*3+1] = 255;
								debugImg.ptr<uchar>(rpY)[rpX*3+2] = 255;
							}
							int w = (bw_data + rpY*bw_step)[rpX];				
							xx += w*powf(rpX-prs->mean_RPS.x,2);
							yy += w*powf(rpY-prs->mean_RPS.y,2);
						}
						prs->sigmaY_RPS = sqrtf(yy/ww);
						prs->sigmaX_RPS = sqrtf(xx/ww);

						projectLocation2MoA(prs, debug, frames, debugFrame);

					/*	if (frames == debugFrame && prs->id == 0)
						{
							outDebugFile << "Colour model at frame " << frames << endl;
							for (int k = 0; k < MODEL_NBINS; k++)
							{
								//outDebugFile << (float)prs->heightModel[k] << " " << (int)prs->colourModel[k](0) << " " << (int)prs->colourModel[k](1) << " " << (int)prs->colourModel[k](2) << endl;
								outDebugFile << (float)prs->heightModel[k] << " " << (int)prs->colourModel[k].mean(0) << " " << (int)prs->colourModel[k].mean(1) << " " << (int)prs->colourModel[k].mean(2) << endl;
							}
						}*/
					}
				}
				iter++;
			}

		}

		/*
		img : Mat(500,181, CV_16UC1)

		*/

		static void ccDetection(Mat& img, Person* dtctPpl, int& ttl_dtctPpl, const PointMapping* pntsMap, int& ttlPnts, int debug, int frames, int debugFrame)
		{
			Mat imgCpy1, imgCpy2;
			img.copyTo(imgCpy1);

			int img_Rows = img.rows;
			int img_Cols = img.cols;
			ushort* img_data = (ushort*)img.data;
			int img_step = img.step/sizeof(ushort);

			//First threshold
			for (int i = 0; i < img_Rows; i++)
			{
				ushort* ptr = img_data + i*img_step;
				for (int j = 0; j <img_Cols; j++)
				{
					ushort* valPtr = (ushort*)ptr + j;
					int val = (int)*valPtr;
					float range = (img_Rows-(i))*RANGE_STEP; //binSize
			
					//linearize version of the threshold
					float thresh;
					if (range > 4333)
						thresh = -0.46*range+4500;
					else if (range > 3750 && range <= 4333)
					{
						thresh = -2*range+12500;
					}
					//else //if (range > 2000 && range <= 3650)
					//{
					//	thresh = -4.8*range+23000;
					//}
					else
						thresh = 0.6*range + 4000;
			
					/*if (range < 3000)
						thresh = -5*range+20000;
					else
						thresh = -0.73*range+6000;*/

					//float thresh = 50000*exp((-0.0006)*range); //low threshold
					if (val < thresh)
						(*valPtr) = 0;
				}
			}
			img.copyTo(imgCpy2);
	
			Mat outDebug = Mat(RANGE_ROWS, 181, CV_8UC3);	
			if (debug >= DEBUG_MED)
			{
				//Show the results after first thresholding
		
				for (int i = 0; i < imgCpy1.rows; i++)
				{
					ushort* ptrOrig = imgCpy1.ptr<ushort>(i); //original
					ushort* ptrAft = imgCpy2.ptr<ushort>(i); //after low thresholding
					uchar* ptrDebug = outDebug.ptr<uchar>(i);
					for (int j = 0; j < imgCpy1.cols; j++)
					{
						if (ptrOrig[j] > 10 && ptrAft[j] > 10)//Green areas of areas that surpass the low threshold
						{
							ptrDebug[j*3] = 0; ptrDebug[j*3+1] = 255; ptrDebug[j*3+2] = 0; 
						}
						else if (ptrOrig[j] > 10) //Blue areas where the threshold is not surpassed
						{
							ptrDebug[j*3] = 255; ptrDebug[j*3+1] = 0; ptrDebug[j*3+2] = 0;
						}
						else if (ptrAft[j] > 10) //It should never comes to here
						{
							cout << "Hey dude there is a problem in the low threshold" << endl;
							ptrDebug[j*3] = 0; ptrDebug[j*3+1] = 0; ptrDebug[j*3+2] = 255;
						}

					}
				}
			}
			detectCC(img, dtctPpl, ttl_dtctPpl, pntsMap, ttlPnts, outDebug, debug, frames, debugFrame);
			if (debug >= DEBUG_MED)
			{
				//Show the change points in the thresholds
				int rowLow = RANGE_ROWS - ((int)3750/RANGE_STEP);	
				line(outDebug, Point(0,rowLow), Point(RANGE_COLS, rowLow), Scalar(0,0,255));
				imshow("Thresholds", outDebug);	
			}
		}

		//Used to set the orientation of the ellipse; based on the last closest detection
		static Point findNearestPoint(Point pnt, Person* ppl, int ttlPpl)
		{
			Point out = Point (0,0);
			bool found = false;
			int i = 0;
			while (i < ttlPpl && !found)
			{
				int X = ppl[i].stateMoA.at<float>(0,0);
				int Y = ppl[i].stateMoA.at<float>(1,0);
				out.x = X-pnt.x;
				out.y = Y-pnt.y;
				float dist = sqrtf(powf(out.x,2) + powf(out.y,2));
				found = (dist < 40);
				//cout << "distance: " << dist << endl;	
				i++;
			}
			return out;
		}

		//Display detection in both RPS and MoA
		static void displayDetections(Person* trckPpl, int ttl_trckPpl, Mat& remapPolar, Person* pastPpl, int ttlPastppl, Mat& moa, int debug)
		{
			for (int iter = 0; iter < ttl_trckPpl; iter++)
			{
				const Person* p = &(trckPpl[iter]);
				if (p->lost == 0)
				{
					if (debug > DEBUG_NONE)
					{
						cv::circle(remapPolar, p->mean_RPS, 2, Scalar::all(0), -1);
						cv::ellipse(remapPolar, p->mean_RPS, Size(p->sigmaX_RPS*2, p->sigmaY_RPS*2), 0,0,360, Scalar::all(0));
					}

					Point meanMoA = Point(p->stateMoA.at<float>(0,0), p->stateMoA.at<float>(1,0));
					cv::circle(moa, meanMoA, 2, Scalar(0,0,255));

					Scalar color = Scalar(255,0,0);
					float sgX = sqrtf(p->covMoA.at<float>(0,0));
					float sgY = sqrtf(p->covMoA.at<float>(1,1));
					float area = sgX*sgY;
					if (area > 100 && area < 800) 
						color = Scalar(0,255,0);
					else if (area > 800)
						color = Scalar(0,0,255);

					int bigAxis = 20*DEPTH_SCALE;
					int smallAxis = 10*DEPTH_SCALE;

					Point vel = findNearestPoint(meanMoA, pastPpl, ttlPastppl); 
					float angle = 0;
					if (vel.x != 0 && vel.y != 0)
					{
						if ((vel.x > 0 && vel.y > 0) || (vel.x < 0 && vel.y < 0))
						{
							angle = 90 + ((atanf(vel.y/vel.x))*180/CV_PI);
						}
						else
						{
							angle = 90 - ((atanf(vel.y/vel.x))*180/CV_PI);
						}
					}

					cv::ellipse(moa, meanMoA, Size(bigAxis, smallAxis), -angle, 0, 360, color,-1);
					cv::circle(moa, meanMoA, 0.8*smallAxis, Scalar(0,0,0), -1);
				}
			}
		}

		//For debgging purpouses - size of the kernel used in the convolution
		static void addGrid(Mat& img, Size sz)
		{

			for (int c = 0; c <= img.cols-sz.width; c += sz.width)
				line(img, Point(c, 0), Point(c, img.rows), Scalar::all(200));

			for (int r = 0; r < img.rows-sz.height; r += sz.height)
				line(img, Point(0, r), Point(img.cols, r), Scalar::all(200));

		}

		static void copyPerson(Person& dst, const Person* src)
		{
			src->stateMoA.copyTo(dst.stateMoA);
		}

		static void fillArray(const Mat& roi, XnPoint3D* points, int origX)
		{
			int ttlRows = roi.rows;
			int ttlCols = roi.cols;
			for (int i = 0; i < ttlRows; i++)
			{
				const ushort* ptr = roi.ptr<ushort>(i);
				for (int j = 0; j < ttlCols; j++)
				{
					int id = i*ttlCols + j;
					points[id].X = j+origX;
					points[id].Y = i;
					points[id].Z = ptr[j];
				}
			}
		}

		static void maskDepthImage(Mat& roi, const XnPoint3D* points)
		{
			int ttlRows = roi.rows;
			int ttlCols = roi.cols;
			for (int i = 0; i < ttlRows; i++)
			{
				ushort* ptr = roi.ptr<ushort>(i);
				for (int j = 0; j < ttlCols; j++)
				{
					int id = i*ttlCols + j;
					XnPoint3D p = points[id];
					if (p.X < XN_VGA_X_RES && p.X >= 0 && p.Y < XN_VGA_Y_RES && p.Y >= 0)
					{
						ptr[j] = 0;
					}
				}
			}
		}

		static void updateDepthImage(Mat& dImg, Mat& mask)
		{
			for (int i = 0; i < dImg.rows; i++)
			{
				uchar* ptrD = dImg.ptr<uchar>(i);
				ushort* ptrM = mask.ptr<ushort>(i);
				for (int j = 0; j < dImg.cols; j++)
				{
					if (ptrM[j] == 0)
					{
						ptrD[j*3] = 255;
						ptrD[j*3+1] = 255;
						ptrD[j*3+2] = 255;
					}
				
				}
			}

		}

		static Rect calculateRect(KinectSensor* kOut, KinectSensor* k, bool left)
		{
			int angle = 29;
			if (!left)
				angle = -angle;

			XnPoint3D edge3D[1];
			edge3D[0].X = 9700*tanf(angle*CV_PI/180);
			float range = sqrtf( powf(9700,2) + powf(edge3D[0].X,2) );
			edge3D[0].Y = 0;//range * tanf(21.5*CV_PI/180);
			edge3D[0].Z = 9700;
	
			kOut->transformArrayNoTilt(edge3D, 1);
			XnPoint3D* edge2D = k->arrayProject(edge3D, 1);
			int x = edge2D[0].X;
			delete []edge2D;
			Rect out;

			if (left)
				out = Rect(0,0, x, XN_VGA_Y_RES);
			else
				out = Rect(x, 0, XN_VGA_X_RES-x, XN_VGA_Y_RES);

			return out;
		}

		static void maskOutOverlappingSelective(Mat& depthMat, Rect rLeft, Rect rRight, KinectSensor* kinects)
		{
	/*		XnPoint3D edge2D[1];
			XnPoint3D edge3D[1];
			edge2D[0].X = 615; edge2D[0].Y = 240; edge2D[0].Z = 4500;
			kinects[REF_CAM].arrayBackProject(edge2D, edge3D, 1);
			kinects[REF_CAM].transformArrayNoTilt_rev(edge3D, 1, 2);
			XnPoint3D* edge2Dp = kinects[2].arrayProject(edge3D, 1);*/

			int tpLeft = rLeft.width*rLeft.height;
			int tpRight = rRight.width*rRight.height;
			//Go through al the points in the roi and create an array of XnDepth3D
			Mat leftRoi = depthMat(rLeft);
			Mat rightRoi = depthMat(rRight);
			//TODO: may be parameters of the function
			XnPoint3D* leftSide2D = new XnPoint3D[tpLeft];
			XnPoint3D* rightSide2D = new XnPoint3D[tpRight];

			fillArray(leftRoi, leftSide2D, rLeft.x);
			fillArray(rightRoi, rightSide2D, rRight.x);

			//Back project to the 3D space
			XnPoint3D* leftSide3D = new XnPoint3D[tpLeft];
			XnPoint3D* rightSide3D = new XnPoint3D[tpRight];
			kinects[REF_CAM].arrayBackProject(leftSide2D, leftSide3D, tpLeft);
			kinects[REF_CAM].arrayBackProject(rightSide2D, rightSide3D, tpRight);

			// transform into the second CS (make sure it is calibrated the other way around)
		//	XnPoint3D* leftCamP = new XnPoint3D[tpLeft];
		//	XnPoint3D* rightCamP = new XnPoint3D[tpRight];
			kinects[REF_CAM].transformArrayNoTilt_rev(leftSide3D, tpLeft, 0);
			kinects[REF_CAM].transformArrayNoTilt_rev(rightSide3D, tpRight, 2);

			//Project into the image plane
			XnPoint3D* leftCamP = kinects[0].arrayProject(leftSide3D, tpLeft);
			XnPoint3D* rightCamP = kinects[2].arrayProject(rightSide3D, tpRight);

			maskDepthImage(leftRoi, leftCamP);
			maskDepthImage(rightRoi, rightCamP);

			//Free memory
			delete []leftSide2D;
			delete []rightSide2D;
			delete []leftSide3D;
			delete [] rightSide3D;
			delete [] leftCamP;
			delete [] rightCamP;

		}

	//TRACKING FUNCTIONS

		static void displayTrackers(Person* trckPpl, int ttl_trckPpl, Mat& remapPolar, Mat& moa, int debug)
		{
	
			for (int iter = 0; iter < ttl_trckPpl; iter++)
			{
				const Person* p = &(trckPpl[iter]);
		
		

					if (debug > DEBUG_NONE)
					{
						cv::circle(remapPolar, p->mean_RPS, 2, Scalar::all(0), -1);
						cv::ellipse(remapPolar, p->mean_RPS, Size(p->sigmaX_RPS*2, p->sigmaY_RPS*2), 0,0,360, Scalar::all(0));
					}

					Point meanMoA = Point(p->stateMoA.at<float>(0,0), p->stateMoA.at<float>(1,0));
					cv::circle(moa, meanMoA, 2, Scalar(0,0,255));

					Scalar color = Scalar(255,0,0);
					if (p->lost > 0)
						color = Scalar(127,127,127);
					else
					{
						float sgX = sqrtf(p->covMoA.at<float>(0,0));
						float sgY = sqrtf(p->covMoA.at<float>(1,1));
						float area = sgX*sgY;
						if (area > 100 && area < 800) 
							color = Scalar(0,255,0);
						else if (area > 800)
							color = Scalar(0,0,255);
					}
					int bigAxis = 20*DEPTH_SCALE;
					int smallAxis = 10*DEPTH_SCALE;

					//Point vel = findNearestPoint(meanMoA, pastPpl, ttlPastppl); 
					Point vel = Point(p->stateMoA.at<float>(2,0), p->stateMoA.at<float>(3,0));
					float angle = 0;
					if (vel.x != 0 && vel.y != 0)
					{
						if ((vel.x > 0 && vel.y > 0) || (vel.x < 0 && vel.y < 0))
						{
							angle = 90 + ((atanf(vel.y/vel.x))*180/CV_PI);
						}
						else
						{
							angle = 90 - ((atanf(vel.y/vel.x))*180/CV_PI);
						}
					}

					//cv::ellipse(moa, meanMoA, Size(bigAxis, smallAxis), -p->mean_RPS.x, 0, 360, color,-1);
			
					cv::ellipse(moa, meanMoA, Size(bigAxis, smallAxis), -angle, 0, 360, color,-1);
					cv::circle(moa, meanMoA, 0.8*smallAxis, Scalar(0,0,0), -1);

					char txt[15];
					itoa(p->id, txt, 10);
					putText(moa, txt, meanMoA,FONT_HERSHEY_PLAIN, 0.7, Scalar(255,255,255));

					//if (debug > DEBUG_MED)
					//{
					//	float sgGtX = sqrtf(p->gtArea.at<float>(0,0));
					//	float sgGtY = sqrtf(p->gtArea.at<float>(1,1));
					//	cv::ellipse(moa, meanMoA, Size(sgGtY, sgGtX), -p->mean_RPS.x, 0, 360, Scalar(100,100,100));
					//}

						//BEGIN Options from the Polar space

						//Error propagation
						//float sigmaAlphaRad = pPolar.sigmaX*CV_PI/180;
						//float sigmaRange = pPolar.sigmaY;
						//float alpha = pPolar.mean.x*CV_PI/180; //rad
						//float range = pPolar.mean.y; //mm

						//Covariance propagation (option 1: propagate the correlation and the variance)
						/*	float varValues[4] = {powf(pPolar.sigmaY,2), 0,0, powf(sigmaAlphaRad,2)}; 
							Mat covPolar = Mat(2,2, CV_32FC1, varValues);
							float jacobValues[4] = {cosf(alpha)/actMapCreator.xStep, (-range*sinf(alpha))/actMapCreator.xStep, sinf(alpha)/actMapCreator.depthStep, (range*cosf(alpha))/actMapCreator.depthStep}; 
							Mat jacobMat = Mat(2,2, CV_32FC1, jacobValues);
							Mat covCartessian = jacobMat * covPolar * jacobMat.t();
							int sigmaX = sqrtf(covCartessian.at<float>(0,0));
							int sigmaY = sqrtf(covCartessian.at<float>(1,1));*/
						//end Option1

						//Option2: independent variables
						//int sigmaX = sqrtf( powf(cosf(alpha)*sigmaRange/20,2) +  powf(range*sinf(alpha)*sigmaAlphaRad/20,2));
						//int sigmaY = sqrtf( powf(sinf(alpha)*sigmaRange/20,2) + powf(range*cosf(alpha)*sigmaAlphaRad/20,2));
						//end Option2

						//END OPTIONS
		
			}
		}

		static void drawPersonMean_debug(const Person& p, Mat* moa, Scalar color)
		{
			Point pntMean = Point(p.stateMoA.at<float>(0,0),p.stateMoA.at<float>(1,0));

			cv::circle(*moa, pntMean, 2, color, -1);	
		}

		/*
		The detected people is classified based on the area of their bounding boxes 
		*/
		static void look4MergeSplits(Person* dtctPpl, int ttl_dtctPpl)
		{
			for (int i = 0; i < ttl_dtctPpl; i++)
			{
				Person* prs = &dtctPpl[i];
		
				float sgX = sqrtf(prs->covMoA.at<float>(0,0));
				float sgY = sqrtf(prs->covMoA.at<float>(1,1));
				//float area = prs->sigmaX_RPS*prs->sigmaY_RPS;

				float area = sgX*sgY;
				if (area < 50)
					prs->control = DTC_SPLIT;
				else if (area > 800)
					prs->control = DTC_MERGE;
				else
					prs->control = DTC_FULL;
			}
		}

		/*
		Predict the state and covariance of the position 
		*/
		static void predictState(Person*& p, int debug)
		{
			p->stateMoA = p->A * p->stateMoA;

			if (debug > DEBUG_MED)
			{
				outDebugFile << "PREDICT STATE (COVARIANCE UPDATE)" << endl;
				Utils::printValuesF(&p->covMoA, "Covariance matrix (before prediction)", cout);
				Utils::printValuesF(&p->A, "Prediction model (A)", outDebugFile);
				Utils::printValuesF(&p->Q, "Error prediction cov (Q)", outDebugFile);
				Mat tmp = p->A * p->covMoA * p->A.t();
				Utils::printValuesF(&tmp, "(APA')", outDebugFile);
			}

			//CovMoA is the covariance projected from the detection
			p->covMoA = p->Q + p->A * p->covMoA * p->A.t();

			if (debug > DEBUG_MED)
			{
		
				Utils::printValuesF(&p->covMoA, "Covariance matrix (after prediction)", cout);
				outDebugFile << "-----------------------------------" << endl;
			}
		}

		static void drawPersonCov_debug(const Person& p, Mat* moa, Scalar color)
		{
			Point pntMean = Point(p.stateMoA.at<float>(0,0),p.stateMoA.at<float>(1,0));
			SVD svd(p.covMoA(Rect(0,0,2,2)));
			/*printValuesF(&svd.u, "Eigen vectors (U)", outDebugFile);
			printValuesF(&svd.w, "Eigen values (W)", outDebugFile);
			printValuesF(&svd.vt, "Eigen vectors (Vt)", outDebugFile);*/
	
			float bigAxisX = svd.u.at<float>(0,0);
			float bigAxisY = svd.u.at<float>(1,0);

			float angle = atanf(bigAxisY/bigAxisX)*180/CV_PI;

			float bigAxisMag = svd.w.at<float>(0);
			float smallAxisMag = svd.w.at<float>(1);

			cv::ellipse(*moa, pntMean, Size(bigAxisMag, smallAxisMag), angle, 0, 360, color, 3);	
		}

		/*
		Normalize histograms using only bins whith data in both models
		*/
		static void normalizeModels(Person& p1, Person& p2)
		{
			float ttlP1, ttlP2;
			ttlP1 = ttlP2 = 0;
			//get total points;
			for (int i = 0; i < MODEL_NBINS; i++)
			{
				int p1Bin = p1.heightModel[i];
				int p2Bin = p2.heightModel[i];
				if (p1Bin > 0 && p2Bin > 0) //Only bins with data in both models
				{
					ttlP1 += p1Bin;
					ttlP2 += p2Bin;
				}
				else
				{
					p1.heightModel[i] = 0;
					p2.heightModel[i] = 0;
				}
			}
			//normalize
			for (int i = 0; i < MODEL_NBINS; i++)
			{
				if (ttlP1 > 0 && ttlP2 > 0)
				{
					p1.heightModel[i] /= ttlP1;
					p2.heightModel[i] /= ttlP2;
				}
				else
					outDebugFile << "normilizeModels: Totalpoints cannot be equal 0: Total p1: " << ttlP1 <<". Total p2: " << ttlP2 << endl;
		
			}
		}

		static float bhattacharyaDist_Cont(const gaussianParam* trgt, const gaussianParam* cndt, int debug, int frames)
		{
			//dist = 8pi det(S_tgt*S_cndt)^(1/4) * exp( (Mean_trt - Mea_cndt)' * S^-1 * (Mean_trt - Mean_cndt) )/2 )

			Mat S_trgt = trgt->cov;
			Mat S_cndt = cndt->cov;

			//Means subtraction
			Mat meanSub = Mat(3,1, CV_32F);
			meanSub.at<float>(0) = trgt->mean(0) - cndt->mean(0);
			meanSub.at<float>(1) = trgt->mean(1) - cndt->mean(1);
			meanSub.at<float>(2) = trgt->mean(2) - cndt->mean(2);

			//Pooled sigma
			if (determinant(S_cndt) < 0.0001)
			{
				S_cndt.at<float>(0,0) +=1;
				S_cndt.at<float>(1,1) +=1;
				S_cndt.at<float>(2,2) +=1;
			}

			Mat S1 = S_trgt + S_cndt;
			Mat S2 = 2*S1;
			Mat S3 = S_trgt*S_cndt;
			Mat S2_Inv = S2.inv();
			double S1det = determinant(S1);
			double S3det = determinant(S3);


			if (debug == DEBUG_MED && frames == 96)
			{
				Utils::printValuesF(&trgt->cov, "Covariance target", outDebugFile);
				Utils::printValuesF(&cndt->cov, "Covariance CAndidate", outDebugFile);
				Utils::printValuesF(&S1, "pooled Cov(strgt+scndt)", outDebugFile);
				outDebugFile << "Det(Strg + Scndt): " << S1det << endl;
				Utils::printValuesF(&S2_Inv, "pooled Cov inv", outDebugFile);
			}
	 	
			Mat mahalanobisDist = meanSub.t() * S2_Inv * meanSub;
			float mDist = mahalanobisDist.at<float>(0);
			float normTerm = (sqrtf(2)* powf( S3det, 0.25))/ sqrtf(S1det);
			if (S1det != 0 && S3det != 0)
			{
				float out = normTerm * exp(-mDist/2);
				return out;
			}
			else
			{
				outDebugFile << "CompareColours. Error Determinant = 0. Info: Pooled Sigma det(Strgt + SCndt): " << S1det  <<
					" det(Strgt*Scndt): " << S3det << endl;
				return -1;
			}
		}

		/*
		A detected person is compared with the target using the appearance model
		*/
		static float compareModels(const Person* target, const Person* candidate, int debug, int frames)
		{
			//copy models
			Person trgtCpy = *target;
			Person cndtCpy = *candidate;

			assert(&trgtCpy != target);
			assert(&cndtCpy != candidate);

			normalizeModels(trgtCpy, cndtCpy);
	
			float bCoeff = 0;

			if (debug == DEBUG_MED && frames == 123)
			{
				outDebugFile << "Compare Models - trgtId: " << target->id << " cndtId: " << candidate->id << " BEGINS " << endl;
			}
			for (int i = 0; i < MODEL_NBINS; i++)
			{
				//Using the mean colours
				//bCoeff += sqrtf(trgtCpy.heightModel[i] * cndtCpy.heightModel[i] * compareColours(trgtCpy.colourModel[i].mean, cndtCpy.colourModel[i].mean));
				//Using the colour distributions
				float hT = trgtCpy.heightModel[i];
				float hC = cndtCpy.heightModel[i];
				if (hT > 0.05 && hC > 0.05)
				{
					gaussianParam colourT = trgtCpy.colourModel[i];
					gaussianParam colourC = cndtCpy.colourModel[i];
					//float cC = compareColours(&colourT, &colourC);
					//outDebugFile << "CompareColours- bin " << i << " : " << cC << endl;

					float hBinD = sqrtf(hT*hC);
					float colourBinD = bhattacharyaDist_Cont(&colourT, &colourC, debug, frames); 

					if (debug == DEBUG_HIGH)
						outDebugFile << "Bin " << i << " Height comparison: " << hBinD << ". ColourComarison: " << colourBinD << endl;

			
					bCoeff += hBinD*colourBinD;			
				}
			}
	
			if (debug == DEBUG_MED)
			{
				outDebugFile << "ENDS: Compare Models - trgtId: " << target->id << " cndtId: " << candidate->id << ": " << bCoeff << endl;
			}
			if (bCoeff < 0 || bCoeff > 0.52) //it should be 0.5 but due to decimal errors
			{
				outDebugFile << "CompareModels: Error Similarity coefficient out of range: " << bCoeff << "trgtId: " << target->id << " cndtId: " <<
					candidate->id << ". Frame: " << frames << endl;
			}
			return bCoeff;
		}

		
		/*
		out = (dPrs.mean - target.mean) * inv((target.gtCov + dPrs.covMoa)/2) * (dPrs.mean - target.mean)T
		*/
		static float mahalanobis(const Person* target, const Person* dPrs)
		{
			//Mean diff
			Mat tgtMean = target->stateMoA(Rect(0,0, 1,2));
			Mat dtcMean = dPrs->stateMoA(Rect(0,0,1,2));
			Mat meanDiff = dtcMean-tgtMean;

			//sum covariances
			Mat tgtCov = target->covMoA(Rect(0,0,2,2));
			//Mat dtcCov = dPrs->covMoA(Rect(0,0,2,2));
			//Mat avgCov = (tgtCov + dtcCov)/2;

			Mat out = meanDiff.t() * tgtCov.inv() * meanDiff;

			float val = sqrtf(out.at<float>(0,0));

			return val;

		}

		static void gateDetection(const Person* target,  Person* dtctPpl, int ttl_dtctPpl, Person** gPpl, int& ttl_gPpl, int debug)
		{
			for (int i = 0; i < ttl_dtctPpl; i++)
			{
				Person* dPrs = &dtctPpl[i];
				if (!dPrs->associated)
				{
					float c = mahalanobis(target, dPrs);
					if (debug == DEBUG_MED)
					{
						outDebugFile << "Mahalanobis (tgt id: " << target->id << ". dtct id:  " << dPrs->id << "): " << c << endl;
					}
					if (c < 10)
					{
						gPpl[ttl_gPpl] = dPrs;
						ttl_gPpl++;
					}
				}
			}
		}


		/*
		Look the closer detection with the target (location and appearance).
		1- Defines a gating area (filter out detections far from the predicted location)
		2- Compare appearance models

		IN:
		Target, dtectPpl, ttl_dtctPpl

		OUT:
		candidate, compCoeff (it will be used later to decide if the appearance update is performed or not.
		*/
		static void association(const Person* target,  Person* dtctPpl, int ttl_dtctPpl, Person*& candidate, float& compCoeff, int debug, int frames)
		{
			//Filter all detections within a gate region
			Person** gPpl = new Person*[ttl_dtctPpl];
			int ttl_gPpl = 0;
			gateDetection(target, dtctPpl, ttl_dtctPpl, gPpl, ttl_gPpl, debug);
	
			for (int i = 0; i < ttl_gPpl; i++)
			{
				Person* v = gPpl[i];
				float c = compareModels(target, v, debug, frames);

				if (c > compCoeff)
				{
					compCoeff = c;
					//if (candidate == NULL)
					//	candidate = new Person();
					candidate = v;
					assert(candidate, v);
				}
			}
			delete []gPpl;
		}


		/*
		Update the target state (x,y,vx,vy) from the measurement state through the kalman equations 
		*/
		static void updateState(Person* trgt, const Person* msr, float dComp, int debug)
		{
			//K = 4x2 matrix
			//Biggest R entails smallest K and viceversa
			Mat K = trgt->covMoA * H.t() * (H * trgt->covMoA * H.t() + trgt->R).inv();

			//2x1 location of measurement
			Mat msreState = msr->stateMoA(Rect(0,0,1,2));
			//2x1 location of the target
			Mat trgtState = trgt->stateMoA(Rect(0,0,1,2));
			//UPDATE EQUATION
			trgt->stateMoA = trgt->stateMoA + K*(msreState - H*trgt->stateMoA);
	
			trgt->lost = 0;

			//update the covariance 
			Mat I = Mat::eye(4,4,CV_32F);
			trgt->covMoA = (I - (K*H))*trgt->covMoA;

			if (debug > DEBUG_MED)
			{
				outDebugFile << "UPDATE STATE" << endl;
				Utils::printValuesF(&I, "Identity matrix", outDebugFile);
				Utils::printValuesF(&K, "K", outDebugFile);
				Utils::printValuesF(&H, "H", outDebugFile);
				Utils::printValuesF(&trgt->covMoA, "Covariance Matrix", outDebugFile); 
				Utils::printValuesF(&trgt->covMoA, "Covariance Matrix (Updated)", outDebugFile); 
			}

		}

		/*
		Updates the model considereing the different bins
		*/
		static void updateModel(Person* target, const Person* msr)
		{
			float updateCoeff = 0;
			for (int i = 0; i < MODEL_NBINS; i++)
			{
				int numHeightMsr = msr->heightModel[i];
				int numHeightTrgt = target->heightModel[i];
				if (numHeightMsr > 0)
				{
					//case 1: full update
					if (numHeightTrgt > 0)
						updateCoeff = ALPHA;
			
					//Case 2: slow update
					else 
						updateCoeff = BETA;
		
					//update hight
					target->heightModel[i] = numHeightTrgt*(1 - updateCoeff) + numHeightMsr*updateCoeff;
					//update colour
					gaussianParam* clrMdlTrgt = &target->colourModel[i];
					const gaussianParam* clrMdlCndt = &msr->colourModel[i];
					for (int c = 0; c < 3; c++)
					{
						clrMdlTrgt->mean(c) = clrMdlTrgt->mean(c)*(1 - updateCoeff) + clrMdlCndt->mean(c)*updateCoeff;	

						float* covTgt = clrMdlTrgt->cov.ptr<float>(c);
						const float* covCndt = clrMdlCndt->cov.ptr<float>(c);
						for (int r = 0;  r < 3; r++)
							covTgt[r] =covTgt[r]*(1 - updateCoeff) + covCndt[r]*updateCoeff;
				
					}
				}
			}

		}

		/*
		Create a new list without the losts and with the new tracks
		*/
		static void checkEnd_NewTracks(Person*& trckPpl, int& ttl_trckPpl, Person*& dtctPpl, int& ttl_dtctPpl)
		{
			Person* out = new Person[ttl_trckPpl+ttl_dtctPpl];
			int cont = 0;
			//int maxId = -1;
			//look for end tracks
			for (int i = 0; i < ttl_trckPpl; i++)
			{
				Person* trck = &trckPpl[i];
				if (trck->lost < TRACKLOST_THRESHOLD)
				{
					out[cont++] = *trck;	
					//if (trck->id > maxId)
						//maxId = trck->id;
				}
			}

			//look for new tracks
			for (int i = 0; i < ttl_dtctPpl; i++)
			{
				Person* dtc = &dtctPpl[i];
				if (!dtc->associated && dtc->control == DTC_FULL)
				{
					dtc->id = pplId_cont++;
					out[cont++] = *dtc;
				}
			}

			//Transfer all data from out to trckPpl 
			for (int i = 0; i < cont; i++)
			{
				trckPpl[i] = out[i];
			}
			ttl_trckPpl = cont;
			delete [] out;
		}



		static void tracking(Person*& trckPpl, int& ttl_trckPpl, Person*& dtctPpl, int& ttl_dtctPpl, Mat* moa, int debug, int frames)
		{
	
			//Based on the area covered by the person
			look4MergeSplits(dtctPpl, ttl_dtctPpl);
			for (int i = 0; i < ttl_trckPpl; i++)
			{
				Person* target = &trckPpl[i];

				//Instead of removing the lost tracks from the list trckPpl (require the realocation of the rest of tracks)
				//It uses a flag to check if the target is lost
				if (target->lost <= TRACKLOST_THRESHOLD)
				{
			
					//Predict the state of the target using the motion model.
					predictState(target, debug);

					//debug draw the predict position and uncertainty
					if (debug == DEBUG_MED)
					{
						drawPersonCov_debug(*target, moa,Scalar(127,127,127));
						Utils::printValuesF(&target->covMoA, "CovMat", cout);
						//imshow(windMoA, *moa);
						//waitKey(0);
					}

					//Find the measurement generated by the target
					Person* measur = NULL;
					float compDist = -1;
					if (frames == 96 && target->id == 1)
						cout << "Stop dude" << endl;
					association(target, dtctPpl, ttl_dtctPpl, measur, compDist, debug, frames);

					if (measur != NULL)
					{
						measur->associated = true;
						//Update the position of the target using the Kalman equations.
						updateState(target, measur, compDist, debug);

						//Only updates the height and colour model when it is "certain" that
						//the candidate corresponds to the target. Also it needs to be fully detected (not splits or merges)
		//				if (debug > DEBUG_MED)
		//					outDebugFile << "Comparison coefficient: " << compDist << ". COMPARISON THRESHOLD: " << COMP_THRESHOLD << endl;

						if (compDist < COMP_THRESHOLD && measur->control == DTC_FULL)
							updateModel(target, measur);	

					}
					else
						target->lost++;
				}
			}
			checkEnd_NewTracks(trckPpl, ttl_trckPpl, dtctPpl, ttl_dtctPpl);
		}


		/*
		Update the tracks trajectory with the information from the current frames.
		Used to store in memory the trajectories of all tracks in the entire sequence
		*/
		static void generateTrackHistory(Person*& trckPpl, int& ttl_trckPpl, int frames)
		{
			for (int i = 0; i < ttl_trckPpl; i++)
			{
				//Create the position at current frame
				Person p = trckPpl[i];
				Position pos;
				pos.frameId = frames;
				Point mean = Point(p.stateMoA.at<float>(0), p.stateMoA.at<float>(1));
				float varX = sqrtf(p.covMoA.at<float>(0,0));
				float varY = sqrtf(p.covMoA.at<float>(1,1));
				pos.bbox = Rect(mean.x-varX, mean.y-varY, varX*2, varY*2);

				bool found = false;
				int iter = 0;
				while (!found && iter < tracks.size())
				{		
					found = (trckPpl[i].id == tracks[iter].id);
					iter++;
				}
				if (!found)
				{
					Track trck;
					trck.id = trckPpl[i].id;
					trck.trajectory.push_back(pos);
					tracks.push_back(trck);
				}
				else
					tracks[--iter].trajectory.push_back(pos);
			}
		}

}

