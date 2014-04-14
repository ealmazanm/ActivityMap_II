#pragma once
#include <Utils.h>
#include <vld.h>

#include <tinystr.h>
#include <tinyxml.h>

namespace AM
{
	//INTERVALS TIME EXECUTION
		static const int TOTAL_INTERVALS = 10;
		static const int BSUB_ID = 0;
		static const int RPSPACE_ID = 1;
		static const int MOA_ID = 2;
		static const int DET_ID = 3;
		static const int BACKPR_ID = 4;
		static const int PTRANS_ID = 5;
		static const int SMOOTH_ID = 6;
		static const int DISPLAY_ID = 7;
		static const int TRACK_ID = 8;
		static const int TOT_ID = 9;
		static char* titles[TOTAL_INTERVALS] = {"BACKGROUND SUBTRACTION", "REMAP POLAR SPACE", "MAP OF ACTIVITY", "DETECTION", "POINTS BACKPROJECTION", "POINTS TRANSFORMATION", "SMOOTH", "DISPLAY", "TRACKING", " TOTAL"};
		static float totalIntervals[TOTAL_INTERVALS] = {0,0,0,0,0,0,0,0,0,0};

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

		//Time interval sub-function People detection
		static const int TOTAL_SUBINTERVAL_DETECTION = 4;
		static const int FIRSTTHRES_ID = 0;
		static const int BLOBS_ID = 1;
		static const int BUILDAPPEARANCE_ID = 2;
		static const int PROJECT2MOA_ID = 3;
		static char* titles_subDet[TOTAL_SUBINTERVAL_DETECTION] = {"FIRST THRESHOLDS", "BLOB DETECTION", "BULD AAPEARANCE", "PROJECT 2 MOA"};
		static float totalSubIntervalsDetection[TOTAL_SUBINTERVAL_DETECTION] = {0,0,0,0};

		//Time interval sub sub function BUILD APPEARANCE MODEL
		static const int TOTAL_SUBSUBINTERVAL_APP = 6;
		static const int IDENTIFY_PREP_ID = 0;
		static const int IDENTIFY_PREP2_ID = 1;
		static const int IDENTIFY_MULT_ID = 2;
		static const int IDENTIFY_GETVAL_ID = 3;
		static const int UPDATEMODEL_ID = 4;
		static const int COVAR_ID = 5;
		static char* titles_subsubAPP[TOTAL_SUBSUBINTERVAL_APP] = {"IDENTIFY_ PREPARAION", "IDENTIFY_ PREPARAION2", "IDENTIFY_MULT", "IDENTIFY_GETVAL", "UPDATE MODEL", "COVARIANCE"};
		static float totalSubSubIntevalsAPP[TOTAL_SUBSUBINTERVAL_APP] = {0,0,0,0,0,0};


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
		static const int MODEL_MAX_HEIGHT = -400;
		static const int MODEL_MIN_HEIGHT = -2400;
		static const int MODEL_BINRANGE = 250;
		static const int MODEL_NBINS = (MODEL_MAX_HEIGHT-MODEL_MIN_HEIGHT)/MODEL_BINRANGE;
		//static const int MODEL_NBINS = (ActivityMap_Utils::CEILING_THRESHOLD-ActivityMap_Utils::FLOOR_THRESHOLD)/MODEL_BINRANGE;
		static const int MAX_PEOPLE = 60;

		static float meas_std = 1;
		static float proc_std = 30;
		static const int MAX_POINTS_BIN = 20000;
		struct gaussianParam
		{
			Scalar mean;
			Mat cov;

			//Debug
			double rr, gg, bb, rg, rb, gb;
			//const XnRGB24Pixel** colours;
			//int totalColours;
		};

		static int AREA_THRESHOLD = 160;
		static int GATING_THRESHOLD = 4;
		struct Person
		{
			int id;
			Mat stateMoA;
			Mat stateUncertainty;
			Mat covMoA_points;
			Mat innUncertainty;
			Point2d  mean_RPS;
			double sigmaY_RPS;
			double sigmaX_RPS;
			Mat sigmaRPS_inv;
			Mat R;
			Mat Q;
			Mat A; 
			Mat K; //Kalman gain for debuggin
	
			float heightModel[MODEL_NBINS];
			gaussianParam colourModel[MODEL_NBINS];
			int control;
			int lost;
			bool associated;

			Scalar colour;

			//debug
			float maxHeight;
			float minHeight;
			int idDetection;
			Mat apperance;
			int countApperance[MODEL_NBINS];

			//debugging
			vector<Point> moAPoints;
			Mat covMoA2;
			Point meanMoA2;
			Point meanDtction;
			Mat covDtction;
			XnPoint3D mean3D;
			float euclThresh;
			float range3D;//probably not needed
			Mat rotation; //matrix that rotates a covariance matrix depending on its mean position in the MoA
			Mat stateUn_noupdate;
			Point mean_noupdate;
			//Only when it is a merge detection
			int numAsso; //counts the number of target associations
			vector<int> associatedIds; //Store the id's of the targets associated
		};

		struct EllipseParam
		{
			Point mean;
			double covX;
			double covY;
			double covXY;
		};

		struct DCF
		{
			int idFrame;
			vector<int> trgtIds;
			vector<EllipseParam> gd_Prms;
			EllipseParam avgEllipse;
		};

		struct Merge
		{
			vector<int> tgtIds;
			int initFrame;
			int endFrame;
			int id;
			vector<EllipseParam> gd_Prms;
		};


		struct PointMapping
		{
			const XnPoint3D* p3D;
			const XnRGB24Pixel* colour;
			int rpX;
			int rpY;
			//debug
			int cam;
			XnPoint3D p2D;
			int idPerson;
		};

		//Function parameters
		//Euclidean threshold for the gatting volume
		static float gateEuclidean[] = {0.05, 0.038};

	//Tracking variables
		static const float MEASUREMENT_RHO_ERROR = 400; //it will be down scale to a maximum of 45%
		static const float MEASUREMENT_THETA_ERROR = 40; //constant
		static const float PROCESS_ERROR = 50;
		static const int DTC_FULL = 0;
		static const int DTC_MERGE = 1;
		static const int DTC_SPLIT = 2;
		static const int TRACKLOST_THRESHOLD = 10;
		static const int EUCLIDEAN_THRESHOLD = 150;
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
			float covX;
			float covY;
			float covXY;
			Point mean;
			//Rect bbox;
		};
		struct TrackInfo
		{
			int id;
			vector<Position> trajectory;
		};
		//static vector<Track> tracks;

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
		static ofstream outMoaPnts("c:\\Dropbox\\PhD\\Matlab\\MoAPoints\\distributionMoA.txt");
		static ofstream outRPSPnts("c:\\Dropbox\\PhD\\Matlab\\MoAPoints\\distributionRPS.txt");
		static ofstream outDataAss("c:\\Dropbox\\PhD\\Matlab\\DataAssociation\\SequenceB\\validatedMeasurements.txt");
		static ofstream outMerge("c:\\Dropbox\\PhD\\Matlab\\DataAssociation\\SequenceB\\mergeMeasurements.txt");
		static const char* out_dcfs_system ("d:\\Emilio\\Tracking\\DataSet\\sb125\\SecondDay\\DSet2\\dcfs_system.xml");
		static const char* out_merge_system ("d:\\Emilio\\Tracking\\DataSet\\sb125\\SecondDay\\DSet2\\mergeMeasurements_system.xml");


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
			Utils::printValuesF(&p->stateUncertainty, "Cov MoA", outDebugFile);
			//Utils::printValuesF(&p->gtArea, "Gating area" , outDebugFile);
			Utils::printValuesF(&p->A , "Prediction model (A)", outDebugFile);
			Utils::printValuesF(&p->Q, "Prediction error covariance (Q)", outDebugFile);
			Utils::printValuesF(&p->R, "Measurement error covariance (R)", outDebugFile);
			outDebugFile << "Lost: " << p->lost << ". Lost threshold: " << TRACKLOST_THRESHOLD << endl;
			outDebugFile << "*****************************************" << endl;
		}

		/*
		activityMap(out): Updates the positions where the 3D points project.
		actimityMap_back(out): With the background
		featureSpace(out): Feature space of colour and height. Modelled over a plan view
		am(in): 
		p3D(in): List of foreground points for a particular kinect
		nP(in): Number of foreground points.
		points2D(in): List of 2D points of the image plane for a particular kinect
		rgbMap(in): Map of rgb colours for a particular kinect
		*/
		static void updateActivityMap(Mat& activityMap, Mat& activityMap_back, const ActivityMap_Utils* am, const XnPoint3D* p3D, const int nP, const XnPoint3D* points2D)
		{
			for (int i = 0; i < nP; i++)
			{
				Point p2D = ActivityMap_Utils::findMoACoordinate(&p3D[i], MAX_RANGE, MODEL_MAX_HEIGHT, MODEL_MIN_HEIGHT);

				if (p2D.x != -1)
				{
					uchar* ptr = activityMap.ptr<uchar>(p2D.y);
					uchar* ptr_back = activityMap_back.ptr<uchar>(p2D.y);

					ptr[3*p2D.x] = ptr_back[3*p2D.x] = 0;
					ptr[3*p2D.x+1] = ptr_back[3*p2D.x+1] = 0;
					ptr[3*p2D.x+2] = ptr_back[3*p2D.x+2] = 0;
				}
				//else
				//{
				//	if (debug > DEBUG_NONE)
				//		outDebugFile << "UpdateActivityMap: BAD PROJECTION (MoA). Info: 3DPoint: " << p3D[i].X <<", "<< p3D[i].Y << ", " << p3D[i].Z <<
				//		"MAX_RANGE: " << MAX_RANGE << ". MAX_Z_TRANS: " << ActivityMap_Utils::MAX_Z_TRANS << ". CEILING_THRESH: " << ActivityMap_Utils::CEILING_THRESHOLD <<
				//		". FLOOR_THRESHOLD: " << ActivityMap_Utils::FLOOR_THRESHOLD << endl;
				//}
			}
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
		static void updatePolarAlternateive(Mat* polarAlt, Mat* polar, vector<PointMapping>* pntsMap2 , int& ttlPnts, const XnPoint3D* p3D, const XnPoint3D* points2D, const XnRGB24Pixel* rgbMap, const int nP, int debug, int camId)
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
				//if (range > MAX_RANGE || Z > ActivityMap_Utils::MAX_Z_TRANS  || Y > ActivityMap_Utils::CEILING_THRESHOLD || Y < ActivityMap_Utils::FLOOR_THRESHOLD)
				if (range > MAX_RANGE || Z > ActivityMap_Utils::MAX_Z_TRANS  || Y > MODEL_MAX_HEIGHT || Y < MODEL_MIN_HEIGHT)
				{
					//if (debug > DEBUG_NONE)
					//	outDebugFile << "UpdatePolarAlternative: 3D Point Rejectected. Info: 3DPoint: " << p.X <<", "<< p.Y << ", " << p.Z << 
					//	". Range: " << range << ". MAX_RANGE: " << MAX_RANGE << ". MAX_Z_TRANS: " << ActivityMap_Utils::MAX_Z_TRANS << ". CEILING_THRESHOLD: " <<
					//	ActivityMap_Utils::CEILING_THRESHOLD << ". FLOOR_THRESHOLD: " << ActivityMap_Utils::FLOOR_THRESHOLD << endl;
					continue; //TODO: ADD A LOG OUTPUT
				}

				Point meanMoA = ActivityMap_Utils::findMoACoordinate(&p3D[i], MAX_RANGE, MODEL_MAX_HEIGHT, MODEL_MIN_HEIGHT);
	
				startTime = clock(); //time debuggin
				int angle = 90;
				if (X != 0 && Z != 0)
					if (X > 0)
						angle = ((atanf(Z/abs(X)))*toRad)+0.5; //rounds to the nearest int
					else
						angle = (180-((atanf(Z/abs(X)))*toRad))+0.5; //rounds to the nearest int
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
				XnPoint3D p2d = (*(points2D+i));
				int rgbMapPos = (int)p2d.Y*XN_VGA_X_RES+(int)p2d.X;
				pMap.colour = rgbMap + rgbMapPos;
				//debug
				pMap.cam = camId;
				pMap.p2D = p2d;

				int id = yPos*RANGE_COLS+angle;
				pntsMap2[id].push_back(pMap);
				//pntsMap[ttlPnts++] = pMap;
				totalSubIntervalsRPS[FEATURE_ID] += clock() - startTime; //time debugging


				if (debug >= DEBUG_NONE)
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


		static void findBlobs(const cv::Mat &binary, std::vector < std::vector<cv::Point2d> > &blobs)
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

					std::vector <cv::Point2d> blob;

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
		static Point convertBack(const Point2d* p)
		{
			Point out;
			//Calculate range
			float rangeAlt = (RANGE_ROWS - p->y)*RANGE_STEP + RANGE_STEP/2;
			float range = (-3.83328*(0.869577-exp( (3.0303*rangeAlt)/SCALE_RANGE_ALT) ) )*1000;
			out.x = range*cos((p->x)*CV_PI/180); //+4 there is a misplacement of around 4 degrees in the estimation (ToDo: find out why)
			out.y = range*sin((p->x)*CV_PI/180);
			return out;
		}

		static void printValuesF(const Mat* m, char* title, ostream& out)
		{
			out << title << endl;
			for (int i = 0; i < m->rows; i++)
			{
				const float* ptr = m->ptr<float>(i);
				for (int j = 0; j < m->cols; j++)
				{
					out << (float)ptr[j] << " ";
				}
				out << endl;
			}
			out << endl;
		}

		/*
		Initialize the values of the bins in the height and colour model
		*/
		static void initPerson(Person* p, int debug)
		{
			p->euclThresh = EUCLIDEAN_THRESHOLD;

			//Covariance of the prediction error
			//first iteration the prediction model is trusted 0%
			float valsQ[] = {1,0,0,0,   0,1,0,0,  0,0,1,0,  0,0,0,1};
			Mat m = Mat(4,4, CV_32F, valsQ);
			m = m*PROCESS_ERROR;
			m.copyTo(p->Q);
			m.copyTo(p->stateUn_noupdate);

			float valsR[] = {1,0,0,1};
			m = Mat(2,2, CV_32F, valsR);
			m = m*MEASUREMENT_RHO_ERROR;
			m.copyTo(p->R);

			//Prediction linear model
			float valsA[] = {1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1};
			Mat(4,4, CV_32F, valsA).copyTo(p->A);
			
			//p->stateMoA = Mat::zeros(4,1,CV_32F);
			float valState[] = {0, 0, 0, 0};
			Mat(4,1,CV_32F, valState).copyTo(p->stateMoA);
			p->stateUncertainty = Mat::zeros(4,4, CV_32F);
			p->covMoA_points = Mat::zeros(2,2,CV_32F);
			p->innUncertainty = Mat::zeros(2,2, CV_32F);
	

			


			p->control = DTC_FULL;
			p->lost = 0;
			p->associated = false;

			//Image for debuggin to show the apperance model
			p->apperance = Mat(400,370, CV_8UC3);
			Utils::initMat3u(p->apperance, 255);
			for (int i = 1; i < MODEL_NBINS; i++)
			{
				int row =  i*50;
				line(p->apperance, Point(0, row), Point(370, row), Scalar::all(0));
			}
			line(p->apperance, Point(50,0), Point(50, 400), Scalar::all(0));
			for (int i = 0; i < MODEL_NBINS; i++)
			{
				p->heightModel[i] = 0;
				p->colourModel[i].cov = Mat::zeros(3,3, CV_32F);
				//p->colourModel[i].totalColours = 0;
				//p->colourModel[i].colours = new const XnRGB24Pixel*[MAX_POINTS_BIN];
				p->colourModel[i].rr = p->colourModel[i].gg = p->colourModel[i].bb = 0;
				p->colourModel[i].rg = p->colourModel[i].rb = p->colourModel[i].gb = 0;
				p->countApperance[i] = 0;
			}

			//for debuggin
			p->maxHeight = -2000;
			p->minHeight = 300;
	
			if (debug == DEBUG_HIGH)
			{
				printValuesF(&p->R, "R(init)", outDebugFile);
				printValuesF(&p->Q, "Q(init)", cout);
				printValuesF(&p->A, "A(init)", cout);
				printValuesF(&p->stateUncertainty, "CovMoa(init)", cout);
			}
			p->covMoA2 = Mat::zeros(2,2,CV_32F);
		}

		static void projectCovariance_Jacob(Person* prs)
		{
			//Covariance projection to MoA
			float sigmaAlphaRad = prs->sigmaX_RPS*CV_PI/180;
			float sigmaRange = prs->sigmaY_RPS;
			float meanAlpha = prs->mean_RPS.x*CV_PI/180; //rad
			float meanRange = prs->mean_RPS.y; //mm
						
			float jacob_11 = -2.09059*exp(-0.00280843*meanRange)*cosf(meanAlpha);
			float jacob_12 = (158.73 - 744.397*exp(-0.00280843*meanRange))*sinf(meanAlpha);
			float jacob_21 = 2.09059*exp(-0.00280843*meanRange)*sinf(meanAlpha);
			float jacob_22 = (158.73-744.397*exp(-0.00280843*meanRange))*cosf(meanAlpha);

			float jacobValues[4] = {jacob_11, jacob_12, jacob_21, jacob_22}; 
			Mat jacobMat = Mat(2,2, CV_32F, jacobValues);
			//covariance matrix in the plan view remap polar space(RPS)
			float varValues[4] = {sigmaRange*sigmaRange, 0,0, sigmaAlphaRad*sigmaAlphaRad}; 
			Mat covPolar = Mat(2,2, CV_32F, varValues);
			//Covariance approximation in the plan view MoA
			Mat covCartessian = jacobMat * covPolar * jacobMat.t();

			//printValuesF(&covCartessian, "CovCartessian_Jacob" , cout);

			covCartessian.copyTo(prs->stateUncertainty(Rect(0,0,2,2)));
			covCartessian.copyTo(prs->covMoA_points);
			//To keep detection values
			covCartessian.copyTo(prs->covDtction);
		}


		//This is explained in "KinectDepthSensor_Analysis_updated" document
		static void projectCovariance_Var(Person* prs)
		{

			//derivative of the transformation function that goes from the rps to the polar space
			int muRho_ = (RANGE_ROWS+0.5 - prs->mean_RPS.y)*RANGE_STEP;
			int muRho = -3833.28*(0.869577 - exp(0.000117019*muRho_));
			float fDerv = 0.448567*exp(0.000117019 * muRho_);
			float sigmaRho_ = prs->sigmaY_RPS * RANGE_STEP;
			float sigmaRho = sigmaRho_ * fDerv;
			float sigmaTheta = muRho*tanf(prs->sigmaX_RPS*CV_PI/180);

			float sigmaY_Moa = sigmaRho/ActivityMap_Utils::Z_STEP;
			float sigmaX_Moa = sigmaTheta/ActivityMap_Utils::X_STEP;

			float covValues[] = {sigmaY_Moa*sigmaY_Moa, 0, 0, sigmaX_Moa*sigmaX_Moa};
			Mat covMoA = Mat(2,2, CV_32F, covValues);

			Mat R;
			float meanTheta_rad = prs->mean_RPS.x*CV_PI/180;
			float cosTheta = cosf(meanTheta_rad);
			float vals[] = {cosTheta, sinf(meanTheta_rad), -sinf(meanTheta_rad), cosTheta}; 
			R = Mat(2,2, CV_32F, vals);
			R.copyTo(prs->rotation);
			
			//printValuesF(&prs->rotation, "Rotation I", outDebugFile);

			Mat covCartessian = R*covMoA*R.t();

			//printValuesF(&covCartessian, "CovCartessian_var" , cout);

			covCartessian.copyTo(prs->stateUncertainty(Rect(0,0,2,2)));
			covCartessian.copyTo(prs->covMoA_points);
			//To keep detection values
			covCartessian.copyTo(prs->covDtction);

		}

		static void setTimeVariantParameters(Person* prs, int debug, int frames)
		{
			float range = sqrtf(pow(prs->mean3D.X,2) + pow(prs->mean3D.Z,2));
			float scale = ((0.33*log(10+3*range/1000))-0.80597);
	
			float valsR[] = {MEASUREMENT_RHO_ERROR*scale,0,0,MEASUREMENT_THETA_ERROR};
			Mat m = Mat(2,2, CV_32F, valsR);
			Mat s = (prs->rotation*m*prs->rotation.t());
			s.copyTo(prs->R);
			prs->range3D = range; //probably not needed
			prs->euclThresh = scale*EUCLIDEAN_THRESHOLD; //probably not needed
  		}


		//Projects the detected gaussian distribution (mean, variance) to the MoA (non linear)
		static void projectLocation2MoA(Person* prs, int debug, int frames, int debugFrame)
		{
			//Mean projection to MoA
			Point cMoA;
			cMoA = convertBack(&prs->mean_RPS);
			//XnPoint3D p3D;
			prs->mean3D.X = cMoA.x; prs->mean3D.Y = MODEL_MAX_HEIGHT - 1; prs->mean3D.Z = cMoA.y;
			Point meanMoA = ActivityMap_Utils::findMoACoordinate(&prs->mean3D, MAX_RANGE, MODEL_MAX_HEIGHT, MODEL_MIN_HEIGHT);
			if (meanMoA.x == -1)
			{
				if (debug > DEBUG_NONE)
				{
					outDebugFile << "ProjectLocation: person located out of MoA range. INFO: RPS point: " << meanMoA.x << ", " << meanMoA.y <<
						". 3D MoA point: " << cMoA.x << ", " << cMoA.y << ". Range: " << sqrtf(pow(prs->mean3D.X,2) + pow(prs->mean3D.Z,2)) <<". MAX_RANGE: " << 
						MAX_RANGE << ". MAX_Z_TRANS: " << ActivityMap_Utils::MAX_Z_TRANS << ". CEILING_THRESH: " << MODEL_MAX_HEIGHT << 
						". FLOOR_THRESH: " << MODEL_MIN_HEIGHT << endl;
				}
				return; //TODO: ERROR LOG
			}
			prs->stateMoA.at<float>(0, 0) = meanMoA.x;
			prs->stateMoA.at<float>(1, 0) = meanMoA.y;

			//First option (using jacobian matrices)
			//projectCovariance_Jacob(prs);

			//Second option (projecting the variances)
			projectCovariance_Var(prs);

			//To keep detection values
			prs->meanDtction = meanMoA;
		}

		//TODO: REVISE THIS METHOD
		static bool belong(const Mat& meanRPS, const Mat& sigRPS_inv, int x, int y)
		{
			clock_t start1 = clock();
			float pVals[] = {x,y};
			Mat pMx = Mat(2,1, CV_32F, pVals);
			Mat v = pMx-meanRPS;
			totalSubSubIntevalsAPP[IDENTIFY_PREP2_ID] += clock() - start1;

			//printValuesF(&pMx, "Point", cout);
			//printValuesF(&meanRPS, "Mean RPS", cout);
			//printValuesF(&sigRPS, "Sigma RPS", cout);
			//printValuesF(&v, "Point-Mean", cout);

			start1 = clock();
			Mat d = v.t()*sigRPS_inv*v;
			totalSubSubIntevalsAPP[IDENTIFY_MULT_ID] += clock() - start1;
			//printValuesF(&d, "CriticalValue", cout);
			start1 = clock();
			float val = d.at<float>(0,0);
			totalSubSubIntevalsAPP[IDENTIFY_GETVAL_ID] += clock() - start1;
			return val < 13.82; //2.5% . If the value is bigger than 5.991 then the point has less than 5% of belonging to the person
		}

		//static bool belong(const Person* p, int x, int y)
		//{
		//	float pVals[] = {x,y};
		//	Mat pMx = Mat(2,1, CV_32F, pVals);

		//	float pMeanVals[] = {p->mean_RPS.x, p->mean_RPS.y};
		//	Mat mMx = Mat(2,1, CV_32F, pMeanVals);

		//	float sigmaVals[] = {p->sigmaX_RPS, 0, p->sigmaY_RPS, 0};
		//	Mat SMx = Mat(2,2, CV_32F, sigmaVals);

		//	Mat d = ((pMx*mMx).t()*SMx.inv()*(pMx*mMx));
		//	return d.at<float>(0,0) < 5.991; //5% . If the value is bigger than 5.991 then the point has less than 5% of belonging to the person


		//	//float val = powf((x-p->mean_RPS.x),2)/powf(p->sigmaX_RPS,2) + powf((y - p->mean_RPS.y),2)/powf(p->sigmaY_RPS,2);
		//	//return val <= 1;
		//}


		//Mapp all points in the blob b (rps) to the MoA and the normal distribution parameters are estimated (mu, covariance)
		static void mapPointsMoA(vector<Point2d>& b, Person* prs, const Mat& rpsWeights, int debug, int frames, int debugFrame)
		{
			//test
		/*	Point2d pp(106, 44);
			Point ppp;
			ppp = convertBack(&pp);
			XnPoint3D p3D_;
			p3D_.X = ppp.x; p3D_.Y = MODEL_MAX_HEIGHT - 1; p3D_.Z = ppp.y;
			Point pMoA2D_ = ActivityMap_Utils::findMoACoordinate(&p3D_, MAX_RANGE, MODEL_MAX_HEIGHT, MODEL_MIN_HEIGHT);*/

			double x, y, xx, yy, xy, ww;
			x = y = xx = yy = xy = ww = 0;

			uchar* rpsWeights_data = (uchar*)rpsWeights.data;
			int rpsWeights_step = rpsWeights.step/sizeof(uchar);
			int rpsWeights_cols = rpsWeights.cols;

			int maxP = b.size();
			for (int i = 0; i < maxP; i++)
			{
				Point2d p = b[i];
							
				Point pMoA;
				pMoA = convertBack(&p);
				XnPoint3D p3D;
				p3D.X = pMoA.x; p3D.Y = MODEL_MAX_HEIGHT - 1; p3D.Z = pMoA.y;
				Point pMoA2D = ActivityMap_Utils::findMoACoordinate(&p3D, MAX_RANGE, MODEL_MAX_HEIGHT, MODEL_MIN_HEIGHT);
				if (pMoA2D.x != -1)
				{
					//print the point
	//				if (frames == 247)
	//				{
	//					outMoaPnts << pMoA2D.x << " " << pMoA2D.y << endl;
	//					outRPSPnts << p.x << " " << p.y << endl;
	//				}
					prs->moAPoints.push_back(pMoA2D);
					int w = (rpsWeights_data + (int)p.y*rpsWeights_step)[(int)p.x];

					x += (pMoA2D.x*w);
					y += (pMoA2D.y*w);
					ww += w;
					xx += pMoA2D.x*pMoA2D.x*w;
					yy += pMoA2D.y*pMoA2D.y*w; 
					xy += pMoA2D.x*pMoA2D.y*w;
				}
				else
				{
					cout << "stop" << endl;
				}
			}
			double xMean = x/ww;
			double yMean = y/ww;
			prs->meanMoA2 = Point2d(x/ww, y/ww);
			double vx = xx/ww - (xMean*xMean);
			double vy = yy/ww - (yMean*yMean);
			double vxy = xy/ww - (xMean*yMean);

			prs->covMoA2.at<float>(0,0) = vx; 
			prs->covMoA2.at<float>(0,1) = vxy;
			prs->covMoA2.at<float>(1,0) = vxy;
			prs->covMoA2.at<float>(1,1) = vy;

		}

		static void buildAppearanceModel(vector<Point2d>& b, Person* prs, vector<PointMapping>* pntsMap2, int debug, int debugFrame, int frames)
		{
			int ttlRowsApp = prs->apperance.rows;
			int ttlColsApp = prs->apperance.cols - 50;

			double x, y, xx, yy, ww;
			x = y = xx = yy = ww = 0;
			int maxP = b.size();
			for (int i = 0; i < maxP; i++)
			{
				Point2d p = b[i];
				int step = p.y*RANGE_COLS+p.x;
				vector<PointMapping>* pnts = &(pntsMap2[step]);
				int maxPs = pnts->size();
				for (int j = 0; j < maxPs; j++)
				{
					PointMapping* pMap = &((*pnts)[j]);
					
					//PointMapping* pMap = &((*pnts)[j]);
					//(*pnts)[j].idPerson = prs->idDetection;
					pMap->idPerson = prs->idDetection;
					//pMap->idPerson = prs->idDetection;
					if (pMap->p3D->Y > prs->maxHeight)
						prs->maxHeight = pMap->p3D->Y;
					if (pMap->p3D->Y < prs->minHeight)
						prs->minHeight = pMap->p3D->Y;

					//update the appearance model
					float height = pMap->p3D->Y;
					const XnRGB24Pixel* colour = pMap->colour;
					if (debug > DEBUG_NONE && frames == debugFrame)
						outHeights << height << endl;
					//If the point is whithin the height range of a person
					if (height < MODEL_MAX_HEIGHT && height > MODEL_MIN_HEIGHT)
					{
						int bin = (MODEL_MAX_HEIGHT - height)/MODEL_BINRANGE;
						
						//Fill the debug image of the apperance						
						int rowPos = prs->countApperance[bin]/ttlColsApp;
						int colPos = (prs->countApperance[bin]%ttlColsApp + 50);
						prs->apperance.ptr<uchar>(bin*50+rowPos)[colPos*3] = (uchar)colour->nBlue;
						prs->apperance.ptr<uchar>(bin*50+rowPos)[colPos*3+1] = (uchar)colour->nGreen;
						prs->apperance.ptr<uchar>(bin*50+rowPos)[colPos*3+2] = (uchar)colour->nRed;
						prs->countApperance[bin]++;

						gaussianParam* modelBinColour = prs->colourModel + bin;

						//if (modelBinColour->totalColours == MAX_POINTS_BIN)
						//{
						//	outDebugFile << "BuildAppearanceModel:ERROR Reached maximum number of points per bin. Info: BinId: " << bin <<" TotalPoints: " << modelBinColour-> totalColours << 
						//		"Max_Points_bin: " << MAX_POINTS_BIN << endl;
						//}
						//else
						{
							//modelBinColour->colours[modelBinColour->totalColours++] = colour;

							int numBinPnts = prs->heightModel[bin];
								
							//update the model (height, colour)
							if (numBinPnts > 0)	
							{
								(*modelBinColour).mean(0) = (colour->nRed + numBinPnts * (*modelBinColour).mean(0))/(numBinPnts+1);
								(*modelBinColour).mean(1) = (colour->nGreen + numBinPnts * (*modelBinColour).mean(1))/(numBinPnts+1);
								(*modelBinColour).mean(2) = (colour->nBlue + numBinPnts * (*modelBinColour).mean(2))/(numBinPnts+1);
							}
							else
								prs->colourModel[bin].mean = Scalar(colour->nRed, colour->nGreen, colour->nBlue);

							//for the covariance
							(*modelBinColour).rr += powf(colour->nRed,2);
							(*modelBinColour).gg += powf(colour->nGreen,2);
							(*modelBinColour).bb += powf(colour->nBlue,2);
							(*modelBinColour).rg += colour->nRed * colour->nGreen;
							(*modelBinColour).rb += colour->nRed * colour->nBlue;
							(*modelBinColour).gb += colour->nGreen * colour->nBlue;

							prs->heightModel[bin]++;
							if (debug >= DEBUG_MED && frames == 96 && prs->id == 1)
							{
								outPersModel << height << " " << (int)colour->nRed << " " << (int)colour->nGreen << " " << (int)colour->nBlue << endl;
							}
						}

					}
					else
					{
						outDebugFile << "DetectCC: Person point out of height range: Info: RPS coordinates: " << pMap->rpX << ", " << pMap->rpY <<
							". Height: " << height << ". MAX_HEIGHT: " << MODEL_MAX_HEIGHT << ". MIN_HEIGHT: " << MODEL_MIN_HEIGHT << ". Colour: " 
							<< (int)colour->nRed << ", " << (int)colour->nGreen << ", " << (int)colour->nBlue << endl;
					}
				}
			}
			for (int j = 0; j < MODEL_NBINS; j++)
			{
				int numBinPnts = prs->heightModel[j];
				if (numBinPnts > 0)
				{
					gaussianParam* mbc = prs->colourModel + j;

					Mat* cov = &mbc->cov;
					cov->at<float>(0,0) = mbc->rr/numBinPnts - powf(mbc->mean(0),2);
					cov->at<float>(0,1) = mbc->rg/numBinPnts - (mbc->mean(0)*mbc->mean(1));
					cov->at<float>(0,2) = mbc->rb/numBinPnts - (mbc->mean(0)*mbc->mean(2));
					cov->at<float>(1,0) = mbc->rg/numBinPnts - (mbc->mean(0)*mbc->mean(1));
					cov->at<float>(1,1) = mbc->gg/numBinPnts - powf(mbc->mean(1),2);
					cov->at<float>(1,2) = mbc->gb/numBinPnts - (mbc->mean(1)*mbc->mean(2));
					cov->at<float>(2,0) = mbc->rb/numBinPnts - (mbc->mean(0)*mbc->mean(2));
					cov->at<float>(2,1) = mbc->gb/numBinPnts - (mbc->mean(1)*mbc->mean(2));
					cov->at<float>(2,2) = mbc->bb/numBinPnts - powf(mbc->mean(2),2);		

					//fill the mean colour in the apperance image
					int red = mbc->mean.val[0];
					int green = mbc->mean.val[1];
					int blue = mbc->mean.val[2];
					int step = j*50;
					for (int y = 0; y < 50; y++)
					{
						uchar* ptr = prs->apperance.ptr<uchar>(y+step);
						for (int x = 0; x < 50; x++)
						{
							ptr[x*3] = blue;
							ptr[x*3+1] = green;
							ptr[x*3+2] = red;
						}
					}
				}
			}
		}


		//Detect people blobs using an hysteresis threshold and a component labelling (on the RMPSpace)
		static void detectCC(Mat& bw, Person* dtctPpl, int& ttl_dtctPpl, vector<PointMapping>* pntsMap2, int ttlPnts, Mat& debugImg, int debug, int frames, int debugFrame)
		{
			Mat imgCpy = Mat(bw.size(), CV_8UC1);
			Mat cpy = Mat(bw.size(), CV_8UC1);
			Utils::convert16to8(&bw, imgCpy); //convert the image to a range between 0-255. And also inverts the values, so 0 belongs to high values in viceversa

			//Create a binary image
			threshold(imgCpy, cpy, 254, 1, THRESH_BINARY); //backround = 1, foreground = 0;

			//find connected components
			clock_t startTime = clock();
			std::vector < std::vector<cv::Point2d > > blobs;
			findBlobs(cpy, blobs);
			totalSubIntervalsDetection[BLOBS_ID] += clock() - startTime; //time debugging
			//for each region calculate mean and covariance
			vector<vector<Point2d>>::iterator iter = blobs.begin();
			double x, y, xx, yy, ww;
			float range,alpha, rr, aa;


			uchar* bwcpy_data = (uchar*)imgCpy.data;
			int bwcpy_step = imgCpy.step/sizeof(uchar);
			int bwcpy_cols = imgCpy.cols;

			ushort* bw_data =(ushort*) bw.data;
			int bw_step = bw.step/sizeof(ushort);
			int bw_cols = bw.cols;

			int idPers = 0;
			while (iter != blobs.end())
			{
				bool pass = false;
				vector<Point2d> b = *iter;
				//if (b.size() > 10) //rejection of small regions
				{
					x = y = xx = yy = ww = 0;
					int maxI = b.size();
					for (int i = 0; i < maxI; i++)
					{
						Point2d p = b[i];
					
						float rangeTmp = (bw.rows-p.y)*RANGE_STEP; //binSize
						float thresh;
						if (rangeTmp > 4333)
							thresh = -0.46*rangeTmp+5500;
						else if (rangeTmp <= 4333 && rangeTmp > 3750)
							thresh = -2*rangeTmp + 14000;
						else
							thresh = 0.6*rangeTmp+5500;
	
						int w = (bwcpy_data + (int)p.y*bwcpy_step)[(int)p.x];
						int w1 = (bw_data + (int)p.y*bw_step)[(int)p.x];
						if (w1 > thresh) //check that there is at least one pixel that get the higher threshold
							pass = true;


						x += (p.x*w);
						y += (p.y*w);
						ww += w;
						xx += p.x*p.x*w;
						yy += p.y*p.y*w; 

						//Create the appearance model here
					}
					if (pass)
					{	
						Person* prs = &dtctPpl[ttl_dtctPpl++];
						initPerson(prs, debug);
						prs->mean_RPS = Point2d(x/ww, y/ww);
						prs->id = idPers++;
						prs->idDetection = prs->id;

						double mRpSX = x/ww;
						double mRpSY = y/ww;

						double vx1 = xx/ww - (mRpSX*mRpSX);
						double vy1 = yy/ww - (mRpSY*mRpSY);

						float vx = xx/ww - powf(prs->mean_RPS.x,2);
						float vy = yy/ww - powf(prs->mean_RPS.y,2);
						if (vx > 0 && vy > 0)
						{
							prs->sigmaX_RPS = sqrt(vx1);
							prs->sigmaY_RPS = sqrt(vy1);
							double covVals[] = {vx1, 0, 0, vy1};
							prs->sigmaRPS_inv = Mat(2,2, CV_32F, covVals);
						}
						else
						{
							outDebugFile << "detectionCC: Error sigma. Negative value inside square root" << endl;
						
							xx = yy = 0;
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
								int w = (bwcpy_data + rpY*bwcpy_step)[rpX];	
								outDebugFile << rpX << " " << rpY << " " << w  << ";"<< endl;
							
								xx += w*powf(rpX-prs->mean_RPS.x,2);
								yy += w*powf(rpY-prs->mean_RPS.y,2);
							}
							prs->sigmaY_RPS = sqrtf(yy/ww);
							prs->sigmaX_RPS = sqrtf(xx/ww);
							outDebugFile << "end " << endl;
						}

						//Todo build appearance model
						startTime = clock();
						//buildAppearanceModel(b, prs, pntsMap2, debug, debugFrame, frames);
						totalSubIntervalsDetection[BUILDAPPEARANCE_ID] += clock() - startTime; //time debugging

						startTime = clock();
						projectLocation2MoA(prs, debug, frames, debugFrame);
						totalSubIntervalsDetection[PROJECT2MOA_ID] += clock() - startTime;

						//debug to get the location in the MoA by mapping all the points
						if (debug >= DEBUG_MED)//&& prs->id == 0)
						{
							mapPointsMoA(b, prs, imgCpy, debug, frames, debugFrame);
							/*outDebugFile << "Distribution parameters (mapped)" << endl;
							printValuesF(&prs->stateMoA, "Mean MoA", outDebugFile);
							printValuesF(&prs->stateUncertainty, "Cov MoA" , outDebugFile);
							outDebugFile << "Distribution parameters (points mapping) " << endl;
							outDebugFile << "Mean: " << prs->meanMoA2.x << ", " << prs->meanMoA2.y << endl;
							printValuesF(&prs->covMoA2, "Cov Moa 2 " , outDebugFile);
							outDebugFile << "Distribution parameters (RPSpace)" << endl;
							outDebugFile << "Mean RPS: " << prs->mean_RPS.x << " " << prs->mean_RPS.y << endl;
							outDebugFile << "Sigma (X,Y): " << prs->sigmaX_RPS << " " << prs->sigmaY_RPS << endl;*/
						}
					}
				}
				iter++;
			}
			

			//create appearance model
			//startTime = clock();
			//buildAppearanceModels(dtctPpl, ttl_dtctPpl,  pntsMap,  ttlPnts, debug, frames, debugFrame);
			//totalSubIntervalsDetection[BUILDAPPEARANCE_ID] += clock() - startTime; //time debugging
		}

		/*
		img : Mat(500,181, CV_16UC1)

		*/

		static void ccDetection(Mat& img, Person* dtctPpl, int& ttl_dtctPpl, vector<PointMapping>* pntsMap2, int& ttlPnts, int debug, int frames, int debugFrame)
		{
			Mat imgCpy1, imgCpy2;
			img.copyTo(imgCpy1);

			int img_Rows = img.rows;
			int img_Cols = img.cols;
			ushort* img_data = (ushort*)img.data;
			int img_step = img.step/sizeof(ushort);

			clock_t startTime;

			startTime = clock(); //time debuggin
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
			totalSubIntervalsDetection[FIRSTTHRES_ID] += clock() - startTime; //time debugging

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
			detectCC(img, dtctPpl, ttl_dtctPpl, pntsMap2, ttlPnts, outDebug, debug, frames, debugFrame);
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
					float sgX = sqrtf(p->stateUncertainty.at<float>(0,0));
					float sgY = sqrtf(p->stateUncertainty.at<float>(1,1));
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

		static void drawGatinArea(const Mat& innUncertainty, const Point& pntMean, Mat* moa, Scalar color, char* txt)
		{
			//Mat gtMat = GATING_THRESHOLD*innUncertainty;
			SVD svd(innUncertainty);
						
			float bigAxis = GATING_THRESHOLD*sqrtf(svd.w.at<float>(0));
			float smallAxis = GATING_THRESHOLD*sqrtf(svd.w.at<float>(1));

			//identify the quadrant of the main eigenvector
			bool upperQuadrant = (svd.u.at<float>(1,0) > 0);
			Mat bigEigenVct = svd.u(Rect(0,0, 1,2));
			float vals[] = {1, 0};
			Mat mainAxis = Mat(2,1, CV_32F, vals);
			float dotPrd = bigEigenVct.dot(mainAxis);
			float angle = acosf(dotPrd)*180/CV_PI;
			if (!upperQuadrant)
					angle = -angle;

			cv::ellipse(*moa, pntMean, Size(bigAxis, smallAxis), angle, 0, 360, color, 1);		
			if (txt != NULL)
				putText(*moa, txt, Point(pntMean.x + bigAxis, pntMean.y), FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, color);

		}

		//static void drawPersonPointsCov_debug(const Perso&n p, Mat* moa, Scalar color)
		static void drawPersonPointsCov_debug(const Point& pntMean, const Mat& cov, Mat* moa, Scalar color, int thickness, char* txt)
		{
			SVD svd(cov);
	
			float bigAxis = sqrtf(svd.w.at<float>(0))*2;
			float smallAxis = sqrtf(svd.w.at<float>(1))*2;

			//identify the quadrant of the main eigenvector
			bool upperQuadrant = (svd.u.at<float>(1,0) > 0);
			Mat bigEigenVct = svd.u(Rect(0,0, 1,2));
			float vals[] = {1, 0};
			Mat mainAxis = Mat(2,1, CV_32F, vals);
			float dotPrd = bigEigenVct.dot(mainAxis);
			float angle = acosf(dotPrd)*180/CV_PI;
			if (!upperQuadrant)
				angle = -angle;

			cv::ellipse(*moa, pntMean, Size(bigAxis, smallAxis), angle, 0, 360, color, thickness);	
			if (txt != NULL)
				putText(*moa, txt, Point(pntMean.x + bigAxis, pntMean.y), FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, Scalar(0,0,255));
		}


		static void drawPersonCov_debug(const Person& p, Mat* moa, Scalar color)
		{
			Point pntMean = Point(p.stateMoA.at<float>(0,0),p.stateMoA.at<float>(1,0));
			SVD svd(p.stateUncertainty(Rect(0,0,2,2)));
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

		static bool displayMergeMeasurements(Person* trckPpl, int ttl_trckPpl, Person* dtctPpl, int ttl_dtctPpl, Mat& moa, int debug, int frames)
		{
			for (int iter = 0; iter < ttl_trckPpl; iter++)
			{
				const Person* p = &(trckPpl[iter]);

				Point meanMoA = Point(p->stateMoA.at<float>(0,0), p->stateMoA.at<float>(1,0));
				
				Scalar color = Scalar(0,0,255); //occluded blob
				
				char txt[15];
				itoa(p->id, txt, 10);
				putText(moa, txt, Point(meanMoA.x + 10, meanMoA.y + 10), FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, color);
			}

			bool out = false;		
			for (int iter = 0; iter < ttl_dtctPpl; iter++)
			{
				const Person* p = &(dtctPpl[iter]);
					
				Scalar color;
				
				if (p->control != DTC_MERGE)
					color = Scalar(255,0,0);
				else if (p->control == DTC_MERGE)
				{
					color = Scalar(0,0,255);
					out = true;
				}
				
				Point meanMoA = Point(p->stateMoA.at<float>(0,0), p->stateMoA.at<float>(1,0));
						
				float area = 4*p->sigmaX_RPS * p->sigmaY_RPS;
				char txt[15];
				itoa(area, txt, 10);
				drawGatinArea(p->covDtction, meanMoA, &moa, color, txt);
			}
			return out;

		}

		static void displayRPSDetections(Person* dtctPpl, int ttl_dtctPpl, Mat& remapPolar, int debug)
		{
						
			for (int iter = 0; iter < ttl_dtctPpl; iter++)
			{
				const Person* p = &(dtctPpl[iter]);
					
				Scalar color;
				
				if (p->control == DTC_FULL)
					color = Scalar(255,0,0);
				else if (p->control == DTC_MERGE)
					color = Scalar(0,0,255);
				
				cv::circle(remapPolar, p->mean_RPS, 2, Scalar::all(0), -1);

				float vals[] = {powf(p->sigmaX_RPS,2), 0, 0, powf(p->sigmaY_RPS,2)};
				Mat cov = Mat(2,2, CV_32F, vals);

				drawPersonPointsCov_debug(p->mean_RPS, cov, &remapPolar, Scalar::all(0), 1, NULL);
			}
		}

		static void displayDetections(Person* dtctPpl, int ttl_dtctPpl, Mat& remapPolar, Mat& moa, int debug)
		{
			for (int iter = 0; iter < ttl_dtctPpl; iter++)
			{
				const Person* p = &(dtctPpl[iter]);
					
				Scalar color;
				
				if (p->control == DTC_FULL)
					color = Scalar(255,0,0);
				else if (p->control == DTC_MERGE)
					color = Scalar(0,0,255);
				

				if (debug >= DEBUG_NONE)
				{
					cv::circle(remapPolar, p->mean_RPS, 2, Scalar::all(0), -1);
					cv::ellipse(remapPolar, p->mean_RPS, Size(p->sigmaX_RPS*2, p->sigmaY_RPS*2), 0,0,360, Scalar::all(0));
				}

				Point meanMoA = Point(p->stateMoA.at<float>(0,0), p->stateMoA.at<float>(1,0));

				drawPersonPointsCov_debug(meanMoA, p->covMoA_points, &moa, Scalar::all(0), 1, NULL);

				//Display the mapping of the rps points along with the ellipse of the distribution
				if (debug >= DEBUG_MED)
				{
					int ttl = p->moAPoints.size();
					for (int i = 0; i < ttl; i++)
					{
						Point pnt = p->moAPoints[i];
						uchar* ptr = moa.ptr<uchar>(pnt.y);
						ptr[3*pnt.x] = color.val[0];
						ptr[3*pnt.x+1] = color.val[1];
						ptr[3*pnt.x+2] = color.val[2];
					}
				}
			}
		}
		static void displayTrackersRPS(Person* trckPpl, int ttl_trckPpl, Mat& remapPolar,int debug)
		{
			for (int iter = 0; iter < ttl_trckPpl; iter++)
			{
				const Person* p = &(trckPpl[iter]);
				float vals[] = {powf(p->sigmaX_RPS,2), 0, 0, powf(p->sigmaY_RPS, 2)};
				Mat covRPS = Mat(2,2, CV_32F, vals);
				drawPersonPointsCov_debug(p->mean_RPS, covRPS, &remapPolar, Scalar(0,255,0),1, NULL);
				cv::circle(remapPolar, p->mean_RPS, 2, Scalar::all(0), -1);
			}
		}

		static void displayTrackersMoA(Person* trckPpl, int ttl_trckPpl, Mat& moa, int debug, int frames)
		{
			for (int iter = 0; iter < ttl_trckPpl; iter++)
			{
				const Person* p = &(trckPpl[iter]);

				Point meanMoA = Point(p->stateMoA.at<float>(0,0), p->stateMoA.at<float>(1,0));
				//cv::circle(moa, meanMoA, 2, Scalar(255,0,0));

				Scalar color = Scalar(255,0,0); //occluded blob
				if (p->lost > 0)
					color = Scalar(0,0,0);
				
				if (p->control != DTC_MERGE)
					color = p->colour;
				else 
					color = Scalar(0,0,255);

					//To display the ellipse of the detection in comparison with the updated tracked ellipse
					if (debug >= DEBUG_MED)
					{
//						if (p->id == 1 && frames == 80)
//							printValuesF(&p->stateUncertainty, "State uncertainty before drawing ellipse", outDebugFile);
						//Associated measurement ellipse
						//if (p->associated)
						//	drawPersonPointsCov_debug(p->meanDtction, p->covDtction, &moa, Scalar(0,0,255), -1);

						//uncertainty
						Scalar c = Scalar(0,0,255);
						if (!p->associated)
							c = Scalar(0,0,0);
						
						//float range = sqrtf(pow(p->mean3D.X,2) + pow(p->mean3D.Z,2));
						//float scale = ((0.33*log(10+3*range/1000))-0.80597) + 2;
						//printValuesF(&p->innUncertainty, "Innovatin uncertainty", cout);
						drawGatinArea(p->innUncertainty, p->mean_noupdate, &moa, Scalar(255,0,0),NULL);

						//drawPersonPointsCov_debug(p->mean_noupdate, p->stateUn_noupdate, &moa, Scalar(250, 0,0), 1);
						//drawPersonPointsCov_debug(meanMoA, p->stateUncertainty, &moa, c, 1);

						//The error measurement
						//printValuesF(&p->rotation, "Rotation matrix III", outDebugFile);
						//printValuesF(&p->R, "Error Measurement", outDebugFile);
						//drawPersonPointsCov_debug(meanMoA, p->R, &moa, Scalar(20,20,20), 1);

						//if (p->id == 0)
						//	printValuesF(&p->stateUncertainty, "Uncertainty of the state(updated)", outDebugFile);

						//The euclidean validated region
						//Point p1 = Point(meanMoA.x-p->euclThresh, meanMoA.y - p->euclThresh);
						//Point p2 = Point(meanMoA.x + p->euclThresh, meanMoA.y + p->euclThresh);
						//rectangle(moa, p1, p2, Scalar::all(0));
					
					}

					//Tracking ellipse
					char txt[15];
					itoa(p->id, txt, 10);
					//putText(moa, txt, meanMoA,FONT_HERSHEY_PLAIN, 0.8, Scalar(0,0,255));

					drawPersonPointsCov_debug(meanMoA, p->covMoA_points, &moa, color, 2, txt);
					
					if (debug >= DEBUG_MED)
					{
						//drawPersonPointsCov_debug(p->meanMoA2, p->covMoA2, &moa, Scalar::all(0));
						//Display the mapping of the rps points along with the ellipse of the distribution
						int ttl = p->moAPoints.size();
						for (int i = 0; i < ttl; i++)
						{
							Point pnt = p->moAPoints[i];
							uchar* ptr = moa.ptr<uchar>(pnt.y);
							ptr[3*pnt.x] = color.val[0];
							ptr[3*pnt.x+1] = color.val[1];
							ptr[3*pnt.x+2] = color.val[2];
						}
					}
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
		
				//float sgX = sqrtf(prs->stateUncertainty.at<float>(0,0));
				//float sgY = sqrtf(prs->stateUncertainty.at<float>(1,1));
				//float area = sgX*sgY;
				float area = 4*prs->sigmaX_RPS * prs->sigmaY_RPS;

				//outDebugFile << area << " " << prs->mean_RPS.y << endl;
				//float thresh = 330 - 0.694*prs->mean_RPS.y;
				if (area >= 122) // joint blob
				{
					prs->control = DTC_MERGE;
					prs->numAsso = 0;
				}
				//if (area < 50)
				//	prs->control = DTC_SPLIT;
				//else if (area > 800)
				//	prs->control = DTC_MERGE;
				//else
				//	prs->control = DTC_FULL;
			}
		}

		/*
		Predict the state and covariance of the position 
		*/
		static void predictState(Person*& p, int debug, int frames)
		{

			p->stateMoA = p->A * p->stateMoA;

			//CovMoA is the covariance projected from the detection
			if (p->associated) //from last frame. It does not increase its uncertainty if there were not measurement
				p->stateUncertainty = p->Q + p->A * p->stateUncertainty * p->A.t();
			
			Mat innUncertainty = H * p->stateUncertainty * H.t() + p->R;
			innUncertainty.copyTo(p->innUncertainty);

			//updates the measurement error (range dependent) and the euclidean gatting area
			setTimeVariantParameters(p, debug, frames);
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
		static float mahalanobis(const Mat* tgtMean, const Mat* uncertInv, const Person* dPrs)
		{
			//Mean diff
			Mat dtcMean = dPrs->stateMoA(Rect(0,0,1,2));
			Mat meanDiff = dtcMean-(*tgtMean);

			//sum covariances
			Mat out = meanDiff.t() * (*uncertInv) * meanDiff;

			float val = out.at<float>(0,0);

			return val;

		}

		/*
		It uses a first gating area to select the measurment closest to the target. Then chooses the measurement with the minimum
		Mahalanobis distance
		*/
		static void gateDetectionNNeigh(Person* target,  Person* dtctPpl, int ttl_dtctPpl, Person*& candidate, int debug, int frames)
		{
			target->innUncertainty = H*target->stateUncertainty*H.t() + target->R;

			Mat tgtMean = H*target->stateMoA;
			target->mean_noupdate.x = tgtMean.at<float>(0);
			target->mean_noupdate.y = tgtMean.at<float>(1);
		
			Mat innUncertInv = target->innUncertainty.inv();
			float maxDist = 50;
			int numValidatedMeas = 0;
			int pos = -1;
			for (int i = 0; i < ttl_dtctPpl; i++)
			{
				Person* dPrs = &dtctPpl[i];
				if (!dPrs->associated)
				{	
					float c = mahalanobis(&tgtMean, &innUncertInv, dPrs);
					if (c < GATING_THRESHOLD && c < maxDist) //it consider only those measurement that fall within the gating area (less than 10.597)
					{
						maxDist = c;
						candidate = dPrs;
						numValidatedMeas++;
						pos = i;
					}
					if (debug == DEBUG_MED)
					{
						outDebugFile << "Mahalanobis (tgt id: " << target->id << ". dtct id:  " << dPrs->id << "): " << c << endl;
					}
				}
			}
			//update the number of associations for this measurement if it is merge
			if (debug >= DEBUG_NONE)
				if (pos > -1 && dtctPpl[pos].control == DTC_MERGE)
					dtctPpl[pos].numAsso++;
			
			outDataAss << target->id << " " << frames << " " << numValidatedMeas << endl;
		}

		static void gateDetection(const Person* target,  Person* dtctPpl, int ttl_dtctPpl, Person** gPpl, int& ttl_gPpl, int debug)
		{
			Mat uncert = target->stateUncertainty(Rect(0,0,2,2));
			uncert.copyTo(target->stateUn_noupdate);
			Mat uncertInv = uncert.inv();
			Mat tgtMean = target->stateMoA(Rect(0,0, 1,2));
			for (int i = 0; i < ttl_dtctPpl; i++)
			{
				Person* dPrs = &dtctPpl[i];
				if (!dPrs->associated)
				{
					float eucDist = sqrtf(powf(target->stateMoA.at<float>(0,0) - dPrs->meanDtction.x, 2) + powf(target->stateMoA.at<float>(1,0) - dPrs->meanDtction.y, 2));
					if (eucDist < target->euclThresh)
					{
						float c = mahalanobis(&tgtMean, &uncertInv, dPrs);
						if (debug == DEBUG_MED)
						{
							outDebugFile << "Mahalanobis (tgt id: " << target->id << ". dtct id:  " << dPrs->id << "): " << c << endl;
						}
						//2.5% . If the value is bigger than 5.991 then the point has less than 5% of belonging to the person
						cout << "Mahalanobis distance: " << c << endl;
						if (c < 13.82)
						{
							gPpl[ttl_gPpl] = dPrs;
							ttl_gPpl++;
						}
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
		Search from the available measurment the spatial closest.
		1- Define a euclidean threshold
		2- Define a gating area based on the Mahalanobis distance and the Chi square tables.
		It selects the one with the minimum Mahalanobis distance from the ones that are inside the 
		euclidean area.

		IN:
		Target, dtectPpl, ttl_dtctPpl

		OUT:
		candidate, compCoeff (it will be used later to decide if the appearance update is performed or not.
		*/
		static void associationNNeigh(Person* target,  Person* dtctPpl, int ttl_dtctPpl, Person*& candidate, int debug, int frames)
		{
			//Filter all detections within a gate region
			gateDetectionNNeigh(target, dtctPpl, ttl_dtctPpl, candidate, debug, frames);
		}	


		/*
		Update the target state (x,y,vx,vy) from the measurement state through the kalman equations 
		*/
		static void updateState(Person* trgt, Person* msr, float dComp, int debug, int frames)
		{
			//Biggest R entails smallest K and viceversa
			Mat K = trgt->stateUncertainty * H.t() * trgt->innUncertainty.inv();
			K.copyTo(trgt->K);

			//2x1 location of measurement
			Mat msreState = msr->stateMoA(Rect(0,0,1,2));
			//2x1 location of the target
			Mat trgtState = trgt->stateMoA(Rect(0,0,1,2));
			//UPDATE EQUATION
			trgt->stateMoA = trgt->stateMoA + trgt->K*(msreState - H*trgt->stateMoA);
	
			trgt->lost = 0;
			trgt->associated = true;
			//Update the location in the remap polar space
			trgt->mean_RPS = msr->mean_RPS;
			trgt->sigmaX_RPS = msr->sigmaX_RPS;
			trgt->sigmaY_RPS = msr->sigmaY_RPS;
			trgt->idDetection = msr->id;
			msr->apperance.copyTo(trgt->apperance);
			msr->covMoA_points.copyTo(trgt->covMoA_points);
			//For the representation of the ellipse.
			msr->covMoA2.copyTo(trgt->covMoA2);
			trgt->meanMoA2 = msr->meanMoA2;
			trgt->moAPoints = msr->moAPoints;
			trgt->meanDtction = msr->meanDtction;
			msr->covDtction.copyTo(trgt->covDtction);
			trgt->mean3D = msr->mean3D;
			trgt->control = msr->control;
			msr->rotation.copyTo(trgt->rotation);
			msr->associated = true;
			

			//update the covariance 
			Mat I = Mat::eye(4,4,CV_32F);
			trgt->stateUncertainty = (I - (trgt->K*H))*trgt->stateUncertainty;

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
				if (!dtc->associated && dtc->control != DTC_MERGE)
				{
					int red = rand() % 255 + 1;
					int green = rand() % 255 + 1;
					int blue = rand() % 255 + 1;
					dtc->colour = Scalar(red,green, blue);
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


		static void checkDCFs(Person*& dtctPpl, int& ttl_dtctPpl, vector<DCF>& DCFs, int frames)
		{
			for (int i = 0; i < ttl_dtctPpl; i++)
			{
				Person p = dtctPpl[i];
				if (p.control == DTC_MERGE && p.associatedIds.size() > 0)
				{
					DCF dcf;
					dcf.idFrame = frames;
					dcf.trgtIds = p.associatedIds;
					EllipseParam ep;
					dcf.avgEllipse.mean.x = p.stateMoA.at<float>(0,0);
					dcf.avgEllipse.mean.y = p.stateMoA.at<float>(1,0);
					dcf.avgEllipse.covX = p.covDtction.at<float>(0,0);
					dcf.avgEllipse.covY = p.covDtction.at<float>(1,1);
					dcf.avgEllipse.covXY = p.covDtction.at<float>(1,0);
					DCFs.push_back(dcf);					
				}
			}
		}

		static void writeXMLDCFs(const vector<DCF>& DCFs)
		{
			//Add xml header <DCFs>
			TiXmlDocument doc; 
			TiXmlElement * root = new TiXmlElement( "DCFs" );  
			doc.LinkEndChild( root );  
			int idDCF = 0;
			for (int frId = 0; frId < 1000; frId++)
			{
				//Add to xml File
				TiXmlElement * dcfXml = new TiXmlElement( "DCF" );  
				dcfXml->SetAttribute("frameId", frId);
				root->LinkEndChild( dcfXml );  
	
				int cont = 0;
				while (idDCF < DCFs.size() && DCFs[idDCF].idFrame == frId)
				{
					DCF dcf = DCFs[idDCF];
					//Add to xml File
					TiXmlElement * occXml = new TiXmlElement( "Occlusion" );  
					occXml->SetAttribute("id", cont++);			
					for (int i = 0; i < dcf.trgtIds.size(); i++)
					{
						TiXmlElement * tgtXml = new TiXmlElement( "Target" ); 
						tgtXml->SetAttribute("Id", dcf.trgtIds[i]);
						occXml->LinkEndChild(tgtXml);
					}
					dcfXml->LinkEndChild(occXml);
					idDCF++;
				}
			}
			doc.SaveFile( out_dcfs_system );  
		}

		static void writeXMLMerges(list<Merge>& mergeList)
		{
			//Add xml header <DCFs>
			TiXmlDocument doc; 
			TiXmlElement * root = new TiXmlElement( "Merges" );  
			doc.LinkEndChild( root );

			list<Merge>::iterator iter = mergeList.begin();
			while (iter != mergeList.end())
			{
				Merge m = *iter;
				TiXmlElement * mergeXml = new TiXmlElement( "Merge" ); 
				root->LinkEndChild( mergeXml );
				mergeXml->SetAttribute("Id", m.id);
				mergeXml->SetAttribute("InitFrame", m.initFrame);
				mergeXml->SetAttribute("EndFrame", m.endFrame);
				int ttlTgts = m.tgtIds.size();
				for (int i = 0; i < ttlTgts; i++)
				{
					TiXmlElement * tgtXML = new TiXmlElement( "Target" ); 
					int tgtId = m.tgtIds[i];
					tgtXML->SetAttribute("Id", tgtId);
					mergeXml->LinkEndChild(tgtXML);
				}

				TiXmlElement * framesXML = new TiXmlElement( "Frames" ); 
				for (int i = m.initFrame; i <= m.endFrame; i++)
				{
					TiXmlElement * frameXML = new TiXmlElement( "Frame" ); 
					EllipseParam ep = m.gd_Prms[i-m.initFrame];
					frameXML->SetAttribute("Number", i);
					frameXML->SetAttribute("meanX", ep.mean.x);
					frameXML->SetAttribute("meanY", ep.mean.y);
					frameXML->SetDoubleAttribute("covX", ep.covX);
					frameXML->SetDoubleAttribute("covY", ep.covY);
					frameXML->SetDoubleAttribute("covXY", ep.covXY);
					framesXML->LinkEndChild(frameXML);
				}
				mergeXml->LinkEndChild(framesXML);
				iter++;
		
			}
			doc.SaveFile( out_merge_system );  
		}

		static bool isOnCurrenList(list<Merge>& currentList, Merge* mOld)
		{
			Merge* m = NULL;

			bool out = false;
			list<Merge>::iterator iter = currentList.begin();
			while ( iter != currentList.end() && !out)
			{
				Merge &mC( *iter );
				if ((mC.tgtIds == mOld->tgtIds))
				{
					mC.initFrame = mOld->initFrame;
					mC.id = mOld->id;
					if (mC.gd_Prms.size() != 1)
						cout << "Error" << endl;
					EllipseParam ep = mC.gd_Prms[0];
					mOld->gd_Prms.push_back(ep);
					mC.gd_Prms = mOld->gd_Prms;
					out =  true;
				}

				iter++;
			}
			return out;
		}

		static int getId(list<Merge>& mergeList)
		{
			int idMax = -1;
			list<Merge>::iterator iter = mergeList.begin();
			while (iter != mergeList.end())
			{
				Merge m = *iter;
				if (m.id > idMax)
					idMax = m.id;

				iter++;
			}
			return idMax + 1;
		}

		//Assumes DCFs are ordered according to the frame ides
		static void detectMerges(vector<DCF>& DCFs, list<Merge>& mergeList)
		{

			list<Merge> oldList;
			int idDCF = 0;
	
			for (int frId = 0; frId < 1000; frId++)
			{
				list<Merge> currentList;
				int mergeIds = mergeList.size() + oldList.size();
				if (frId == 910)
					cout << "Stop" << endl;

				while (idDCF < DCFs.size() && DCFs[idDCF].idFrame == frId)
				{
					DCF dcf = DCFs[idDCF++];
					Merge m;
					m.id = mergeIds++;
					m.initFrame = frId;
					m.tgtIds = dcf.trgtIds;
					m.gd_Prms.push_back(dcf.avgEllipse);
					currentList.push_front(m);
				}
				list<Merge>::iterator iterOld = oldList.begin();
				while (iterOld != oldList.end())
				{
					Merge mOld = *iterOld;
					if (!isOnCurrenList(currentList, &mOld))
					{
						mOld.endFrame = frId-1;
						mOld.id = getId(mergeList);
						mergeList.push_back(mOld);
						iterOld = oldList.erase(iterOld);
				
					}
					else
					{
						iterOld++;
					}
				}
		
				oldList = currentList;
			}

			//Sanity check
			list<Merge>::iterator iter = mergeList.begin();
			while (iter != mergeList.end())
			{
				Merge m = *iter;
				int ttlEllipses = m.gd_Prms.size();
				int ttlFrames = (m.endFrame - m.initFrame) + 1;
				if (ttlEllipses != ttlFrames)
					cout << "Error" << endl;

				iter++;
			}

		}

		static void look4Merges(Person*& trckPpl, int& ttl_trckPpl, Person*& dtctPpl, int& ttl_dtctPpl)
		{
			//A merge detection is set if two or more targets fall within the gating area
			for (int i = 0; i < ttl_dtctPpl; i++)
			{
				Person* dtc = &(dtctPpl[i]);
				Mat dtctMean = H*dtc->stateMoA;
				int n = 0;
				for (int j = 0; j < ttl_trckPpl; j++)
				{
					Person tgt = trckPpl[j];
					const Mat covInv = dtc->covDtction.inv();
					float c = mahalanobis(&dtctMean, &covInv, &tgt);
					if (c < GATING_THRESHOLD)
						n++;
				}

				if (n > 1)
				{
					float area = 4*dtc->sigmaX_RPS * dtc->sigmaY_RPS;
					if (area >= AREA_THRESHOLD) // joint blob
					{
						dtc->control = DTC_MERGE;
						dtc->numAsso = 0;
					}
				}

			}

		}

		static void tracking(Person*& trckPpl, int& ttl_trckPpl, Person*& dtctPpl, int& ttl_dtctPpl, Mat* moa, vector<DCF>& DCFs, int debug, int frames, ofstream& outDtcAreas)
		{	
			//Predict the state of all targets
			for (int i = 0; i < ttl_trckPpl; i++)
			{
				Person* target = &trckPpl[i];

				//Predict the state of the target using the motion model.
				predictState(target, debug, frames);
			}
			
			//Based on the area covered by the person Version 1:
			//look4MergeSplits(dtctPpl, ttl_dtctPpl);

			//Based on the proximity of targets and measurement area. Version 2:
			look4Merges(trckPpl, ttl_trckPpl, dtctPpl, ttl_dtctPpl);

			//Store the area of the detections in a file
			if (debug >= DEBUG_NONE)
			{
				for (int i = 0; i < ttl_dtctPpl; i++)
				{
					Person rpsDtc = dtctPpl[i];
					float area = (2*rpsDtc.sigmaX_RPS) * (2*rpsDtc.sigmaY_RPS);
												
					EllipseParam ep;
					ep.mean = rpsDtc.mean_RPS;
					ep.covX = rpsDtc.sigmaX_RPS*rpsDtc.sigmaX_RPS;
					ep.covY = rpsDtc.sigmaY_RPS*rpsDtc.sigmaY_RPS;
					ep.covXY = 0;
					EllipseParam epMoA;
					epMoA.mean = rpsDtc.meanDtction;
					epMoA.covX = rpsDtc.covDtction.at<float>(0,0);
					epMoA.covY = rpsDtc.covDtction.at<float>(1,0);
					epMoA.covXY = rpsDtc.covDtction.at<float>(1,1);
					int merge = 0;
					if (rpsDtc.control  == DTC_MERGE)
						merge = 1;

					outDtcAreas << rpsDtc.idDetection << " " << frames << " " << area << " " << ep.mean.x << " " << ep.mean.y << " "
						<< ep.covX << " " << ep.covY << " " << merge << " " << epMoA.mean.x << " " << epMoA.mean.y << " " 
						<< epMoA.covX << " " << epMoA.covY << " " << epMoA.covXY << endl;

				}
			}


			for (int i = 0; i < ttl_trckPpl; i++)
			{
				Person* target = &trckPpl[i];

				//Instead of removing the lost tracks from the list trckPpl (require the realocation of the rest of tracks)
				//It uses a flag to check if the target is lost
				if (target->lost <= TRACKLOST_THRESHOLD)
				{
					//Predict the state of the target using the motion model.
					//predictState(target, debug, frames);

					//Find the measurement generated by the target
					Person* measur = NULL;
					float compDist = -1;
					//This area of the code is use to plug-in and plug-out different data association techniques
				
					//1- Nearest neighbour (greedy approach based on the gating area)
					associationNNeigh(target, dtctPpl, ttl_dtctPpl, measur, debug, frames);

					//association(target, dtctPpl, ttl_dtctPpl, measur, compDist, debug, frames);

					if (measur != NULL && measur->control != DTC_MERGE)
					{
						//Update the position of the target using the Kalman equations.
						updateState(target, measur, compDist, debug, frames);

						if (compDist < COMP_THRESHOLD && measur->control == DTC_FULL)
							updateModel(target, measur);	

					}
					else if (measur == NULL) // not associated
					{
						target->associated = false;
						target->lost++;
						target->moAPoints.clear();
					}
					else
					{
						measur->associatedIds.push_back(target->id);
						target->associated = false;
						target->control = DTC_MERGE;
					}

				}
			}

			checkDCFs(dtctPpl, ttl_dtctPpl, DCFs, frames);
			checkEnd_NewTracks(trckPpl, ttl_trckPpl, dtctPpl, ttl_dtctPpl);

			if (debug >= DEBUG_NONE)
			{
				for (int i = 0; i < ttl_dtctPpl; i++)
				{
					Person p = dtctPpl[i];
					if (p.control == DTC_MERGE && p.numAsso > 0)
						outMerge << frames << " " << p.numAsso << endl;				
				}
			}
		}

		/*
		Update the tracks trajectory with the information from the current frames.
		Used to store in memory the trajectories of all tracks in the entire sequence
		*/
		static void generateTrackHistory(vector<TrackInfo>& tracks, Person*& trckPpl, int& ttl_trckPpl, int frames)
		{
			for (int i = 0; i < ttl_trckPpl; i++)
			{
				//Create the position at current frame
				Person p = trckPpl[i];
				Position pos;
				pos.frameId = frames;
				pos.mean = Point(p.stateMoA.at<float>(0), p.stateMoA.at<float>(1));
				/*float varX = sqrtf(p.stateUncertainty.at<float>(0,0));
				float varY = sqrtf(p.stateUncertainty.at<float>(1,1));
				pos.bbox = Rect(mean.x-varX, mean.y-varY, varX*2, varY*2);*/
				pos.covX = p.covMoA_points.at<float>(0,0);
				pos.covY = p.covMoA_points.at<float>(1,1);
				pos.covXY = p.covMoA_points.at<float>(0,1);
				//pos.bbox = Rect(mean.x-varX, mean.y-varY, varX*2, varY*2);
				

				bool found = false;
				int iter = 0;
				while (!found && iter < tracks.size())
				{		
					found = (trckPpl[i].id == tracks[iter].id);
					iter++;
				}
				if (!found)
				{
					TrackInfo trck;
					trck.id = trckPpl[i].id;
					trck.trajectory.push_back(pos);
					tracks.push_back(trck);
				}
				else
					tracks[--iter].trajectory.push_back(pos);
			}
		}

}

