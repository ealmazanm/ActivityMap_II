#pragma once
#include <Utils.h>
#include <vld.h>
#include "hungarian.h"
#include <tinystr.h>
#include <tinyxml.h>

namespace PplDtcV1
{
	//TRACKING SETUP
		static const int DA_TYPE_NN = 0;
		static const int DA_TYPE_GNN = 1;
		static const int DA_TYPE = DA_TYPE_GNN;

		static const int DA_FEAT_SPATIAL = 0;
		static const int DA_FEAT_APPEARANCE = 1;
		static const int DA_FEATURE = DA_FEAT_APPEARANCE;

		static const float UPDATE_RATE = 0;

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
		static const int MAX_PEOPLE = 100;

		static float meas_std = 1;
		static float proc_std = 30;
		static const int MAX_POINTS_BIN = 20000;

		struct Person
		{
			int id;
			Mat stateMoA;
			Mat covMoA_points;
			Point2d  mean_RPS;
			double sigmaY_RPS;
			double sigmaX_RPS;
			Mat sigmaRPS_inv;

			Scalar colour;
			Mat rotation; //matrix that rotates a covariance matrix depending on its mean position in the MoA
			XnPoint3D mean3D;

		};

		struct EllipseParam
		{
			Point mean;
			double covX;
			double covY;
			double covXY;
		};


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
		//static vector<Track> tracks;

	//Output files
		static ofstream outDebugFile("d:/Debug.txt");
		static ofstream outPProjec("c:\\Dropbox\\PhD\\Matlab\\DetectionEval\\V_MOA\\projections.txt");
		//static ofstream outPProjRPS("c:\\Dropbox\\PhD\\Matlab\\DetectionEval\\V_RPS\\projections.txt");
		
		//static ofstream outPvariancePS("c:\\Dropbox\\PhD\\Matlab\\DetectionEval\\V_RPS\\pplRangeVariance.txt");
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
			Utils::printValuesF(&p->covMoA_points, "Cov MoA", outDebugFile);
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
		static void updateActivityMap(Mat& acMoA, const XnPoint3D* p3D, const int nP)
		{
			ushort* acMoA_data = (ushort*)acMoA.data;
			int acMoA_step = acMoA.step/sizeof(ushort);

			for (int i = 0; i < nP; i++)
			{
				Point p2D = ActivityMap_Utils::findMoACoordinate(&p3D[i], MAX_RANGE, MODEL_MAX_HEIGHT, MODEL_MIN_HEIGHT);

				if (p2D.x != -1)
				{
					int y = p2D.y;
					int x = p2D.x;
					ushort* ptracMoA = acMoA_data + (y*acMoA_step);
					ptracMoA[x]++;
				}
			}
		}

		static void updateForegroundImg(Mat& forImg, XnPoint3D* forePnts, int nP)
		{
			uchar* forImg_data = (uchar*)forImg.data;

			for (int i = 0; i < nP; i++)
			{
				XnPoint3D p3D = forePnts[i];
				if (p3D.Z > 0)
				{
					int y = p3D.Y;
					int x = p3D.X;
					uchar *ptr = forImg_data + y*forImg.step;
					ptr[x] = 255;
				}
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

		static void printValuesUchar(const Mat* m, char* title, ostream& out)
		{
			out << title << endl;
			for (int i = 0; i < m->rows; i++)
			{
				const uchar* ptr = m->ptr<uchar>(i);
				for (int j = 0; j < m->cols; j++)
				{
					out << (int)ptr[j] << " ";
				}
				out << endl;
			}
			out << endl;
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

			p->stateMoA = Mat::zeros(4,1, CV_32F);
			p->covMoA_points = Mat::zeros(2,2,CV_32F);
	
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

			covCartessian.copyTo(prs->covMoA_points);
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
			covCartessian.copyTo(prs->covMoA_points);
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
		}



		//Detect people blobs using an hysteresis threshold and a component labelling (on the RMPSpace)
		static void detectCC(Mat& bw, Person* dtctPpl, int& ttl_dtctPpl, vector<PointMapping>* pntsMap2, int ttlPnts, Mat& debugImg, int debug, int frames, int debugFrame, bool isRPS)
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
					
						
						float thresh;

						if (isRPS)
						{
							float rangeTmp = (bw.rows-p.y)*RANGE_STEP; //binSize
							/*if (rangeTmp > 4333)
								thresh = -0.46*rangeTmp+5500;
							else if (rangeTmp <= 4333 && rangeTmp > 3750)
								thresh = -2*rangeTmp + 14000;
							else
								thresh = 0.6*rangeTmp+5500;*/
							if (rangeTmp > 7253)
								thresh = -0.28*rangeTmp+3400;
							else if (rangeTmp <= 7253 && rangeTmp > 3027)
								thresh = -1.7*rangeTmp + 13700;
							else
								thresh = 2*rangeTmp+2500;
						}
						else
						{
							float X = p.x*ActivityMap_Utils::X_STEP + ActivityMap_Utils::MIN_X;
							float Z = (bw.rows - p.y)*ActivityMap_Utils::Z_STEP;
							float rangeTmp = sqrtf(X*X + Z*Z);

							if (rangeTmp > 5957)
								thresh = -0.06*rangeTmp+900;
							else if (rangeTmp <= 5957 && rangeTmp > 2750)
								thresh = -rangeTmp + 6500;
							else
								thresh = -3*rangeTmp+12000;
						}
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
						if (isRPS)
							prs->mean_RPS = Point2d(x/ww, y/ww);
						else
						{
							prs->stateMoA.at<float>(0) = x/ww;
							prs->stateMoA.at<float>(1) = y/ww;
						}
						prs->id = idPers++;

						double mRpSX = x/ww;
						double mRpSY = y/ww;

						double vx1 = xx/ww - (mRpSX*mRpSX);
						double vy1 = yy/ww - (mRpSY*mRpSY);

						float vx = xx/ww - powf(prs->mean_RPS.x,2);
						float vy = yy/ww - powf(prs->mean_RPS.y,2);
						if (vx > 0 && vy > 0)
						{
							double covVals[] = {vx1, 0, 0, vy1};
							if (isRPS)
							{
								prs->sigmaX_RPS = sqrt(vx1);
								prs->sigmaY_RPS = sqrt(vy1);
								prs->sigmaRPS_inv = Mat(2,2, CV_32F, covVals);
							}
							else
							{
								prs->covMoA_points = Mat(2,2, CV_32F, covVals);
							}
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

						if (isRPS)
						{
							startTime = clock();
							projectLocation2MoA(prs, debug, frames, debugFrame);
							totalSubIntervalsDetection[PROJECT2MOA_ID] += clock() - startTime;
						}
					}
				}
				iter++;
			}
		}

		/*
		img : Mat(500,181, CV_16UC1)

		*/
		static void ccDetection(Mat& img, Person* dtctPpl, int& ttl_dtctPpl, vector<PointMapping>* pntsMap2, int& ttlPnts, int debug, int frames, int debugFrame, bool isRPS)
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
					//linearize version of the threshold
					float thresh;
					if (isRPS)
					{
						float range = (img_Rows-(i))*RANGE_STEP; //binSize
						
						/*if (range > 4333)
							thresh = -0.46*range+4500;
						else if (range > 3750 && range <= 4333)
						{
							thresh = -2*range+12500;
						}
						else
							thresh = 0.6*range + 4000;*/

						if (range > 7133)
							thresh = -0.2*range+2000;
						else if (range > 2892 && range <= 7133)
						{
							thresh = -1.7*range+12700;
						}
						else
							thresh = 2*range + 2000;

			
						
					}
					else //Detection in the MoA
					{
						float X = j*ActivityMap_Utils::X_STEP + ActivityMap_Utils::MIN_X;
						float Z = (img.rows - i)*ActivityMap_Utils::Z_STEP;
						float range = sqrtf(X*X + Z*Z);

						if (range > 5213)
							thresh = -0.06*range+600;
						else if (range > 2250 && range <= 5213)
						{
							thresh = -range+5500;
						}
						else
							thresh = -3*range + 10000;

					}

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
			detectCC(img, dtctPpl, ttl_dtctPpl, pntsMap2, ttlPnts, outDebug, debug, frames, debugFrame, isRPS);
			if (debug >= DEBUG_MED)
			{
				//Show the change points in the thresholds
				int rowLow = RANGE_ROWS - ((int)3750/RANGE_STEP);	
				line(outDebug, Point(0,rowLow), Point(RANGE_COLS, rowLow), Scalar(0,0,255));
				imshow("Thresholds", outDebug);	
			}
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

		static void displayRPSDetections(Person* dtctPpl, int ttl_dtctPpl, Mat& remapPolar, int debug)
		{
						
			for (int iter = 0; iter < ttl_dtctPpl; iter++)
			{
				const Person* p = &(dtctPpl[iter]);
					
				Scalar color = Scalar(255,0,0);
				
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
					
				Scalar color = Scalar(255,0,0);
				
				if (debug >= DEBUG_NONE)
				{
					cv::circle(remapPolar, p->mean_RPS, 2, Scalar::all(0), -1);
					cv::ellipse(remapPolar, p->mean_RPS, Size(p->sigmaX_RPS*2, p->sigmaY_RPS*2), 0,0,360, Scalar::all(0));
				}

				Point meanMoA = Point(p->stateMoA.at<float>(0,0), p->stateMoA.at<float>(1,0));

				drawPersonPointsCov_debug(meanMoA, p->covMoA_points, &moa, Scalar::all(0), 1, NULL);

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


		static void drawPersonMean_debug(const Person& p, Mat* moa, Scalar color)
		{
			Point pntMean = Point(p.stateMoA.at<float>(0,0),p.stateMoA.at<float>(1,0));

			cv::circle(*moa, pntMean, 2, color, -1);	
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

		static void normalizeRPS(const Mat* src, Mat& out)
		{
			double max = 30000;
			//minMaxIdx(*src, NULL, &max);
			if (max != 0)
			{
				src->convertTo(out, CV_8UC1, 255/max);
			
				subtract(cv::Scalar::all(255),out, out);

			}
			else
			{
				out = Mat::zeros(out.size(), CV_8UC1) + 255;
				//Utils::initMat1u(out, 255);
			}
		}

}

