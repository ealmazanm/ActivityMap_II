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
		static const int MAX_PEOPLE = 20;

		static float meas_std = 1;
		static float proc_std = 30;
		static const int MAX_POINTS_BIN = 20000;

		struct PersonIPS
		{
			int id;
			Point meanIPS;
			Mat covIPS;

			Mat mean3D;
			Mat cov3D;

			Point meanMoA;
			Mat covMoA;

			Scalar colour;
		};

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

		static ofstream depthOut("c:\\Dropbox\\PhD\\Matlab\\DetectionEval\\IPS\\depths_TrainigDSet_II.txt");
		static ofstream depthOutSm("c:\\Dropbox\\PhD\\Matlab\\DetectionEval\\IPS\\depths_TrainigDSetSm_II.txt");


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

		static void updateActivityMapII(Mat& activityMap, Mat& activityMap_back, const ActivityMap_Utils* am, const XnPoint3D* p3D, const int nP, const XnPoint3D* points2D)
		{
			for (int i = 0; i < nP; i++)
			{
				Point p2D = ActivityMap_Utils::findMoACoordinate(&p3D[i], MAX_RANGE, MODEL_MAX_HEIGHT, MODEL_MIN_HEIGHT);

				if (p2D.x != -1)
				{
					if (p2D.y >= 480)
						p2D.y = 479;
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
			// 1  - background
			// 0  - unlabelled foreground
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
					if(row[x] == 0) 
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



		static void findBlobsII(const cv::Mat &binary, std::vector < std::vector<XnPoint3D> > &blobs, const XnDepthPixel* dMap)
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
					if(row[x] == 0) 
					{
						continue;
					}
					else if (row[x] == 1)
					{
						cv::Rect rect;
						cv::floodFill(label_image, cv::Point(x,y), label_count, &rect, 0, 0, 4);

						std::vector <XnPoint3D> blob;

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
								XnPoint3D p2dD;
								p2dD.X = j; p2dD.Y = i;
								p2dD.Z = dMap[i* XN_VGA_X_RES + j];
								blob.push_back(p2dD);
							}
						}

						blobs.push_back(blob);

						label_count++;
					}
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

		static void maskOutOverlapping(Mat& depthMat, Rect rLeft, Rect rRight)
		{
			Mat leftRoi = depthMat(rLeft);
			Mat rightRoi = depthMat(rRight);
			leftRoi = Mat::zeros(leftRoi.size(), CV_16U);
			rightRoi = Mat::zeros(rightRoi.size(), CV_16U);
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

			cv::circle(*moa, pntMean, 2, color, thickness);
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
				

				/*bool found = false;
				int iter = 0;
				while (!found && iter < tracks.size())
				{		
					found = (trckPpl[i].id == tracks[iter].id);
					iter++;
				}
				if (!found)*/
				{
					TrackInfo trck;
					trck.id = trckPpl[i].id;
					trck.trajectory.push_back(pos);
					tracks.push_back(trck);
				}
				//else
				//	tracks[--iter].trajectory.push_back(pos);
			}
		}


		/*
		Update the tracks trajectory with the information from the current frames.
		Used to store in memory the trajectories of all tracks in the entire sequence
		*/
		static void generateTrackHistoryIPS(vector<TrackInfo>& tracks, PersonIPS* trckPpl, int& ttl_trckPpl, int frames)
		{
			for (int i = 0; i < ttl_trckPpl; i++)
			{
				//Create the position at current frame
				PersonIPS p = trckPpl[i];
				Position pos;
				pos.frameId = frames;
				pos.mean = p.meanMoA;
				pos.covX = p.covMoA.ptr<float>(0)[0];
				pos.covY = p.covMoA.ptr<float>(1)[1];
				pos.covXY = p.covMoA.ptr<float>(0)[1];

				TrackInfo trck;
				trck.id = trckPpl[i].id;
				trck.trajectory.push_back(pos);
				tracks.push_back(trck);
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


		static Mat createForegroundImage(XnPoint3D* p2D, int ttlPnts)
		{
			//Create a binary image of foreground points
			Mat foreBin = Mat::zeros(XN_VGA_Y_RES, XN_VGA_X_RES, CV_8UC1);
			for (int i = 0; i < ttlPnts; i++)
			{
				XnPoint3D* p = &(p2D[i]); 
				uchar* ptr = foreBin.ptr<uchar>(p->Y);
				ptr[(int)p->X] = 255;
			}
			return foreBin;
		}

		static void detectPeopleIPS(XnPoint3D* p2D, int ttlPnts, Person* dtctPpl, int& ttl_dtctPpl, int debug, int frames)
		{
			//Create a binary image of foreground points
			Mat foreBin = Mat::zeros(XN_VGA_Y_RES, XN_VGA_X_RES, CV_8UC1);
			for (int i = 0; i < ttlPnts; i++)
			{
				XnPoint3D* p = &(p2D[i]); 
				uchar* ptr = foreBin.ptr<uchar>(p->Y);
				ptr[3*(int)p->X] = 255;
			}




			//debug: throw to the depth dimension
			if (debug > DEBUG_MED)
			{
				for (int i = 0; i < ttlPnts; i++)
				{
					depthOut << p2D[i].Z << endl;
				}
			}
		}

		static void thresholdSmallBlobs(std::vector < std::vector<XnPoint3D> > blobs, std::vector < std::vector<XnPoint3D> >& blobsFilter)
		{
			vector<vector<XnPoint3D>>::iterator iter = blobs.begin();

			while (iter != blobs.end())
			{
				vector<XnPoint3D> b = *iter;
				int maxI = b.size();
				if (maxI > 600)
					blobsFilter.push_back(b);

				iter++;
			}

		}
		
		static void updateForegroundImgII(Mat& foreBinary, std::vector < std::vector<XnPoint3D> > blobs, const XnDepthPixel* dMap, bool print)
		{
			foreBinary = Mat::zeros(XN_VGA_Y_RES, XN_VGA_X_RES, CV_8UC3);
			vector<vector<XnPoint3D>>::iterator iter = blobs.begin();

			//static ofstream depthOut("c:\\Dropbox\\PhD\\Matlab\\DetectionEval\\IPS\\depths_hist.txt");
			//char common[] = "c:\\Dropbox\\PhD\\Matlab\\DetectionEval\\IPS\\blobs\\depths_histFail401_";
			int i = 0;
			while (iter != blobs.end())
			{
				bool pass = false;
				vector<XnPoint3D> b = *iter;

				int maxI = b.size();
				if (maxI > 600)
				{
					//char idTxt[15];
					//itoa(i, idTxt, 10);
					//char path[150];
					//strcpy(path, common);
					//strcat(path, idTxt);
					//strcat(path, ".txt");
					//ofstream depthOutBlob(path);


					//int ttlX, ttlY, ttlD;
					//ttlX = ttlY = ttlD = 0;
					for (int i = 0; i < maxI; i++)
					{
						XnPoint3D p = b[i];
						//int d = dMap[(int)p.y*XN_VGA_X_RES+ (int)p.x];
						//ttlD += d;
						//ttlX += p.x;
						//ttlY += p.y;
						foreBinary.ptr<uchar>((int)p.Y)[3*(int)p.X] = 255;
						foreBinary.ptr<uchar>((int)p.Y)[3*(int)p.X + 1] = 255;
						foreBinary.ptr<uchar>((int)p.Y)[3*(int)p.X + 2] = 255;
						//if (print)
						//	depthOutBlob << d << endl;
					}

					/*int meanX = ttlX/maxI;
					int meanY = ttlY/maxI;
					int meanD = ttlD/maxI;
					char txt[20];
					itoa(meanD, txt, 10);*/
				//	putText(foreBinary, txt, Point(meanX, meanY), FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, Scalar(0,0,255));
				//	depthOut << meanD << " " << maxI << endl;
					/*if (print)
						print = false;*/

				}
				iter++;
				i++;
			}
		}


		static void writeOutTrainingDSetMat(Mat& trainDsetValues, Mat& trainDsetValuesSmooth)
		{
			ushort* d_data = (ushort*)trainDsetValues.data;
			ushort* d_dataSmooth = (ushort*)trainDsetValuesSmooth.data;

			int ttl = trainDsetValues.cols;
			int maxVal = 0;
			int maxBin = -1;
			for (int i = 0; i < ttl; i++)
			{
				int vSm = d_dataSmooth[i];
				if (vSm >maxVal)
				{
					maxVal = vSm;
					maxBin = i;
				}
			}
			float depth = maxBin*20 + (20/2);
			depthOutSm << depth << " " << maxVal << endl;

		}

		static void writeOutBlobPoints(Mat& depthHistS, int idBlob, int frames)
		{
			char common[200];
			strcpy(common, "c:\\Dropbox\\PhD\\Matlab\\DetectionEval\\IPS\\blobs\\depths_hist_Miss_262_");
			
			char idTxt[15];
			itoa(idBlob, idTxt, 10);
			char path[150];
			strcpy(path, common);
			strcat(path, idTxt);
			strcat(path, ".txt");
			ofstream depthOutBlob(path);

			ushort* d_data = (ushort*)depthHistS.data;
			int ttl = depthHistS.cols;
			for (int i = 0; i < ttl; i++)
			{
				int v = d_data[i];
				if (v >= 0)
					depthOutBlob << v << endl;
			}
			
		}


		
		void static writeOutTrainingDSet(vector<int> trainDsetValues)
		{
			vector<int>::iterator iterB = trainDsetValues.begin();
			vector<int>::iterator iterE = trainDsetValues.end();
			int i = 0;
			while (iterB != iterE)
			{
				int v = *iterB;
				if (v > 0)
				{
					float depth = i*20 + (20/2);
					depthOut << depth << " " << v << endl;
				}
				else if (v < 0)
					cout << "Error writingOutTrainingDsest" << endl;

				iterB++;
				i++;
			}

		}

		void static populateDepthHist(vector<int> &histD, XnPoint3D* p3d, int ttlPnts)
		{
			for (int i = 0; i < ttlPnts; i++)
			{
				XnPoint3D *p = &(p3d[i]);
				int bin = p->Z/20;
				histD[bin]++;
			}

		}

		static void populateMatDepthHist(Mat& depthHist, XnPoint3D* p3d, int ttlPnts)
		{
			ushort* d_data = (ushort*)depthHist.data;
			//int d_step = depthHist.step/sizeof(ushort);

			for (int i = 0; i < ttlPnts; i++)
			{
				XnPoint3D *p = &(p3d[i]);
				int bin = p->Z/20; //size of the bins 20 mm.
				d_data[bin]++;
			}

		}

				
		void static populateDepthHist (vector<int> &histD, std::vector<cv::Point2d > blob, const XnDepthPixel* dMap)
		{
			vector<Point2d>::iterator iterPnts = blob.begin();
			vector<Point2d>::iterator iterPntsEnd = blob.end();
			while (iterPnts != iterPntsEnd)
			{
				Point2d p = *iterPnts;
				int d = dMap[(int)p.y*XN_VGA_X_RES+ (int)p.x];
				int bin = d/20;
				histD[bin]++;
				iterPnts++;
			}
		}

		//Store the number of depths in each range (bin)
		void static populateTDSet(vector<int> &trainDsetValues, std::vector < std::vector<cv::Point2d > > blobsFilter, const XnDepthPixel* dMap)
		{
			vector<vector<Point2d>>::iterator iterBlobs = blobsFilter.begin();
			vector<vector<Point2d>>::iterator iterBlobsEnd = blobsFilter.end();
			
			while (iterBlobs != iterBlobsEnd)
			{
				vector<Point2d> points = *iterBlobs;
				
				populateDepthHist(trainDsetValues, points, dMap);
				
				iterBlobs++;
			}

		}


		static void binarizeMatDepthHist(Mat& depthHistS, Mat& depthHistBin, float a, float b)
		{
			ushort* d_dataS = (ushort*)depthHistS.data;
			uchar* d_dataB = (uchar*)depthHistBin.data;

			int ttlRows = depthHistS.cols;
			for (int i = 0; i < ttlRows; i++)
			{
				int v = d_dataS[i];
				float dBin = i*20; //bin size (mm.)
				float thresh = a*exp(b*dBin);
				if (v >= thresh)
					d_dataB[i] = 1;
				else
					d_dataB[i] = 0;

			}
		}


		static void binarizeDepthHist(const vector<int> &depthHist, vector<int> &depthHistBin, float a, float b)
		{
			int max = depthHist.size();
			for (int i = 0; i < max; i++)
			{
				int v = depthHist[i];
				float dBin = i*200; //bin size (mm.)
				float thresh = a*exp(b*dBin);
				if (v >= thresh)
					depthHistBin[i] = 1;
				else
					depthHistBin[i] = 0;
			}
		}


		static void findBlobs1DMat(Mat& depthHistBin, Mat& depthHist, std::vector < std::vector<int> > &blobs1D, std::vector <float> &blobsDCentroid)
		{
			blobs1D.clear();

			// Fill the label_image with the blobs
			// 0  - background
			// 1  - unlabelled foreground
			// 2+ - labelled foreground

	
			int label_count = 2; // starts at 2 because 0,1 are used already		
			int max = depthHistBin.cols;

			ushort* d_dataS = (ushort*)depthHist.data;
			uchar* d_dataB = (uchar*)depthHistBin.data;

			for(int i=0; i < max; i++) 
			{
				if(d_dataB[i] == 0) 
				{
					continue;
				}
				else if (d_dataB[i] == 1)
				{
					std::vector <int> blob;
					bool end = false;
					int j = i;
					int ttlD = 0; //sum of weights
					double wd = 0; //for the mean
					//double wdd = 0; //for the variance
					while (j < max && d_dataB[j] == 1)
					{
						d_dataB[j] = label_count;
						int ttlDBin = d_dataS[j];

						ttlD += ttlDBin;
						wd += j*ttlDBin;
						//wdd += ttlDBin*j*j;

						blob.push_back(j);
						j++;
					}
					double centroid = wd/ttlD;
					//double var = wdd/ttlD - (centroid*centroid);
					//if (var < 0)
					//	cout << "Stop" << endl;
					//if (var == 0)
					//	var = 1;

					//double std = powf(var, 0.5);
					//pair<double, double> p;
					//p.first = centroid*200; //The centroid is expressed in mm.
					//p.second = std*200;
					blobsDCentroid.push_back(centroid*20); //centroid expressed in mm.
					blobs1D.push_back(blob);
					label_count++;
				}
			}

		}


		/*
		Find Blobs in a 1D histogram
		blobs1D: contain the blobs and for each blob its corresponding bins
		blobsDCentroid: For each blob associates its centroid (weighed bin);
		*/
		static void findBlobs1D(vector<int> &depthHistBin, vector<int> &depthHist, std::vector < std::vector<int> > &blobs1D, std::vector <float> &blobsDCentroid)
		{
			blobs1D.clear();

			// Fill the label_image with the blobs
			// 0  - background
			// 1  - unlabelled foreground
			// 2+ - labelled foreground

	
			int label_count = 2; // starts at 2 because 0,1 are used already		
			int max = depthHistBin.size();

			for(int i=0; i < max; i++) 
			{
				if(depthHistBin[i] == 0) 
				{
					continue;
				}
				else if (depthHistBin[i] == 1)
				{
					std::vector <int> blob;
					bool end = false;
					int j = i;
					int ttlD = 0; //sum of weights
					double wd = 0; //for the mean
					//double wdd = 0; //for the variance
					while (j < max && depthHistBin[j] == 1)
					{
						depthHistBin[j] = label_count;
						int ttlDBin = depthHist[j];

						ttlD += ttlDBin;
						wd += j*ttlDBin;
						//wdd += ttlDBin*j*j;

						blob.push_back(j);
						j++;
					}
					double centroid = wd/ttlD;
					//double var = wdd/ttlD - (centroid*centroid);
					//if (var < 0)
					//	cout << "Stop" << endl;
					//if (var == 0)
					//	var = 1;

					//double std = powf(var, 0.5);
					//pair<double, double> p;
					//p.first = centroid*200; //The centroid is expressed in mm.
					//p.second = std*200;
					blobsDCentroid.push_back(centroid*200); 
					blobs1D.push_back(blob);
					label_count++;
				}
			}
		}

		
		static int closerBlobI(int d, std::vector <float>&  blobsDistr)
		{
			float minDist = 10000;
			int minPos = -1;
			int nBlobs = blobsDistr.size();
			for (int i = 0; i< nBlobs; i++)
			{
				float bDstr = blobsDistr[i];
				float dist = abs(d - bDstr);
				if (dist < minDist)
				{
					minDist = dist;
					minPos = i;
				}
			}
			if (minPos != -1)
				return minPos;
			else
			{
				cout << "Error" << endl;
				return -2;
			}

		}

		/*
		Find the centroid closer to p among all centroids
		*/
		static int closerBlob(const Point2d* p, const std::vector <float>& blobsDCentroid, const XnDepthPixel* dMap)
		{
			float minDist = 10000;
			int minPos = -1;
			int nBlobs = blobsDCentroid.size();
			int d = dMap[(int)p->y*XN_VGA_X_RES+ (int)p->x];
			for (int i = 0; i< nBlobs; i++)
			{
				float bDstr = blobsDCentroid[i];
				float dist = abs(d - bDstr);
				if (dist < minDist)
				{
					minDist = dist;
					minPos = i;
				}
			}
			if (minPos != -1)
				return minPos;
			else
			{
				cout << "Error" << endl;
				return -2;
			}
		}

		//Returns false if there is not enough evidence to support a person
		static bool look4MergesBlob(XnPoint3D* p2dD, XnPoint3D* p3d, int ttlPnts, std::vector < std::vector<XnPoint3D> > &subBlobs2dD, std::vector < std::vector<XnPoint3D> > &subBlobs3d, bool print, int idBlob, int frames, int cam)
		{
			//Exponentical coeeficients ae^bx
			float a = 3300;
			float b = -0.0006;
			Mat depthHist = Mat::zeros(1,500, CV_16S);
			Mat depthHistS = Mat::zeros(1,500, CV_16S);
			Mat depthHistBin = Mat::zeros(1,500, CV_8UC1);

			populateMatDepthHist(depthHist, p3d, ttlPnts);
			GaussianBlur(depthHist, depthHistS, Size(7, 1), 0, 0);
			if (print && frames == 264 && cam == 2)
			{
				writeOutBlobPoints(depthHistS, idBlob, frames); 
				//writeOutTrainingDSetMat(depthHist, depthHistS);
			}

			binarizeMatDepthHist(depthHistS, depthHistBin, a, b);
			std::vector<std::vector<int>> blobs1D;
			std::vector <float> blobsDistr;
			findBlobs1DMat(depthHistBin, depthHist, blobs1D, blobsDistr);

			int nBlobs = blobs1D.size();
			if(nBlobs > 1)
			{
				//create a new list of sub-blobs
				subBlobs2dD = std::vector < std::vector<XnPoint3D> >(nBlobs);
				subBlobs3d = std::vector < std::vector<XnPoint3D> >(nBlobs);

				for (int i = 0; i < ttlPnts; i++)
				{
					int d = p3d[i].Z;
					int idBlob = closerBlobI(d, blobsDistr);
					if (idBlob > -1 && idBlob < nBlobs)
					{
						subBlobs2dD[idBlob].push_back(p2dD[i]);
						subBlobs3d[idBlob].push_back(p3d[i]);
					}
					else
						cout << "Error" << endl;
				}
			}
			if (nBlobs == 0)
				return false;
			else 
				return true;

		}

		//static void look4Merges(std::vector < std::vector<XnPoint3D> > blobsFilter, std::vector < std::vector<XnPoint3D> > &finalBlobs, const XnDepthPixel* dMap, int frames, int cam)
		//{
		//	//Exponentical coeeficients ae^bx
		//	float a = 14170;
		//	float b = -0.0005532;
		//	vector<int> depthHist(50);
		//	vector<int> depthHistBin(50);
		//	//for each blob determine how many blobs contain
		//	std::vector < std::vector<XnPoint3D> >::iterator iterB = blobsFilter.begin();
		//	std::vector < std::vector<XnPoint3D> >::iterator iterE = blobsFilter.end();
		//	while (iterB != iterE)
		//	{
		//		std::fill(depthHist.begin(), depthHist.end(), 0);
		//		std::vector<XnPoint3D> blob = *iterB;
		//		populateDepthHist(depthHist, blob, dMap);
		//		binarizeDepthHist(depthHist, depthHistBin, a, b);
		//		std::vector<std::vector<int>> blobs1D;
		//		std::vector <float> blobsDistr;
		//		findBlobs1D(depthHistBin, depthHist, blobs1D, blobsDistr);
		//		//If it contain more than one- then classify the pixels of the big blob among the smaller blobs and add them to the final list
		//		int nBlobs = blobs1D.size();
		//		if(nBlobs > 1)
		//		{
		//			//create a new list of sub-blobs
		//			std::vector < std::vector<cv::Point2d > > subBlobs(nBlobs);

		//			int nPoints = blob.size();
		//			for (int i = 0; i < nPoints; i++)
		//			{
		//				Point2d* p = &(blob[i]);
		//				if(frames == 312 && cam == 0 && p->x == 357 && p->y == 371)
		//					cout << "Stop" << endl;
		//				int idBlob = closerBlob(p, blobsDistr, dMap);
		//				if (idBlob > -1 && idBlob < nBlobs)
		//					subBlobs[idBlob].push_back(*p);
		//				else
		//					cout << "Error" << endl;
		//			}

		//			//Add them all to the final list
		//			for (int i = 0; i < nBlobs; i++)
		//				finalBlobs.push_back(subBlobs[i]);
		//		}
		//		else
		//			finalBlobs.push_back(blob);


		//		iterB++;
		//	}
		//}



		static void updateForegroundImgIII(Mat& foreBinFinal, const std::vector < std::vector<cv::Point2d > > &finalBlobs)
		{
			int ttlBlobs = finalBlobs.size();
			for (int i = 0; i < ttlBlobs; i++)
			{
				int red = rand() % 255 + 1;
				int green = rand() % 255 + 1;
				int blue = rand() % 255 + 1;
				
				std::vector<cv::Point2d > blob = finalBlobs[i];
				int ttlPnts = blob.size();
				for (int j = 0; j < ttlPnts; j++)
				{
					Point2d p = blob[j];
					uchar* ptr = foreBinFinal.ptr<uchar>(p.y);
					ptr[3*(int)p.x] = blue;
					ptr[3*(int)p.x+1] = green;
					ptr[3*(int)p.x+2] = red;

				}
			}
		}
				
		/*
		Initialize the values of the bins in the height and colour model
		*/
		static void initPersonIPS(PersonIPS* p, int id)
		{
			p->id = id;

			p->meanIPS = Point(-1,-1);
			p->covIPS = Mat::zeros(2,2, CV_32F);

			p->mean3D = Mat::zeros(3,1, CV_32F);
			p->cov3D = Mat::zeros(3,3, CV_32F);

			p->meanMoA = Point(-1,-1);
			p->covMoA = Mat::zeros(2, 2, CV_32F);

			int red = rand() % 255 + 1;
			int green = rand() % 255 + 1;
			int blue = rand() % 255 + 1;
			p->colour = Scalar(red, green, blue);
	
		}



		static void detectPeople(const XnDepthPixel* dMap, XnPoint3D* b2dD, XnPoint3D* b3d, int ttlBPnts, KinectSensor& kinect, PersonIPS* dtctPpl, int& ttl_dtctPpl, int cam, Mat& foreImg, bool print, int& idP, const Rect& rLeft, const Rect& rRight)
		{
			uchar* d_data = (uchar*)foreImg.data;
			
			PersonIPS prs;
			initPersonIPS(&prs, idP);

			double x2d, y2d, xx2d, yy2d, xy2d, d2d;
			double x3d, y3d, z3d, xx3d, yy3d, zz3d, xy3d, xz3d, yz3d;
			d2d = x2d = y2d = xx2d = yy2d = xy2d = 0;
			x3d = y3d = z3d = xx3d = yy3d = zz3d = xy3d = xz3d = yz3d = 0;

			for (int i = 0; i < ttlBPnts; i++)
			{
				XnPoint3D p2dD = b2dD[i];
				XnPoint3D p3d = b3d[i];

				int d = dMap[(int)p2dD.Y*XN_VGA_X_RES + (int)p2dD.X];

				d2d += d;
				x2d += p2dD.X;
				y2d += p2dD.Y;
				xx2d += p2dD.X * p2dD.X;
				yy2d += p2dD.Y *  p2dD.Y;
				xy2d += p2dD.X * p2dD.Y;

				x3d += p3d.X; y3d += p3d.Y; z3d += p3d.Z;
				xx3d += p3d.X * p3d.X;
				yy3d += p3d.Y * p3d.Y;
				zz3d += p3d.Z * p3d.Z;

				xy3d += p3d.X * p3d.Y;
				xz3d += p3d.X * p3d.Z;
				yz3d += p3d.Y * p3d.Z;

				if (print)
				{
					uchar* ptr = d_data + ((int)p2dD.Y)*foreImg.step;
					ptr[3*(int)p2dD.X] = prs.colour.val[0];
					ptr[3*(int)p2dD.X + 1] = prs.colour.val[1];
					ptr[3*(int)p2dD.X + 2] = prs.colour.val[2];
				}
			}
				
			//Mean 2D
			prs.meanIPS.x = x2d/ttlBPnts;
			prs.meanIPS.y = y2d/ttlBPnts;
			float meanIPSDepth = d2d/ttlBPnts;
			XnPoint3D meanIPS2dD;
			meanIPS2dD.X = prs.meanIPS.x;
			meanIPS2dD.Y = prs.meanIPS.y;
			meanIPS2dD.Z = meanIPSDepth;
			XnPoint3D meanIPS3d;
			kinect.arrayBackProject(&meanIPS2dD, &meanIPS3d, 1);
			kinect.transformArray(&meanIPS3d, 1);



			if (cam == 1 && (prs.meanIPS.x < rLeft.width || prs.meanIPS.x > rRight.x))
				return;

			idP++;
			//cov 2D
			double varxx = (xx2d/ttlBPnts) - powf(prs.meanIPS.x,2);
			double varyy = (yy2d/ttlBPnts) - powf(prs.meanIPS.y,2);
			double varxy = (xy2d/ttlBPnts) - (prs.meanIPS.x * prs.meanIPS.y);
			float covIPS[] = {varxx, varxy, varxy, varyy};
			Mat m = Mat(2,2, CV_32F, covIPS);
			m.copyTo(prs.covIPS);

			//mean 3D
			float mean3d[] = {x3d/ttlBPnts, y3d/ttlBPnts, z3d/ttlBPnts};
			Mat s = Mat(3, 1, CV_32F, mean3d);
			s.copyTo(prs.mean3D);
			//cov 3D
			double varxx3d = (xx3d/ttlBPnts) - powf(mean3d[0], 2);
			double varyy3d = (yy3d/ttlBPnts) - powf(mean3d[1], 2);
			double varzz3d = (zz3d/ttlBPnts) - powf(mean3d[2], 2);
			double varxy3d = (xy3d/ttlBPnts) - (mean3d[0] * mean3d[1]);
			double varxz3d = (xz3d/ttlBPnts) - (mean3d[0] * mean3d[2]);
			double varyz3d = (yz3d/ttlBPnts) - (mean3d[1] * mean3d[2]);
			float cov3D[] = {varxx3d, varxy3d, varxz3d, varxy3d, varyy3d, varyz3d, varxz3d, varyz3d, varzz3d};
			Mat ss = Mat(3,3, CV_32F, cov3D);
			ss.copyTo(prs.cov3D);

			Mat rTiltNeg = (Mat)kinect.rotTiltNeg;
			Mat rTilt = (Mat)kinect.rotTilt;
			
			Mat sTilt = s.t() * (Mat)rTiltNeg;
			

			//Transform into a common CS
			Mat r = (Mat)kinect.rotation;
			Mat t = (Mat)kinect.translation;
			
			Mat sCCS = r*sTilt.t() + t; //mean
			Mat sCCST = sCCS.t() * rTilt; //final mean 

			//Cova
			Mat ssCCS = r.t() * ss * r; // cov
			//extend Cov
		//	Mat extCov = Mat::zeros(4,4, CV_32F) + 1;
		//	Mat mCov =extCov(Rect(0,0, 3,3));
		//	ssCCS.copyTo(mCov);
			

			//float covVals[] = {1/ActivityMap_Utils::X_STEP, 0, 0, ActivityMap_Utils::MIN_X, 0, 0, 1/ActivityMap_Utils::Z_STEP, ActivityMap_Utils::MIN_Z};
			//Mat projMoAMat = Mat(2,4, CV_32F, covVals);

			float covVals[] = {(float)1/ActivityMap_Utils::X_STEP, 0, 0, 0, 0, (float)1/ActivityMap_Utils::Z_STEP};
			Mat projMoAMat = Mat(2,3, CV_32F, covVals);

			//finalCov: 2x2
			Mat finalCov = projMoAMat * ssCCS * projMoAMat.t();
			finalCov.copyTo(prs.covMoA);


			//Project into MoA
			/*XnPoint3D mean3D;
			mean3D.X = sCCST.ptr<float>(0)[0];
			mean3D.Y = sCCST.ptr<float>(0)[1];
			mean3D.Z = sCCST.ptr<float>(0)[2];
			prs.meanMoA = ActivityMap_Utils::findMoACoordinate(&mean3D, MAX_RANGE, MODEL_MAX_HEIGHT, MODEL_MIN_HEIGHT);*/
			prs.meanMoA = ActivityMap_Utils::findMoACoordinate(&meanIPS3d, MAX_RANGE, MODEL_MAX_HEIGHT, MODEL_MIN_HEIGHT);
			

			

				
			dtctPpl[ttl_dtctPpl++] = prs;
		}

		static void detectPeople(const std::vector < std::vector<cv::Point2d > > &finalBlobs, const XnDepthPixel* dMap, KinectSensor& kinect, PersonIPS* dtctPpl, int& ttl_dtctPpl, int cam)
		{
			int ttlBlobs = finalBlobs.size();
			ttl_dtctPpl = ttlBlobs;
			for (int i = 0; i < ttlBlobs; i++)
			{
				PersonIPS prs;
				initPersonIPS(&prs, cam*3+i);

				std::vector<cv::Point2d > blob = finalBlobs[i];
				int ttlPnts = blob.size();
				XnPoint3D* p2dd = new XnPoint3D[ttlPnts];
				double x, y, xx, yy, xy;
				x = y = xx = yy = xy = 0;
				for (int j = 0; j < ttlPnts; j++)
				{
					Point2d p = blob[j];
					p2dd[j].X = p.x;
					p2dd[j].Y = p.y;
					p2dd[j].Z = dMap[(int)p.y*XN_VGA_X_RES + (int)p.x];

					x += (int)p.x;
					y += (int)p.y;
					xx += (int)p.x*p.x;
					yy += (int)p.y*p.y;
					xy += (int)p.x*p.y;
				}
				prs.meanIPS.x = x/ttlPnts;
				prs.meanIPS.y = y/ttlPnts;

				double varxx = (xx/ttlPnts) - powf(prs.meanIPS.x,2);
				double varyy = (yy/ttlPnts) - powf(prs.meanIPS.y,2);
				double varxy = (xy/ttlPnts) - (prs.meanIPS.x * prs.meanIPS.y);
				float covIPS[] = {varxx, varxy, varxy, varyy};
				Mat m = Mat(2,2, CV_32F, covIPS);
				m.copyTo(prs.covIPS);
				
				dtctPpl[i] = prs;

				delete [] p2dd;
			}

		}

		static void displayDetectedPplIPS(const PersonIPS* dtctPpl, const int& ttlDtctPpl, Mat& rgbImg, Mat& MoA)
		{
			for (int i = 0; i < ttlDtctPpl; i++)
			{
				const PersonIPS* prs = &(dtctPpl[i]);

				char txt[15];
				itoa(prs->id, txt, 10);
				
				//cv::circle(MoA, prs->meanMoA, 2, prs->colour, 2);
				drawPersonPointsCov_debug(prs->meanIPS, prs->covIPS, &rgbImg, prs->colour, 2, NULL);
				drawPersonPointsCov_debug(prs->meanMoA, prs->covMoA, &MoA, prs->colour, 2, NULL);
				
			}

		}

}