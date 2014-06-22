#pragma once
#include <Utils.h>
#include <vld.h>
#include "hungarian.h"
#include <tinystr.h>
#include <tinyxml.h>

#define max(a,b) (((a) > (b)) ? (a) : (b))

namespace AMv2
{

	//INTERVALS TIME EXECUTION
		static const int TOTAL_INTERVALS = 14;
		static const int BSUB_ID = 0;
		static const int RPSPACE_ID = 1;
		static const int MOA_ID = 2;
		static const int DET_ID = 3;
		static const int BACKPR_ID = 4;
		static const int PTRANS_ID = 5;
		static const int SMOOTH_ID = 6;
		static const int DISPLAY_ID = 7;
		static const int TRACK_ID = 8;
		static const int PROBMAP_ID = 9;
		static const int UPDATE_APP = 10;
		static const int REMOVE_ID = 11;
		static const int CLEAN_ID = 12;
		static const int TOT_ID = 13;
		static char* titles[TOTAL_INTERVALS] = {"BACKGROUND SUBTRACTION", "REMAP POLAR SPACE", "MAP OF ACTIVITY", "DETECTION", "POINTS BACKPROJECTION", "POINTS TRANSFORMATION", "SMOOTH", "DISPLAY", "TRACKING", 
			"PROB MAP", "UPDATE APPEARANCE", "REMOVE LOST TRACKS", "CLEAN POINTS", " TOTAL"};
		static float totalIntervals[TOTAL_INTERVALS] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};

		//Time interval sub-function RPS
		static const int TOTAL_SUBINTERVAL_PROB = 2;
		static const int MVNPDF_MEAN_ID = 0;
		static const int MVNPDF_MULT_ID = 1;
		static char* titles_subProb[TOTAL_SUBINTERVAL_PROB] = {"MVNPDF_MEAN", "MVNPDF_MULT"};
		static float totalSubIntervalsProb[TOTAL_SUBINTERVAL_PROB] = {0,0};


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
		static const int MIN_NUM_POINTS_BIN = 50;
		//static const int MODEL_NBINS = (ActivityMap_Utils::CEILING_THRESHOLD-ActivityMap_Utils::FLOOR_THRESHOLD)/MODEL_BINRANGE;
		static const int MAX_PEOPLE = 250;

		static float meas_std = 1;
		static float proc_std = 30;
		static const int MAX_POINTS_BIN = 20000;
		struct gaussianParam
		{
			Scalar mean;
			Mat cov;
			Mat covInv;
			double detCov;
			float normTerm;

			//Debug
			double rr, gg, bb, rg, rb, gb;
			//const XnRGB24Pixel** colours;
			//int totalColours;
		};

		struct Person
		{
			int id;

			Point2d  mean_RPS;
			double sigmaY_RPS;
			double sigmaX_RPS;

			//Point2d meanMoA;
			Mat extentMoA;
			//Rect bBoxMoA;
			RotatedRect rrMoA;
	
			int heightModel[MODEL_NBINS];
			float heightModelPdf[MODEL_NBINS];
			gaussianParam colourModel[MODEL_NBINS];
			int lost;
			Scalar colour;

			int heightModelDtcTtl[MODEL_NBINS];
			float heightModelDtc[MODEL_NBINS];
			float heightModelPdfDtc[MODEL_NBINS];
			gaussianParam colourModelDtc[MODEL_NBINS];

			//debug
			float maxHeight;
			float minHeight;
			int idDetection;

			Mat apperance;
			Mat apperanceRed;
			Mat apperanceRedDtc;
			int countApperance[MODEL_NBINS];

			Mat rotation; //matrix that rotates a covariance matrix depending on its mean position in the MoA
		};

		struct EllipseParam
		{
			Point mean;
			double covX;
			double covY;
			double covXY;
		};

		struct PointMappingII
		{
			const XnPoint3D* p3D;
			const XnRGB24Pixel* colour;
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

	
	//Tracking variables
		static const int TRACKLOST_THRESHOLD = 5;
		//update coefficients
		static float ALPHA = 0.05;
		static float BETA = 0.01;

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
		static void updatePolarAlternateive(Mat* polarAlt, Mat* polar, vector<PointMapping>* pntsMap2 , int& ttlPnts, const XnPoint3D* p3D, const XnPoint3D* points2D, const XnRGB24Pixel* rgbMap, bool* activeP, const int nP, int debug, int camId)
		{
			float toRad = 180/CV_PI;
			int pAltStep = polarAlt->step/sizeof(ushort);
			int pStep = polar->step/sizeof(ushort);

			ushort* ptrPAlt = (ushort*)polarAlt->data;
			ushort* ptrP = (ushort*)polar->data;


			clock_t startTime;
			for (int i = 0; i < nP; i++)
			{
				if (!activeP[i])
					continue;
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
			p->lost = 0;
			for (int i = 0; i < MODEL_NBINS; i++)
			{
				p->heightModel[i] = 0;
				p->heightModelPdf[i] = 0.0;
				p->colourModel[i].cov = Mat::zeros(3,3, CV_32F);
				p->colourModel[i].normTerm = 0;
				//p->colourModel[i].totalColours = 0;
				//p->colourModel[i].colours = new const XnRGB24Pixel*[MAX_POINTS_BIN];
				p->colourModel[i].rr = p->colourModel[i].gg = p->colourModel[i].bb = 0;
				p->colourModel[i].rg = p->colourModel[i].rb = p->colourModel[i].gb = 0;
								
				p->heightModelDtcTtl[i] = 0;
				p->heightModelDtc[i] = 0.0;
				p->heightModelPdf[i] = 0.0;
				p->colourModelDtc[i].cov = Mat::zeros(3,3, CV_32F);
				p->colourModelDtc[i].normTerm = 0;
				p->colourModelDtc[i].rr = p->colourModelDtc[i].gg = p->colourModelDtc[i].bb = 0;
				p->colourModelDtc[i].rg = p->colourModelDtc[i].rb = p->colourModelDtc[i].gb = 0;
				
				p->countApperance[i] = 0;
			}

			Point2d meanMoA = Point (-1,-1);
			Mat extentMoA = Mat::zeros(2,2, CV_32FC1)-1;
			Rect bBoxMoA = Rect(-1,-1,-1,-1);
	
			int red = rand() % 255 + 1;
			int green = rand() % 255 + 1;
			int blue = rand() % 255 + 1;
			p->colour = Scalar(red,green, blue);

			//Image for debuggin to show the apperance model
			p->apperanceRed = Mat::zeros(400, 50, CV_8UC3);
			p->apperanceRedDtc = Mat::zeros(400,50,CV_8UC3);
			p->apperance = Mat(400,370, CV_8UC3);
			Utils::initMat3u(p->apperance, 255);
			Utils::initMat3u(p->apperanceRed, 255);
			Utils::initMat3u(p->apperanceRedDtc, 255);
			for (int i = 1; i < MODEL_NBINS; i++)
			{
				int row =  i*50;
				line(p->apperance, Point(0, row), Point(370, row), Scalar::all(0));
				line(p->apperanceRed, Point(0, row), Point(50, row), Scalar::all(0));
				line(p->apperanceRedDtc, Point(0, row), Point(50, row), Scalar::all(0));
			}
			line(p->apperance, Point(50,0), Point(50, 400), Scalar::all(0));

			//for debuggin
			p->maxHeight = -2000;
			p->minHeight = 300;
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


			covCartessian.copyTo(prs->extentMoA);
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
			

			Mat covCartessian = R*covMoA*R.t();

			covCartessian.copyTo(prs->extentMoA);

			//convert the covariance matrix into a bbox;
			SVD svd(covCartessian);
	
			float bigAxis = sqrtf(svd.w.at<float>(0))*2;
			float smallAxis = sqrtf(svd.w.at<float>(1))*2;

			//identify the quadrant of the main eigenvector
			bool upperQuadrant = (svd.u.at<float>(1,0) > 0);
			Mat bigEigenVct = svd.u(Rect(0,0, 1,2));
			float valsR[] = {1, 0};
			Mat mainAxis = Mat(2,1, CV_32F, valsR);
			float dotPrd = bigEigenVct.dot(mainAxis);
			float angle = acosf(dotPrd)*180/CV_PI;
			if (!upperQuadrant)
				angle = -angle;

			prs->rrMoA.angle = angle;
			//NOT SURE ABOUT THIS
			prs->rrMoA.size = Size(smallAxis, bigAxis);

		}

	
		//Projects the detected gaussian distribution (mean, variance) to the MoA (non linear)
		static void projectLocation2MoA(Person* prs, int debug, int frames, int debugFrame)
		{
			//Mean projection to MoA
			Point cMoA;
			cMoA = convertBack(&prs->mean_RPS);
			XnPoint3D p3D;
			p3D.X = cMoA.x; p3D.Y = MODEL_MAX_HEIGHT - 1; p3D.Z = cMoA.y;
			prs->rrMoA.center = ActivityMap_Utils::findMoACoordinate(&p3D, MAX_RANGE, MODEL_MAX_HEIGHT, MODEL_MIN_HEIGHT);
			if (prs->rrMoA.center.x == -1)
			{
				if (debug > DEBUG_NONE)
				{
					outDebugFile << "ProjectLocation: person located out of MoA range. INFO: RPS point: " << prs->rrMoA.center.x << ", " << prs->rrMoA.center.y <<
						". 3D MoA point: " << cMoA.x << ", " << cMoA.y << ". Range: " << sqrtf(pow(p3D.X,2) + pow(p3D.Z,2)) <<". MAX_RANGE: " << 
						MAX_RANGE << ". MAX_Z_TRANS: " << ActivityMap_Utils::MAX_Z_TRANS << ". CEILING_THRESH: " << MODEL_MAX_HEIGHT << 
						". FLOOR_THRESH: " << MODEL_MIN_HEIGHT << endl;
				}
				return; //TODO: ERROR LOG
			}
		
			//First option (using jacobian matrices)
			//projectCovariance_Jacob(prs);

			//Second option (projecting the variances)
			projectCovariance_Var(prs);

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


		static void updateAppearanceImg(Person* trgt)
		{
			for (int j = 0; j < MODEL_NBINS; j++)
			{
				int numBinPnts = trgt->heightModel[j];
				if (numBinPnts > MIN_NUM_POINTS_BIN)
				{
					gaussianParam* mbc = trgt->colourModel + j;
					
					//fill the mean colour in the apperance image
					int red = mbc->mean.val[0];
					int green = mbc->mean.val[1];
					int blue = mbc->mean.val[2];
					int step = j*50;
					for (int y = 0; y < 50; y++)
					{
						uchar* ptrRed = trgt->apperanceRed.ptr<uchar>(y+step);
						for (int x = 0; x < 50; x++)
						{
							ptrRed[x*3] = blue;
							ptrRed[x*3+1] = green;
							ptrRed[x*3+2] = red;
						}
					}
				}
			}
		}

			
		static void updateAppearanceImgDtc(Person* trgt)
		{
			for (int j = 0; j < MODEL_NBINS; j++)
			{
				int numBinPnts = trgt->heightModelDtcTtl[j];
				if (numBinPnts > MIN_NUM_POINTS_BIN)
				{
					gaussianParam* mbc = trgt->colourModelDtc + j;
					
					//fill the mean colour in the apperance image
					int red = mbc->mean.val[0];
					int green = mbc->mean.val[1];
					int blue = mbc->mean.val[2];
					int step = j*50;
					for (int y = 0; y < 50; y++)
					{
						uchar* ptrRed = trgt->apperanceRedDtc.ptr<uchar>(y+step);
						for (int x = 0; x < 50; x++)
						{
							ptrRed[x*3] = blue;
							ptrRed[x*3+1] = green;
							ptrRed[x*3+2] = red;
						}
					}
				}
			}
		}

		static void buildAppearanceModel(vector<Point2d>& b, Person* prs, vector<PointMapping>* pntsMap2, int debug, int debugFrame, int frames)
		{
			int ttlRowsApp = prs->apperance.rows;
			int ttlColsApp = prs->apperance.cols - 50;

			double x, y, xx, yy, ww;
			x = y = xx = yy = ww = 0;
			int maxP = b.size();
			int ttl = 0;
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
						if (debug > DEBUG_NONE)
						{
							//Fill the debug image of the apperance						
							int rowPos = prs->countApperance[bin]/ttlColsApp;
							int colPos = (prs->countApperance[bin]%ttlColsApp + 50);
							prs->apperance.ptr<uchar>(bin*50+rowPos)[colPos*3] = (uchar)colour->nBlue;
							prs->apperance.ptr<uchar>(bin*50+rowPos)[colPos*3+1] = (uchar)colour->nGreen;
							prs->apperance.ptr<uchar>(bin*50+rowPos)[colPos*3+2] = (uchar)colour->nRed;
							prs->countApperance[bin]++;
						}

						gaussianParam* modelBinColour = prs->colourModel + bin;

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
						ttl++;
						if (debug >= DEBUG_MED && frames == 96 && prs->id == 1)
						{
							outPersModel << height << " " << (int)colour->nRed << " " << (int)colour->nGreen << " " << (int)colour->nBlue << endl;
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
				if (numBinPnts > MIN_NUM_POINTS_BIN)
				{
					prs->heightModelPdf[j] = (float)numBinPnts/(float)ttl;
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

					if (debug > DEBUG_NONE)
					{
						//fill the mean colour in the apperance image
						int red = mbc->mean.val[0];
						int green = mbc->mean.val[1];
						int blue = mbc->mean.val[2];
						int step = j*50;
						for (int y = 0; y < 50; y++)
						{
							uchar* ptr = prs->apperance.ptr<uchar>(y+step);
							uchar* ptrRed = prs->apperanceRed.ptr<uchar>(y+step);
							for (int x = 0; x < 50; x++)
							{
								ptrRed[x*3] = ptr[x*3] = blue;
								ptrRed[x*3+1] = ptr[x*3+1] = green;
								ptrRed[x*3+2] = ptr[x*3+2] = red;
							}
						}
					}
					Mat inv = mbc->cov.inv();
					inv.copyTo(mbc->covInv);
					mbc->detCov = determinant(mbc->cov);
					mbc->normTerm = 1/(2*CV_PI*sqrt(2*CV_PI)*sqrtf(mbc->detCov));

				}
				else
				{
					prs->heightModel[j] = 0;
					prs->heightModelPdf[j] = 0;
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
						if (frames == 196)
							cout << "stop" << endl;
						//Todo build appearance model
						startTime = clock();
						buildAppearanceModel(b, prs, pntsMap2, debug, debugFrame, frames);
						totalSubIntervalsDetection[BUILDAPPEARANCE_ID] += clock() - startTime; //time debugging

						startTime = clock();
						projectLocation2MoA(prs, debug, frames, debugFrame);
						totalSubIntervalsDetection[PROJECT2MOA_ID] += clock() - startTime;
					}
				}
				iter++;
			}
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
					else
						thresh = 0.6*range + 4000;
			
					if (val < thresh)
						(*valPtr) = 0;
				}
			}
			img.copyTo(imgCpy2);
			totalSubIntervalsDetection[FIRSTTHRES_ID] += clock() - startTime; //time debugging

			Mat outDebug = Mat(RANGE_ROWS, 181, CV_8UC3);	
		
			detectCC(img, dtctPpl, ttl_dtctPpl, pntsMap2, ttlPnts, outDebug, debug, frames, debugFrame);
			
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

		static void drawPersonPointsCov_debug(const RotatedRect rr,  Mat* moa, Scalar color, int thickness, char* txt)
		{
			cv::ellipse(*moa, rr.center, rr.size, rr.angle, 0, 360, color, thickness);	
			if (txt != NULL)
				putText(*moa, txt, Point(rr.center.x + rr.size.height, rr.center.y), FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, Scalar(0,0,255));
		}

		//static void drawPersonPointsCov_debug(const Perso&n p, Mat* moa, Scalar color)
		static void drawPersonPointsCov_debug(const Point& pntMean, const Mat& cov, Mat* moa, Scalar color, int thickness, char* txt)
		{
			SVD svd(cov);
	
			float bigAxis = sqrtf(svd.w.at<float>(0));
			float smallAxis = sqrtf(svd.w.at<float>(1));

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
			SVD svd(p.extentMoA(Rect(0,0,2,2)));
			/*printValuesF(&svd.u, "Eigen vectors (U)", outDebugFile);
			printValuesF(&svd.w, "Eigen values (W)", outDebugFile);
			printValuesF(&svd.vt, "Eigen vectors (Vt)", outDebugFile);*/
	
			float bigAxisX = svd.u.at<float>(0,0);
			float bigAxisY = svd.u.at<float>(1,0);

			float angle = atanf(bigAxisY/bigAxisX)*180/CV_PI;

			float bigAxisMag = svd.w.at<float>(0);
			float smallAxisMag = svd.w.at<float>(1);

			cv::ellipse(*moa, p.rrMoA.center, Size(bigAxisMag, smallAxisMag), angle, 0, 360, color, 3);	
		}


		static void displayRPSDetections(Person* dtctPpl, int ttl_dtctPpl, Mat& remapPolar, int debug)
		{
						
			for (int iter = 0; iter < ttl_dtctPpl; iter++)
			{
				const Person* p = &(dtctPpl[iter]);
					
				Scalar color = Scalar(0,0,255);
				
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
					
				Scalar color = Scalar(0,0,255);

				if (debug >= DEBUG_NONE)
				{
					cv::circle(remapPolar, p->mean_RPS, 2, Scalar::all(0), -1);
					cv::ellipse(remapPolar, p->mean_RPS, Size(p->sigmaX_RPS*2, p->sigmaY_RPS*2), 0,0,360, Scalar::all(0));
				}

				drawPersonPointsCov_debug(p->rrMoA.center, p->extentMoA, &moa, Scalar::all(0), 1, NULL);

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

				Scalar color = p->colour; //occluded blob
				if (p->lost > 0)
					color = Scalar(0,0,0);
				
				//Tracking ellipse
				char txt[15];
				itoa(p->id, txt, 10);
					
				//phisical extent
				//drawPersonPointsCov_debug(p->rrMoA.center, p->extentMoA, &moa, color, 2, txt);
				drawPersonPointsCov_debug(p->rrMoA, &moa, color, 2, txt);
				
			}
		}

		static void drawPersonMean_debug(const Person& p, Mat* moa, Scalar color)
		{
			cv::circle(*moa, p.rrMoA.center, 2, color, -1);	
		}


		static float mD(Mat& m, Mat& covInv)
		{
			float* xM = (float*)m.data;

			int stepCovInv = covInv.step/sizeof(float);

			float* a = (float*) covInv.data;
			float* b = (float*)(covInv.data + stepCovInv);
			float* c = (float*)(covInv.data + 2*stepCovInv);

			float xVal = m.at<float>(0);
			float yVal = m.at<float>(1);
			float zVal = m.at<float>(2);

			/*float xVal = xM[0];
			float yVal = xM[1];
			float zVal = xM[2];*/

			return (covInv.ptr<float>(0)[0]*xVal*xVal + 2* covInv.ptr<float>(1)[0]*xVal+yVal + 2* covInv.ptr<float>(2)[0] *xVal*zVal +

				2* covInv.ptr<float>(1)[2] *yVal*zVal + covInv.ptr<float>(1)[1]*yVal*yVal + covInv.ptr<float>(2)[2]*zVal*zVal);
			
			//return (a[0]*xVal*xVal + 2*b[0]*xVal+yVal + 2*c[0]*xVal*zVal + 2*b[2]*yVal*zVal + b[1]*yVal*yVal + c[2]*zVal*zVal);


		}


		//Todo: Add determinant to person
		static float mvnpdf(const Scalar& xC, const Scalar& mu, Mat& S, Mat& Sinv, double detS, float normTerm)
		{
			clock_t startTime_tmp = clock();
			
				//Means subtraction
				Scalar ms_Data = xC - mu;
				double* ms_DataPtr = ms_Data.val;
			
			totalSubIntervalsProb[MVNPDF_MEAN_ID] += clock() - startTime_tmp; //time debugging

			startTime_tmp = clock();
			
				int stepSinv = Sinv.step/sizeof(float);
				float* Sinv_Data = (float*)Sinv.data;
				
				float a = *Sinv_Data; 
				float b = *(Sinv_Data + stepSinv); 
				float c = *(Sinv_Data + 2*stepSinv);  
				float d = *(Sinv_Data + stepSinv + 1);  
				float e = *(Sinv_Data + stepSinv + 2);
				float f = *(Sinv_Data + 2*stepSinv + 2);

				float x = ms_DataPtr[0];
				float y = ms_DataPtr[1];
				float z = ms_DataPtr[2];

				float mDist = a*x*x + 2*b*x*y + 2*c*x*z + 2*e*y*z + d*y*y + f*z*z;
			
			totalSubIntervalsProb[MVNPDF_MULT_ID] += clock() - startTime_tmp; //time debugging

			if (normTerm == NULL)
				normTerm = 1/(2*CV_PI*sqrt(2*CV_PI)*sqrtf(detS));

			return normTerm * exp(-mDist/2);
		}

		static float bhattacharyaDist_Cont(const gaussianParam* trgt, const gaussianParam* cndt, int debug, int frames)
		{
			//dist = 8pi sqrt(2pi) det(S_tgt*S_cndt)^(1/4) * (1 / (2pi * sqrt(2pi) * det(2*(S_tgt+S_cndt)))exp( (Mean_trt - Mea_cndt)' * S^-1 * (Mean_trt - Mean_cndt) )/2 )

			Mat S_trgt = trgt->cov;
			Mat S_cndt = cndt->cov;

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

			double S1det = determinant(S1);
			double S3det = determinant(S3);

			if (S1det != 0 && S3det != 0)
			{
				Mat S2Inv = S2.inv();
				float prob = mvnpdf(trgt->mean, cndt->mean, S2, S2Inv, determinant(S2), 0);
				float constTerm = 8*CV_PI*sqrt(2*CV_PI)*powf(S3det,0.25);
				return constTerm * prob;
			}
			else
			{
				outDebugFile << "CompareColours. Error Determinant = 0. Info: Pooled Sigma det(Strgt + SCndt): " << S1det  <<
					" det(Strgt*Scndt): " << S3det << endl;
				return -1;
			}
		}

	
		/*
		out = (dPrs.mean - target.mean) * inv((target.gtCov + dPrs.covMoa)/2) * (dPrs.mean - target.mean)T
		*/
		static float mahalanobis(const Mat* tgtMean, const Mat* uncertInv, const Person* dPrs)
		{
			//Mean diff
			float vals[] = {dPrs->rrMoA.center.x, dPrs->rrMoA.center.y};
			Mat dtcMean = Mat(2,1, CV_32FC1, vals);
			Mat meanDiff = dtcMean-(*tgtMean);

			//sum covariances
			Mat out = meanDiff.t() * (*uncertInv) * meanDiff;

			float val = out.at<float>(0,0);

			return val;

		}

	
		static void updateMShiftModel(Person* target)
		{
			float updateCoeff = 0;
			for (int i = 0; i < MODEL_NBINS; i++)
			{
				int numHeightMsr = target->heightModelDtcTtl[i];
				int numHeightTrgt = target->heightModel[i];
				if (numHeightMsr > MIN_NUM_POINTS_BIN)
				{
					//case 1: full update
					if (numHeightTrgt > MIN_NUM_POINTS_BIN)
						updateCoeff = ALPHA;
					
					//Case 2: slow update
					else 
						updateCoeff = 1;
		
					//update height
					target->heightModel[i] = numHeightTrgt*(1 - updateCoeff) + numHeightMsr*updateCoeff;
					//update colour
					gaussianParam* clrMdlTrgt = &target->colourModel[i];
					const gaussianParam* clrMdlCndt = &target->colourModelDtc[i];

					Mat tgtCov = clrMdlTrgt->cov * (1 - updateCoeff) + clrMdlCndt->cov*updateCoeff;
					tgtCov.copyTo(clrMdlTrgt->cov);
					Mat sInv = tgtCov.inv();
					sInv.copyTo(clrMdlTrgt->covInv);
					clrMdlTrgt->detCov = determinant(tgtCov);
					clrMdlTrgt->normTerm = 1/(2*CV_PI*sqrt(2*CV_PI)*sqrtf(clrMdlTrgt->detCov));


					Scalar s = clrMdlTrgt->mean*(1-updateCoeff)  + clrMdlCndt->mean*updateCoeff;
					clrMdlTrgt->mean = s;
				}
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
				if (numHeightMsr > 50)
				{
					//case 1: full update
					if (numHeightTrgt > 50)
						updateCoeff = ALPHA;
			
					//Case 2: slow update
					else 
						updateCoeff = BETA;
		
					//update height
					target->heightModel[i] = numHeightTrgt*(1 - updateCoeff) + numHeightMsr*updateCoeff;
					//update colour
					gaussianParam* clrMdlTrgt = &target->colourModel[i];
					const gaussianParam* clrMdlCndt = &msr->colourModel[i];

					Mat tgtCov = clrMdlTrgt->cov * (1 - updateCoeff) + clrMdlCndt->cov*updateCoeff;
					tgtCov.copyTo(clrMdlTrgt->cov);

					Scalar s = clrMdlTrgt->mean*(1-updateCoeff)  + clrMdlCndt->mean*updateCoeff;
					clrMdlTrgt->mean = s;
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
				pos.mean = p.rrMoA.center;
				pos.covX = p.extentMoA.at<float>(0,0);
				pos.covY = p.extentMoA.at<float>(1,1);
				pos.covXY = p.extentMoA.at<float>(0,1);
				
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

		static Rect getUncertainArea(const Person* p)
		{
			
			int sz = max(p->extentMoA.at<float>(0,0), p->extentMoA.at<float>(1,1))*1.5;
			int x = max(p->rrMoA.center.x - (sz/2), 0);
			int y = max(p->rrMoA.center.y - (sz/2), 0);
			return Rect(x,y, sz, sz);
		}

		/*
		Calculates the probability of p3D with colour c to belong to the trgt
		The trgt appearance model need to be normalized (pdf).
		*/
		static float calculateProb(const Person* trgt, const XnPoint3D& p3D, const XnRGB24Pixel* c, int frames)
		{
			if (p3D.Y < MODEL_MAX_HEIGHT && p3D.Y > MODEL_MIN_HEIGHT)
			{
				int bin = (MODEL_MAX_HEIGHT - p3D.Y)/MODEL_BINRANGE;	
				float hProb = trgt->heightModelPdf[bin];
				if (hProb == 0)
					return 0;

				gaussianParam gp = trgt->colourModel[bin];
				//Mat S = trgt->colourModel[bin].cov;
				//Mat Sinv = trgt->colourModel[bin].covInv;

				
				float cProb = mvnpdf(Scalar(c->nRed, c->nGreen, c->nBlue), trgt->colourModel[bin].mean, gp.cov, gp.covInv, gp.detCov, gp.normTerm);
				

				float prob = hProb*cProb;
				if ( !(prob == 0) && !(prob > 0) && !(prob < 0))
				{
					outDebugFile << "At frame: " << frames << ". Bin: " << bin; 
					Scalar cMean = trgt->colourModel[bin].mean;
					outDebugFile << ". Error: Height Prob: " << hProb << " Colour: (" << (int)c->nRed << ", " << (int)c->nGreen << ", " << (int)c->nBlue <<
						"). Colour Mean: (" << cMean.val[0] << ", " << cMean.val[1] << ", " << cMean.val[2] << "). " << endl;
					printValuesF(&gp.cov, "Colour Cov", outDebugFile);
					return 0;
				}
				return prob;
			}
			else
				return 0;
		}


		static Rect getUncertainArea(Rect r, Size imgSZ, float scale)
		{
			
			Rect out;
			int xInc = r.x-(r.width* scale);
			int yInc = r.y - (r.height* scale);
			int width = r.width* (1 + 2*scale);
			int height = r.height* (1 + 2*scale);
			int xMean = max(0, xInc);
			int yMean = max(0, yInc);
			out.width = min(width, imgSZ.width-xMean);
			out.height = min(height, imgSZ.height-yMean);
			out.x = xMean; out.y = yMean;
			return out;
			
		}

		static void getMoA2DPoints(XnPoint3D** points3D, Point** moa2D, int* numP)
		{
			for (int i = 0; i < NUM_SENSORS; i++)
			{
				XnPoint3D* p3DSensor = points3D[i];
				Point* moa2DSensor = moa2D[i];
				for (int j = 0; j < numP[i]; j++)
				{
					XnPoint3D p3D = p3DSensor[j];
					moa2DSensor[j] = ActivityMap_Utils::findMoACoordinate(&p3D, MAX_RANGE, MODEL_MAX_HEIGHT, MODEL_MIN_HEIGHT);
				}
			}
		}

		static void copyModel(Person* trgt)
		{
			for (int i = 0; i < MODEL_NBINS; i++)
			{
				trgt->heightModel[i] = trgt->heightModelDtcTtl[i];
				trgt->heightModelDtcTtl[i] = 0;

				trgt->heightModelPdf[i] = trgt->heightModelPdfDtc[i];
				trgt->heightModelPdfDtc[i] = 0;

				trgt->heightModelDtc[i] = 0;

				trgt->colourModel[i].mean = trgt->colourModelDtc[i].mean;
				trgt->colourModelDtc[i].mean.all(0);

				trgt->colourModelDtc[i].cov.copyTo(trgt->colourModel[i].cov);
				trgt->colourModelDtc[i].cov = NULL;

				trgt->colourModelDtc[i].covInv.copyTo(trgt->colourModel[i].covInv);
				trgt->colourModelDtc[i].covInv = NULL;

				trgt->colourModel[i].detCov = trgt->colourModelDtc[i].detCov;
				trgt->colourModelDtc[i].detCov = -1;

				trgt->colourModel[i].normTerm = trgt->colourModelDtc[i].normTerm;
				trgt->colourModelDtc[i].normTerm = -1;
			}

		}

		static void updateAppearanceDtct(Person* trgt, XnPoint3D** points3D, Point** moa2D, int* numP, XnPoint3D** points2D, const XnRGB24Pixel** rgbMap, int frames, int debug)
		{

			Rect bbox = trgt->rrMoA.boundingRect();
			float ttl = 0.0;
			for (int i = 0; i < NUM_SENSORS; i++)
			{
				Point* moa2DSnsr = moa2D[i];
				int ttlPnts = numP[i];
				for (int j = 0; j < ttlPnts; j++)
				{
					Point p2DMoa = moa2DSnsr[j];
					if (p2DMoa.x > bbox.x && p2DMoa.x < (bbox.x + bbox.width) &&
							p2DMoa.y > bbox.y && p2DMoa.y < (bbox.y + bbox.height))
					{
						
						float height = points3D[i][j].Y;

						if (height < MODEL_MAX_HEIGHT && height > MODEL_MIN_HEIGHT)
						{
							
							//epanechnikov kernel
							float x = powf(p2DMoa.x - trgt->rrMoA.center.x,2)/trgt->rrMoA.size.width;
							float y = powf(p2DMoa.y - trgt->rrMoA.center.y,2)/trgt->rrMoA.size.height;
							float w = max( 0.75 * (1 - ( x + y )), 0);
							if (w == 0)
								continue;
							//w = 1;
							
							XnPoint3D p2dig = points2D[i][j];
							int rgbMapPos = (int)p2dig.Y*XN_VGA_X_RES+(int)p2dig.X;
							const XnRGB24Pixel* colour = rgbMap[i] + rgbMapPos;
						
							int bin = (MODEL_MAX_HEIGHT - height)/MODEL_BINRANGE;	
						

							gaussianParam* modelBinColour = trgt->colourModelDtc + bin;
							float numBinPnts = trgt->heightModelDtc[bin];

							float wRed = w*colour->nRed;
							float wGreen = w*colour->nGreen;
							float wBlue = w*colour->nBlue;
							
							//update the model (height, colour)
							if (numBinPnts > 0.0)	
							{
								(*modelBinColour).mean(0) = (wRed + numBinPnts * (*modelBinColour).mean(0))/(numBinPnts+w);
								(*modelBinColour).mean(1) = (wGreen + numBinPnts * (*modelBinColour).mean(1))/(numBinPnts+w);
								(*modelBinColour).mean(2) = (wBlue + numBinPnts * (*modelBinColour).mean(2))/(numBinPnts+w);
							}
							else
								trgt->colourModelDtc[bin].mean = Scalar(wRed, wGreen, wBlue);

							//for the covariance
							(*modelBinColour).rr += w * colour->nRed*colour->nRed;
							(*modelBinColour).gg += w * colour->nGreen*colour->nGreen;
							(*modelBinColour).bb += w * colour->nBlue*colour->nBlue;
							(*modelBinColour).rg += w * colour->nRed * colour->nGreen;
							(*modelBinColour).rb += w * colour->nRed * colour->nBlue;
							(*modelBinColour).gb += w * colour->nGreen * colour->nBlue;

							trgt->heightModelDtcTtl[bin]++;
							trgt->heightModelDtc[bin] += w;
							ttl += w;

							if (debug  > DEBUG_HIGH && frames == 200)
							{
								outDebugFile << "Colour (" << (int)colour->nRed << ", " << (int)colour->nGreen << ", " << (int)colour->nBlue << ") -> (" <<
									wRed << ", " << wGreen << ", " << wBlue << ")." << "Weight: " << w << endl;
							}

						}
					}
				}
			}

			for (int j = 0; j < MODEL_NBINS; j++)
			{
				float numBinPnts = trgt->heightModelDtc[j];
				if (trgt->heightModelDtcTtl[j] > MIN_NUM_POINTS_BIN)
				{
					
					trgt->heightModelPdfDtc[j] = (float)numBinPnts/(float)ttl;
					gaussianParam* mbc = trgt->colourModelDtc + j;

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

					Mat inv = mbc->cov.inv();
					inv.copyTo(mbc->covInv);
					mbc->detCov = determinant(mbc->cov);
					mbc->normTerm = 1/(2*CV_PI*sqrt(2*CV_PI)*sqrtf(mbc->detCov));

				}
				else
				{
					trgt->heightModelDtc[j] = 0.0;
					trgt->heightModelPdfDtc[j] = 0.0;
				}
			}
			if (debug > DEBUG_HIGH && frames == 200)
			{
				outDebugFile << "Detection" << endl;
				int j = 1;
				outDebugFile << "At frame: " << frames << ". Bin: " << j; 
					Scalar cMean = trgt->colourModelDtc[j].mean;
					outDebugFile << ". Error: Height Prob: " << trgt->heightModelPdfDtc[j] << " Colour Mean: (" << cMean.val[0] << ", " << cMean.val[1] << ", " << cMean.val[2] << "). " << endl;
					printValuesF(&trgt->colourModelDtc[j].cov , "Colour Cov", outDebugFile);
			}
		}

		/*
		Build a probability image using the points and the appearance model of the target
		/ points3D: 3D foreground points from the three cameras
		/ moa2D: is the position in the MoA of all 3D points.
		/ points2D: Points coodinates in the image plane.
		/ rgbMap: RGB representation of the three cameras
		*/
		static void createMoAp(const Person* trgt, XnPoint3D** points3D, Point** moa2D, int* numP, XnPoint3D** points2D, const XnRGB24Pixel** rgbMap, Mat& MoAp, int frames, bool** activeP)
		{

			//Mat MoAa = Mat::zeros(MoAp.size(), CV_32FC1) + 1;
			//Defines an area (rectangle) around the target position
			//it is not calculated for the entire image.
			//Rect probArea = getUncertainArea(trgt);
			Rect probArea = getUncertainArea(trgt->rrMoA.boundingRect(), MoAp.size(), 0.25);

			for (int i = 0; i < NUM_SENSORS; i++)
			{
				XnPoint3D* p3DSensor = points3D[i];
				const XnPoint3D* p2DSensor = points2D[i];
				const XnRGB24Pixel* rgbMapSensor = rgbMap[i];
				Point* moa2DSensor = moa2D[i];
				bool* activePSensor = activeP[i];
				for (int j = 0; j < numP[i]; j++)
				{
					if (activePSensor[j]) // only if the point is enable
					{
						XnPoint3D p3D = p3DSensor[j];
						float height = p3D.Y;
						if (height < MODEL_MAX_HEIGHT && height > MODEL_MIN_HEIGHT)
						{
							Point p2D = moa2DSensor[j];
							if (p2D.x > probArea.x && p2D.x < (probArea.x + probArea.width) &&
								p2D.y > probArea.y && p2D.y < (probArea.y + probArea.height))
							{
								XnPoint3D p2d = (*(p2DSensor + j));
								int rgbMapPos = (int)p2d.Y*XN_VGA_X_RES+(int)p2d.X;
								const XnRGB24Pixel* colour = rgbMapSensor + rgbMapPos;
								float prob = calculateProb(trgt, p3D, colour, frames);
								MoAp.ptr<float>(p2D.y)[p2D.x] += prob;
							}
						}
					}
				}
			}
			
			//DEBUG: Get the minimum and max prob within the search area
			/*Mat MoApRoi = MoAp(probArea);
			double minVal; 
			double maxVal;  
			Point minLoc; 
			Point maxLoc;
			minMaxLoc( MoApRoi, &minVal, &maxVal, &minLoc, &maxLoc );
			outDebugFile << "min val : " << minVal << ". Max val: " << maxVal << endl;*/

		}

		static bool  lowProbability(Person* trgt, Rect r, Mat& MoAp)
		{
			//DEBUG: Get the minimum and max prob within the search area
			Mat MoApRoi = MoAp(r);
			float total = sum(MoApRoi).val[0];
			float avg = total/(r.width*r.height);
			//outDebugFile << "Target " << trgt->id <<". Avg Prob : " << avg << endl;
			return avg < 1.0e-6;
		}

		static void camShift_custom(Person* trgt,  Mat& MoAp, TermCriteria& term)
		{
			if (trgt->rrMoA.size.width == 0)
				cout << "Stop" << endl;
			//Rect r = trgt->rrMoA.boundingRect();
			Rect r = getUncertainArea(trgt->rrMoA.boundingRect(), MoAp.size(), 0.25);
			RotatedRect rr = CamShift(MoAp, r, term);
			if ( (!(rr.size.width == 0) && !(rr.size.width > 0) && !(rr.size.width < 0)) ||
				(!(rr.size.height == 0) && !(rr.size.height > 0) && !(rr.size.height < 0)) ||
				 (trgt->rrMoA.size.width == 0) || rr.size.width == 0 || rr.size.height == 0 ||
				 lowProbability(trgt, rr.boundingRect(), MoAp)) //TODO: ADD A PROBABILITY THRESHOLD TO BECOME LOST
				trgt->lost++;
			else
			{
				rr.size.width = min(rr.size.width, 50);
				rr.size.width = max(rr.size.width , 10);

				rr.size.height = min(rr.size.height, 50);
				rr.size.height = max(rr.size.height, 10);

				trgt->lost = 0;
				trgt->rrMoA = rr;
			}

			if (trgt->rrMoA.size.width == 0)
				cout << "Stop" << endl;
			//789

		}

		//Move left the elements filling the lost track postitions
		static void removeLostTracks(Person*& trckPpl, int& ttl_trckPpl)
		{
			int ttlRmv = 0;
			for (int i = 0; i < ttl_trckPpl; i++)
			{
				if (trckPpl[i].lost < TRACKLOST_THRESHOLD)
				{
					if (ttlRmv > 0)
						trckPpl[i-ttlRmv] = trckPpl[i];
				}
				else
					ttlRmv++;
			}
			ttl_trckPpl -= ttlRmv;

		}

		static void showActivePoints(Mat& mImg, Point** moa2D, bool** activeP, int* numP, int debug)
		{
			for (int i = 0; i < NUM_SENSORS; i++)
			{
				Point* p2D = moa2D[i];
				bool* act = activeP[i];
				for (int j = 0; j < numP[i]; j++)
				{
					Point p = p2D[j];
					if (debug > DEBUG_MED)
						outDebugFile << "Sensor: " << i <<", NumP: " << j << ".Point(" << p.x <<", "<<p.y<<") " << endl;
					if (p.x != -1)
					{
						if (act[j])
							mImg.ptr<uchar>(p.y)[p.x] = 0;
						else
							mImg.ptr<uchar>(p.y)[p.x] = 255;
					}
				}
			}

		}

		/*
		//Remove all points of trgt so they do not affect the rest of targets prediction
		//trgt: Person
		// moa2D: 3x1 arrray of MoA coordinates of points from target of interest with respect to the three cameras
		// activeP: A 2 dimenaional (3xn) array that contain the actual state (enable, disable) of all points from the trhee cameras
		// numP: A 3x1 array that contain the number of points for each camera
		// sz: Dimensions of the MoA.
		*/
		static void cleanPointsTarget(Person* p,  Point** moa2D, bool** activeP, int* numP, Size sz, Mat& MoAp, Mat& MoACopy, int debug)
		{
			float threshold = 1.0e-6;
			Rect r = getUncertainArea(p->rrMoA.boundingRect(), sz, 0.25); //TODO: CHANGE TO 0.5
			int xMax = r.x + r.width;
			int yMax = r.y + r.height;
			for (int i = 0 ; i < NUM_SENSORS; i++)
			{
				Point* p2DSensor = moa2D[i];
				bool* activePSensor = activeP[i];
				for (int j = 0; j < numP[i]; j++)
				{
					Point p2D = p2DSensor[j];
					if (p2D.x > -1 && p2D.y > -1)
					{
						float prob = MoAp.ptr<float>(p2D.y)[p2D.x];
						if (p2D.x > r.x && p2D.x < xMax && p2D.y > r.y && p2D.y < yMax && prob > threshold) //todo: check with the probability map and remove only if prob > T
						{	
							if (debug >= DEBUG_NONE)
							{
								uchar* ptrMoA= MoACopy.ptr<uchar>(p2D.y);
								ptrMoA[3*p2D.x] = 255;
								ptrMoA[3*p2D.x + 1] = 255;
								ptrMoA[3*p2D.x + 2] = 255;
							}
							activePSensor[j] = false;	
						}
					}
				}
			}
		}

		static void cleanPoints(Person*& trckPpl, int& ttl_trckPpl, Point** moa2D, bool** activeP, int* numP, Size sz)
		{
			for (int k = 0; k < ttl_trckPpl; k++)
			{
				Person p = trckPpl[k];
				Rect r = getUncertainArea(p.rrMoA.boundingRect(), sz, 0.25); //TODO: CHANGE TO 0.5
				int xMax = r.x + r.width;
				int yMax = r.y + r.height;
				for (int i = 0 ; i < NUM_SENSORS; i++)
				{
					Point* p2DSensor = moa2D[i];
					bool* activePSensor = activeP[i];
					for (int j = 0; j < numP[i]; j++)
					{
						Point p2D = p2DSensor[j];
						if (p2D.x > r.x && p2D.x < xMax &&
							p2D.y > r.y && p2D.y < yMax)
					
							activePSensor[j] = false;
					}
				}
			}

		}

		static void addNewTracks(Person*& trckPpl, int& ttl_trckPpl, Person*& dtctPpl,  int& ttl_dtctPpl, int& tgtIdCounter)
		{			
			for (int i = 0; i < ttl_dtctPpl; i++)
			{
				Person p = dtctPpl[i];
				p.id = tgtIdCounter++;
				trckPpl[ttl_trckPpl+i] = p;
			}
			ttl_trckPpl += ttl_dtctPpl;
		}

		static void printPerson(const Person* trgt, ostream& out)
		{
			out << "Target Id: " << trgt->id << endl;
			out << "Appeareance Model" << endl;
			
			for (int i = 0; i < MODEL_NBINS; i++)
			{
				gaussianParam gp = trgt->colourModel[i];
				out << "\nBin " << i << "- height: " << trgt->heightModel[i] << ", height(pdf): " << trgt->heightModelPdf[i] << ", Colour (mean): [" <<
					gp.mean.val[0] << ", " << gp.mean.val[1] << ", " << gp.mean.val[2] << "], Colour (cov)" << endl;
				printValuesF(&gp.cov, "Cov", out);
			}
		}


		static void getProbs(float& maxProb, float& ttlProb, Mat& MoAp, Rect& r)
		{
			int xInit = r.x;
			int yInit = r.y;
			int xEnd = xInit + r.width;
			int yEnd = yInit + r.height;
			
			for (int i = yInit; i < yEnd; i++)
			{
				float* ptr = MoAp.ptr<float>(i);
				for (int j = xInit; j < xEnd; j++)
				{
					float v = ptr[j];
					if (v > 0)
						outDebugFile << v << endl;

					ttlProb += v;
					if (v > maxProb)
						maxProb = v;

					if (!(ttlProb < 0) && !(ttlProb > 0) && !(ttlProb == 0))
						cout << "Stop" << endl;
				}
			}
		}

		static void sortByRange(Person* trckPpl, int ttl_trckPpl, const Point2d& origin)
		{
			// create a vector of ranges
			float* ranges = new float[ttl_trckPpl];
			for (int i = 0; i < ttl_trckPpl; i++)
			{
				Point2f c = (trckPpl[i]).rrMoA.center;
				ranges[i] = sqrtf( powf((origin.x - c.x),2) + powf((origin.y - c.y),2));
			}
			//sort array (bubble sort)
			int nChanges = 0;
			bool init = true;
			while (init || nChanges != 0)
			{
				nChanges = 0;
				for (int i = 0; i < (ttl_trckPpl-1); i++)
				{
					if (ranges[i] > ranges[i+1])
					{
						float tmp = ranges[i+1];
						Person tmpP = trckPpl[i+1];

						ranges[i+1] = ranges[i];
						trckPpl[i+1] = trckPpl[i];

						ranges[i] = tmp;
						trckPpl[i] = tmpP;
						nChanges++;
					}
				}

				init = false;
			}
	
			//free memory
			delete [] ranges;
		}
}
