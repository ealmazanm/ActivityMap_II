#include "BackgroundDepthSubtraction.h"
#include <ActivityMap_Utils.h>
#include "KinectSensor.h"
#include "Plane.h"
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <list>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <ctype.h>
#include "XnCppWrapper.h"
#include "connected_components.h"
//#include <vld.h>


//debug levels
static const int DEBUG_NONE = -1;
static const int DEBUG_LOW = 0;
static const int DEBUG_MED = 1;
static const int DEBUG_HIGH = 2;
static const int DEBUG_OUT = -2;

const int NUM_SENSORS = 3;

ifstream tiltTXT("D:\\CameraCalibrations\\extrinsics\\tilt.txt");
ofstream outStd("d:/Emilio/Kinect/DepthSensorStudy/PeopleVariance.txt");

//For debugin
ofstream outDebugFile("d:/Debug.txt");
ofstream fpsOut("c:\\Dropbox\\PhD\\Matlab\\FrameRate\\fps_BGUpdt.txt");
ofstream outPerson("c:\\Dropbox\\PhD\\Matlab\\PdF_person\\pdf_points.txt");
ofstream outHeights("c:\\Dropbox\\PhD\\Matlab\\Height_People\\heights.txt");
ofstream outPersModel ("c:\\Dropbox\\PhD\\Matlab\\Model\\modelInfo.txt");
ofstream outBboxModel ("c:\\Dropbox\\PhD\\Matlab\\BBoxes\\bboxInfo.txt");
ofstream outFPK0 ("C:\\Dropbox\\PhD\\Matlab\\Calibration_wks\\kinect0_full.txt");
ofstream outFPK1 ("C:\\Dropbox\\PhD\\Matlab\\Calibration_wks\\kinect1_full.txt");
ofstream outFPK2 ("C:\\Dropbox\\PhD\\Matlab\\Calibration_wks\\kinect2_full.txt");
ofstream outClrPnts ("C:\\Dropbox\\PhD\\Matlab\\ColourDistribution\\colourDist.txt");
int debugFrame = -1;

vector<vector<float>> peopleRange;
Rect area (-1, -1, -1, -1);
char* windPolarName = "Polar";
char* windMoA = "Activity Map";
char* windPolarSmooth = "Polar Alt Smooth";
RNG rng(12345);

int debug = DEBUG_NONE;

int frames = 0;

//Remap Polar Space
float MAX_RANGE;
float MAX_Z_TRNS;
const int RANGE_ROWS = 500;
const int RANGE_COLS = 181;

float DEPTH_SCALE = 1; // decide the scaling factor with respect to MAX_RANGE
int KINECTS_DISPLACEMENT = 0; 

//*IMPORTANT*//
//THIS TWO VALUES SHOULD NOT BE CHANGED.
//It will affect the amount of points projected in each pixel of the
//remap polar space. The direct consecuence is that the thresholds used for
//the detection will not be suitable.
const int RANGE_STEP = 24; //ceil(11650.0/RANGE_ROWS); //THIS VALUES MUST BE FIXED
const int SCALE_RANGE_ALT = 25896; //scale the alternative espace to match MAX_RANGE



//Time interval checking
const int TOTAL_INTERVALS = 10;
const int BSUB_ID = 0;
const int RPSPACE_ID = 1;
const int MOA_ID = 2;
const int DET_ID = 3;
const int BACKPR_ID = 4;
const int PTRANS_ID = 5;
const int SMOOTH_ID = 6;
const int DISPLAY_ID = 7;
const int TRACK_ID = 8;
const int TOT_ID = 9;
char* titles[TOTAL_INTERVALS] = {"BACKGROUND SUBTRACTION", "REMAP POLAR SPACE", "MAP OF ACTIVITY", "DETECTION", "POINTS BACKPROJECTION", "POINTS TRANSFORMATION", "SMOOTH", "DISPLAY", "TRACKING", " TOTAL"};
float totalIntervals[TOTAL_INTERVALS] = {0,0,0,0,0,0,0,0,0,0};

//Time interval sub-function RPS
const int TOTAL_SUBINTERVAL_RPS = 4;
const int ANGLE_ID = 0;
const int RANGE_ID = 1;
const int UPDATE_ID = 2;
const int FEATURE_ID = 3;
char* titles_subRPS[TOTAL_SUBINTERVAL_RPS] = {"ANGLE", "RANGE", "UPDATE", "FEATURE"};
float totalSubIntervalsRPS[TOTAL_SUBINTERVAL_RPS] = {0,0,0,0};

//Time interval sub-function Create Activity Map
const int TOTAL_SUBINTERVAL_MOA = 4;
const int CONVERT2PONITS_ID = 0;
const int ARRAYBACKPROJECT_ID = 1;
const int TRANSFORM_ID = 2;
const int FINDMOA_ID = 3;
char* titles_subMOA[TOTAL_SUBINTERVAL_MOA] = {"CONVERT 2 POINTS", "ARRAY BACKPROJECT", "TRANSFORM", "FINDMOA_ID"};
float totalSubIntervalsMOA[TOTAL_SUBINTERVAL_MOA] = {0,0,0,0};


//PERSON MODEL SETTINGS
const int MODEL_MAX_HEIGHT = -100;
const int MODEL_MIN_HEIGHT = -2400;
const int MODEL_BINRANGE = 230;
const int MODEL_NBINS = (MODEL_MAX_HEIGHT-MODEL_MIN_HEIGHT)/MODEL_BINRANGE;


//struct BinFeature
//{
//	Scalar meanColor;
//	int total;
//};


//TRACKING VARIABLES
const int DTC_FULL = 0;
const int DTC_MERGE = 1;
const int DTC_SPLIT = 2;

//Update linear model
float valsH[] = {1, 0, 0, 0, 0, 1, 0, 0};
Mat H = Mat(2,4, CV_32F, valsH);

//Covariance of the measurement error
//update coefficients
float ALPHA = 0.05;
float BETA = 0.01;


//max number of frames allowed to be lost
const int TRACKLOST_THRESHOLD = 10;
float const COMP_THRESHOLD = 0.5;
float meas_std = 10;
float proc_std = 1;
//float accel = 0.005;
const int MAX_POINTS_BIN = 7000;
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
const int MAX_PEOPLE = 50;
int pplId_cont = 0;
struct PointMapping
{
	const XnPoint3D* p3D;
	const XnRGB24Pixel* colour;
	int rpX;
	int rpY;
};


void selectROI_callBack(int event, int x, int y, int flags, void* param)
{
	Mat* polar = (Mat*)param;
	if (event == CV_EVENT_LBUTTONDOWN) 
	{
		if (area.x == -1)
		{
			area.x = x;
			area.y = y;
		}
	}
	if (event == CV_EVENT_LBUTTONUP)
	{
		if (area.width = -1)
		{
				area.width = x - area.x;
				area.height = y - area.y;
				rectangle(*polar, area, Scalar::all(0));
				cv::imshow(windPolarName, *polar);
				std::cout << "Area captured" << endl;

				//Convert and store all non zero points
				vector<float> personRange;
				for (int i = area.y+1; i < area.y+area.height-1; i++)
				{
					uchar* ptr = polar->ptr<uchar>(i); 
					for (int j = area.x+1; j < area.x+area.width-1; j++)
					{
						int val = ptr[j];
						if (val < 255)
						{
							//convert to range in space
							float range = i*RANGE_STEP;
							for (val; val < 255; val++)
								personRange.push_back(range);
						}
					}
				}
				peopleRange.push_back(personRange);
				area.x = -1; area.width = -1;
		}
	
	}
}

void selectPoint_callBack(int event, int x, int y, int flags, void* param)
{
	Mat* moa = (Mat*)param;
	if (event == CV_EVENT_LBUTTONDOWN) 
	{
		Mat tmp;
		moa->copyTo(tmp);
		circle(tmp, Point(x,y), 2, Scalar(0,0,255));
		
		imshow("Tmp", tmp);
		waitKey(0);
		

		float X = ActivityMap_Utils::findCoordinate_inv(x, ActivityMap_Utils::MIN_X, ActivityMap_Utils::MAX_X, ActivityMap_Utils::X_STEP);
		int yy = moa->rows - y;
		float Z = ActivityMap_Utils::findCoordinate_inv(yy, ActivityMap_Utils::MIN_Z, ActivityMap_Utils::MAX_Z, ActivityMap_Utils::Z_STEP);
						

		float range = sqrtf(pow(X,2) + pow(Z,2));
		int angle = 90;
		if (X != 0 && Z != 0)
			if (X > 0)
				angle = (atanf(Z/abs(X)))*180/CV_PI;
			else
				angle = 180-((atanf(Z/abs(X)))*180/CV_PI);

		float range_m = range/1000;
		float range_Alt = (int)SCALE_RANGE_ALT * ((0.33*log(10+3*range_m))-0.80597);
		int yAlt = RANGE_ROWS - ((int)range_Alt/RANGE_STEP);	

		//Calculate range
		float rangeAlt_back = (RANGE_ROWS - yAlt)*RANGE_STEP;
		float range_back = (-3.83328*(0.869577-exp( (3.0303*rangeAlt_back)/SCALE_RANGE_ALT) ) )*1000;

	
		float X_Back = range_back*cos(angle*CV_PI/180); 
		float Z_Back = range_back*sin(angle*CV_PI/180);


		XnPoint3D p3D;
		p3D.X = X_Back; p3D.Y = ActivityMap_Utils::CEILING_THRESHOLD-1; p3D.Z = Z_Back;
		Point p2D = ActivityMap_Utils::findMoACoordinate(&p3D, MAX_RANGE);
		if (p2D.x != -1)
			circle(tmp, Point(p2D.x, p2D.y), 2, Scalar(0,255,0));
		
		imshow("Tmp", tmp);
		waitKey(0);
		
	}
}


void selectPoint2_callBack(int event, int x, int y, int flags, void* param)
{
	Mat** images = (Mat**)param;
	Mat* moa = images[1];
	Mat* polarSmooth = images[0];
	
	if (event == CV_EVENT_LBUTTONDOWN) 
	{
		circle(*polarSmooth, Point(x,y), 2, Scalar::all(225));
		
		//imshow(windPolarSmooth, *polarSmooth);

		//Calculate range
		float rangeAlt_back = (RANGE_ROWS - y)*RANGE_STEP;
		float range_back = (-3.83328*(0.869577-exp( (3.0303*rangeAlt_back)/SCALE_RANGE_ALT) ) )*1000;

	
		XnPoint3D p3D;
		p3D.X = range_back*cos(x*CV_PI/180); 
		p3D.Z = range_back*sin(x*CV_PI/180);
		p3D.Y = ActivityMap_Utils::CEILING_THRESHOLD-1;
		
		Point p2D = ActivityMap_Utils::findMoACoordinate(&p3D,MAX_RANGE);
		if (p2D.x != -1)
			circle(*moa, Point(p2D.x, p2D.y), 2, Scalar(0,255,0));
		
		imshow(windMoA, *moa);
		waitKey(0);
		
	}
}


//Debug functions
void printValuesS(const Mat* m, char* title)
{
	cout << title << endl;
	for (int i = 0; i < m->rows; i++)
	{
		const ushort* ptr = m->ptr<ushort>(i);
		for (int j = 0; j < m->cols; j++)
		{
			cout << (int)ptr[j] << " ";
		}
		cout << endl;
	}
	cout << endl;
}

void printValuesF(const Mat* m, char* title, ostream& out)
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
polarAlt(out): Updates the positions where the 3D points project after the remapping.
polar(out): Points projected from the transform points into thep polar coordinate system
pntsMap(out): List of cooridnate mappings (3D coordinate + colour + 2d rmpspace coordinates
ttlPnts(out): Total num points that project on the rmpspace

p3D(in): List of foreground points for a particular kinect
points2D(in): List of 2D points of the image plane for a particular kinect
rgbMap(in): Map of rgb colours for a particular kinect
nP(in): Number of foreground points.
*/
void updatePolarAlternateive(Mat* polarAlt, Mat* polar, PointMapping* pntsMap, int& ttlPnts, const XnPoint3D* p3D, const XnPoint3D* points2D, const XnRGB24Pixel* rgbMap, const int nP)
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
void updateActivityMap(Mat& activityMap, Mat& activityMap_back, const ActivityMap_Utils* am, const XnPoint3D* p3D, const int nP, const XnPoint3D* points2D)
{
	for (int i = 0; i < nP; i++)
	{
		Point p2D = ActivityMap_Utils::findMoACoordinate(&p3D[i], MAX_RANGE);

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



void createDepthMatrix(const XnDepthPixel* dMap, Mat& depthMat)
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

}

void convert16to8(const Mat* src, Mat& out)
{
	double max = 0;
	minMaxIdx(*src, NULL, &max);
	if (max != 0)
	{
		src->convertTo(out, CV_8UC1, 255/max);
			
		subtract(cv::Scalar::all(255),out, out);

	}
	else
	{
		Utils::initMat1u(out, 255);
	}
}

float distf(Point p1, Point p2)
{
	return sqrt(powf(p2.x-p1.x,2)+powf(p2.y-p1.y,2));
}

/*
p = Point in the remap polar space
out = Correspoding point in the Polar coordinate space
*/
Point remap2Polar(const Point* p)
{
	Point out;
	//Calculate range
	float rangeAlt = (RANGE_ROWS - p->y)*RANGE_STEP;
	out.y = (-3.83328*(0.869577-exp( (3.0303*rangeAlt)/SCALE_RANGE_ALT) ) )*1000;
	out.x = p->x;
	return out;

}

/*
Convert a point from the remap polar space rep. to the MoA (3D)
*/
Point convertBack(const Point* p)
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
Convert a point from the remap polar space rep. to the polar space rep.
*/
Point convertBack_Polar(const Point* p)
{
	Point out;
	//Calculate range
	float rangeAlt = (RANGE_ROWS - p->y)*RANGE_STEP;
	float range = (-3.83328*(0.869577-exp( (3.0303*rangeAlt)/SCALE_RANGE_ALT) ) )*1000;
			
	out.y = RANGE_ROWS - ((int)range/RANGE_STEP);
	out.x = p->x;

	return out;
}



void addGrid(Mat& img, Size sz)
{

	for (int c = 0; c <= img.cols-sz.width; c += sz.width)
		line(img, Point(c, 0), Point(c, img.rows), Scalar::all(200));

	for (int r = 0; r < img.rows-sz.height; r += sz.height)
		line(img, Point(0, r), Point(img.cols, r), Scalar::all(200));

}


bool isMaximum(const Mat* img, Rect k)
{
	int found = false;
	int val = img->ptr<ushort>(k.y + k.height/2)[k.x + k.width/2];
	int i = k.y;
	int maxI = k.y + k.height;
	int maxJ = k.x + k.width;
	ushort* img_data = (ushort*)img->data;
	int img_step = img->step/sizeof(ushort);

	while (i < maxI && !found)
	{		
		const ushort* ptrImg = img_data + i*img_step;
		while (int j = k.x < maxJ && !found)
		{
			found = ptrImg[j] > val;
			j++;
		}
		i++;
	}
	return (!found);
}


void findBlobs(const cv::Mat &binary, std::vector < std::vector<cv::Point2i> > &blobs)
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
Normalize the histogram of the heights 0-1
*/
void normalizeHeightModel(float* heightHist)
{
	float max = 0;
	for (int i = 0; i < MODEL_NBINS; i++)
	{
		float v = heightHist[i];
		if (v > max)
			max = v;
	}

	for (int i = 0; i < MODEL_NBINS; i++)
		heightHist[i] = heightHist[i]/max;

}


/*
Initialize the values of the bins in the height and colour model
*/
void initPerson(Person* p)
{
	//first iteration the measruemente is trusted 100%
	float valsR[] = {powf(meas_std,2),0,0,powf(meas_std,2)};
	Mat m = Mat(2,2, CV_32F, valsR);
	m.copyTo(p->R);
	//Covariance of the prediction error
	//first iteration the prediction model is trusted 0%
	float valsQ[] = {1.,0,0,0,   0,1.,0,0,  0,0,1.,0,  0,0,0,1.};
	m = Mat(4,4, CV_32F, valsQ);
	m = m*proc_std;
	m.copyTo(p->Q);

	//Prediction linear model
	float valsA[] = {1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1};
	m = Mat(4,4, CV_32F, valsA);
	m.copyTo(p->A);

	p->stateMoA = Mat::zeros(4,1,CV_32F);
	p->covMoA = Mat::zeros(4,4, CV_32F);
	p->R.copyTo(p->covMoA(Rect(0,0, 2,2)));
	p->Q(Rect(2,2,2,2)).copyTo(p->covMoA(Rect(2,2,2,2)));

	p->control = DTC_FULL;
	p->lost = 0;
	p->associated = false;

	for (int i = 0; i < MODEL_NBINS; i++)
	{
		p->heightModel[i] = 0;
		p->colourModel[i].cov = Mat::zeros(3,3, CV_32F);
		p->colourModel[i].totalColours = 0;
		p->colourModel[i].colours = new const XnRGB24Pixel*[MAX_POINTS_BIN];
	}

	
	if (debug == DEBUG_HIGH)
	{
		printValuesF(&p->R, "R(init)", outDebugFile);
		printValuesF(&p->Q, "Q(init)", cout);
		printValuesF(&p->A, "A(init)", cout);
		printValuesF(&p->covMoA, "CovMoa(init)", cout);
	}
}


//void createModel(int rpX, int rpY, const PointMapping* pntsMap, int ttlPnts, Person* p)
//{
//	for (int iterP = 0; iterP < ttlPnts; iterP++)
//	{
//		const PointMapping* pMap = pntsMap + iterP;
//		if (pMap->rpX == rpX && pMap->rpY == rpY)
//		{
//			float height = pMap->p3D->Y;
//			const XnRGB24Pixel* colour = pMap->colour;
//			if (debug == DEBUG_HIGH && frames == debugFrame)
//				outHeights << height << endl;
//
//			if (height < MODEL_MAX_HEIGHT && height > MODEL_MIN_HEIGHT)
//			{
//				int bin = (MODEL_MAX_HEIGHT - height)/MODEL_BINRANGE;
//								
//				Scalar* modelBinColour = p->colourModel + bin;
//
//				int numBinPnts = p->heightModel[bin];
//								
//				//update the model (height, colour)
//				if (numBinPnts > 0)	
//				{
//
//					(*modelBinColour)(0) = (colour->nRed + numBinPnts * (*modelBinColour)(0))/(numBinPnts+1);
//					(*modelBinColour)(1) = (colour->nGreen + numBinPnts * (*modelBinColour)(1))/(numBinPnts+1);
//					(*modelBinColour)(2) = (colour->nBlue + numBinPnts * (*modelBinColour)(2))/(numBinPnts+1);
//
//				}
//				else
//					p->colourModel[bin] = Scalar(colour->nRed, colour->nGreen, colour->nBlue);
//
//				p->heightModel[bin]++;
//				if (debug == DEBUG_HIGH && frames == debugFrame && p->id == 0)
//					outPersModel << height << " " << (int)colour->nRed << " " << (int)colour->nGreen << " " << (int)colour->nBlue << endl;
//				
//
//			}
//			else
//			{
//				outDebugFile << "DetectCC: Person point out of height range: Info: RPS coordinates: " << rpX << ", " << rpY <<
//					". Height: " << height << ". MAX_HEIGHT: " << MODEL_MAX_HEIGHT << ". MIN_HEIGHT: " << MODEL_MIN_HEIGHT << ". Colour: " 
//					<< colour->nRed << ", " << colour->nGreen << ", " << colour->nBlue << endl;
//			}
//		}
//	}
//}


void projectLocation2MoA(Person* prs)
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
	//covCartessian.copyTo(prs->covMoA(Rect(0,0,2,2)));

	//DEBUG TESTING
	/*prs->covMoA.at<float>(0, 0) = 10;
	prs->covMoA.at<float>(0,1) = 0;
	prs->covMoA.at<float>(1,0) = 0;
	prs->covMoA.at<float>(1,1) = 10;*/


	//define the gate region 2x2
	//Mat spaceCov = prs->covMoA(Rect(0,0, 2,2))*4; //BAD: IAM NOT CONSIDERING THE VARIANCE = STD^2
	//if (debug > DEBUG_MED)
	//{
	//	if (frames == debugFrame)
	//	{
	//		printValuesF(&prs->covMoA, "CovMoa(I)", outDebugFile);
	//		printValuesF(&spaceCov, "Gate area", outDebugFile);
	//		printValuesF(&covCartessian, "Cov real", outDebugFile);
	//	}
	//	assert(spaceCov.rows, 2);
	//	assert(spaceCov.cols, 2);
	//	for (int i = 0; i < spaceCov.rows; i++)
	//	{
	//		for (int j = 0; j < spaceCov.cols; j++)
	//		{
	//			assert(spaceCov.at<float>(i,j), 4*prs->covMoA.at<float>(i,j));	
	//			assert(covCartessian.at<float>(i,j) , prs->covMoA.at<float>(i,j));
	//		}
	//	}
	//}
	//usign 2 standard deviations. Probability of 95%
	//spaceCov.copyTo(prs->gtArea);


	//if (debug > DEBUG_NONE)
	//{
	//	int nRows = prs->covMoA.rows;
	//	int nCols = prs->covMoA.cols;
	//	int covStep = prs->covMoA.step/sizeof(float);
	//	float* covMoa_data = (float*)prs->covMoA.data;
	//	outDebugFile << "Covariance matrix (I)" << endl;
	//	for (int i = 0; i < nRows; i++)
	//	{
	//		float* ptr = covMoa_data + (i*covStep);
	//		for (int j = 0; j < nCols; j++)
	//		{
	//			outDebugFile << ptr[j] << " ";
	//		}
	//		outDebugFile << endl;
	//	}
	//}

	//prs->covMoA = covCartessian;
	//prs->sigmaX_MoA = sqrtf(covCartessian.at<float>(0,0));
	//prs->sigmaY_MoA = sqrtf(covCartessian.at<float>(1,1));
}


/*
Perform a connected component labeling on the remap polar coordinate space.

bw: Mat(500,181, CV_16UC1);
*/
//void detectCC(Mat& bw, Person** dtctPpl, int& ttl_dtctPpl, const PointMapping* pntsMap, int ttlPnts, Mat& debugImg)
//{
//
//	Mat imgCpy = Mat(bw.size(), CV_8UC1);
//	Mat cpy = Mat(bw.size(), CV_8UC1);
//	convert16to8(&bw, imgCpy); //convert the image to a range between 0-255. And also inverts the values, so 0 belongs to high values in viceversa
//
//	//Create a binary image
//	threshold(imgCpy, cpy, 254, 1, THRESH_BINARY); //backround = 1, foreground = 0;
//
//	//find connected components
//	std::vector < std::vector<cv::Point2i > > blobs;
//	findBlobs(cpy, blobs);
//
//	//for each region calculate mean and covariance
//	vector<vector<Point2i>>::iterator iter = blobs.begin();
//	float x, y, xx, yy, ww;
//	float range,alpha, rr, aa;
//
//	ushort* bw_data =(ushort*) bw.data;
//	int bw_step = bw.step/sizeof(ushort);
//	int bw_cols = bw.cols;
//
//	int idPers = 0;
//	while (iter != blobs.end())
//	{
//		bool pass = false;
//		vector<Point2i> b = *iter;
//		//if (b.size() > 10) //rejection of small regions
//		{
//			x = y = xx = yy = ww = 0;
//			int maxI = b.size();
//			for (int i = 0; i < maxI; i++)
//			{
//				Point2i p = b[i];
//					
//				float rangeTmp = (bw.rows-p.y)*RANGE_STEP; //binSize
//				float thresh;
//				if (rangeTmp > 4000)
//					//thresh= 67900*exp((-0.00044)*rangeTmp); //low threshold
//					thresh = -0.6*rangeTmp+7000;
//					//thresh= 55000*exp((-0.00048)*rangeTmp); //low threshold
//				else
//					thresh = -5*rangeTmp+25000;
//				//float thresh = 68103*exp((-0.000446)*rangeTmp); //low threshold
//	
//				int w = (bw_data + p.y*bw_step)[p.x];
//				if (w > thresh) //check that there is at least one pixel that get the higher threshold
//						pass = true;
//
//				x += (p.x*w);
//				y += (p.y*w);
//				ww += w;
//			}
//			if (pass)
//			{
//				Person* prs = new Person();
//				initPerson(prs);
//				prs->mean_RPS = Point(x/ww, y/ww);
//				prs->id = idPers++;
//				for (int i = 0; i < maxI; i++)
//				{
//					int rpX = b[i].x;
//					int rpY = b[i].y;
//					//debug 
//					if (debug > DEBUG_MED)//ONLY points that surpass the higher threshold are drawn in yellow
//					{
//						debugImg.ptr<uchar>(rpY)[rpX*3] = 0;
//						debugImg.ptr<uchar>(rpY)[rpX*3+1] = 255;
//						debugImg.ptr<uchar>(rpY)[rpX*3+2] = 255;
//					}
//										int w = (bw_data + rpY*bw_step)[rpX];				
//					xx += w*powf(rpX-prs->mean_RPS.x,2);
//					yy += w*powf(rpY-prs->mean_RPS.y,2);
//
//					//UPDDATE THE MODEL
//					createModel(rpX, rpY, pntsMap, ttlPnts, prs);
//					//look for points in pntsMap with point coordinates
//
//				}
//				prs->sigmaY_RPS = sqrtf(yy/ww);
//				prs->sigmaX_RPS = sqrtf(xx/ww);
//
//				projectLocation2MoA(prs);
//
//				//normalizeHeightModel(prs.heightModel);
//				dtctPpl[ttl_dtctPpl++] = prs;
//				if (frames == debugFrame && prs->id == 0)
//				{
//					outDebugFile << "Colour model at frame " << frames << endl;
//					for (int k = 0; k < MODEL_NBINS; k++)
//					{
//						outDebugFile << (float)prs->heightModel[k] << " " << (int)prs->colourModel[k](0) << " " << (int)prs->colourModel[k](1) << " " << (int)prs->colourModel[k](2) << endl;
//					}
//				}
//			}
//		}
//		iter++;
//	}
//}


bool belong(const Person* p, int x, int y)
{
	float val = powf((x-p->mean_RPS.x),2)/powf(p->sigmaX_RPS,2) + powf((y - p->mean_RPS.y),2)/powf(p->sigmaY_RPS,2);
	return val <= 1;
}


void buildColourCovariance(Person* p)
{
	int rows =  p->colourModel[0].cov.rows;
	int cols = p->colourModel[0].cov.cols;
	for (int i = 0; i < MODEL_NBINS; i++)
	{
		float rr, gg, bb, rg, rb, bg;
		rr = gg = bb = rg = rb = bg = 0;
		gaussianParam* colourModel = &p->colourModel[i];
		Scalar meanC = colourModel->mean;
		int totalCol = colourModel->totalColours;
		float meanR = meanC(0);
		float meanG = meanC(1);
		float meanB = meanC(2);

		if (debug == DEBUG_HIGH)
		{
			cout << "Build colour covariance" << endl;
			cout << "Bin " << i << " Mean Colour: " << meanC(0) << ", " << meanC(1) << ", " << meanC(2) << ". TotalColours: " << totalCol << endl;
		}



		if (totalCol > 0)
		{
			for (int iter = 0; iter < totalCol; iter++)
			{
				const XnRGB24Pixel* clr = colourModel->colours[iter];
				float subR = (float)clr->nRed - meanR;
				float subG = (float)clr->nGreen - meanG;
				float subB = (float)clr->nBlue - meanB;

				if (debug == DEBUG_HIGH && i == 2 && frames == 96 && p->id == 1)
				{
					outClrPnts << (float)clr->nRed << " " << (float)clr->nGreen << " " << (float)clr->nBlue << endl;
				}

				rr += powf(subR, 2);
				gg += powf(subG, 2);
				bb += powf(subB, 2);
				rg += (subR)*(subG);
				rb += subR*subB;
				bg += subB*subG;
			}
			//TODO: improve performance
			Mat* cov = &p->colourModel[i].cov;
			cov->at<float>(0,0) = rr/totalCol;
			cov->at<float>(0,1) = rg/totalCol;
			cov->at<float>(0,2) = rb/totalCol;
			cov->at<float>(1,0) = rg/totalCol;
			cov->at<float>(1,1) = gg/totalCol;
			cov->at<float>(1,2) = bg/totalCol;
			cov->at<float>(2,0) = rb/totalCol;
			cov->at<float>(2,1) = bg/totalCol;
			cov->at<float>(2,2) = bb/totalCol;
			//once the covariance matrix has been built,we can free the memoriy of the array
		}
		delete []colourModel->colours;
		
	}
}

void buildAppearanceModels(Person* dtctPpl, int ttl_dtctPpl, const PointMapping* pntsMap, int ttlPnts)
{
	for (int i = 0; i < ttlPnts; i++)
	{
		PointMapping pntMap = pntsMap[i];
		bool found = false;
		int c = 0;
		Person* prs = NULL;
		while (c < ttl_dtctPpl && !found)
		{
			prs = &dtctPpl[c];
			found = belong(prs, pntMap.rpX, pntMap.rpY);
			c++;
		}
		if (found && prs != NULL)
		{
			//update the appearance model
			float height = pntMap.p3D->Y;
			const XnRGB24Pixel* colour = pntMap.colour;
			if (debug > DEBUG_NONE && frames == debugFrame)
				outHeights << height << endl;

			if (height < MODEL_MAX_HEIGHT && height > MODEL_MIN_HEIGHT)
			{
				int bin = (MODEL_MAX_HEIGHT - height)/MODEL_BINRANGE;
								
				gaussianParam* modelBinColour = prs->colourModel + bin;

				if (modelBinColour->totalColours == MAX_POINTS_BIN)
				{
					outDebugFile << "BuildAppearanceModel:ERROR Reached maximum number of points per bin. Info: BinId: " << bin <<" TotalPoints: " << modelBinColour-> totalColours << 
						"Max_Points_bin: " << MAX_POINTS_BIN << endl;
				}

				modelBinColour->colours[modelBinColour->totalColours++] = colour;

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

				prs->heightModel[bin]++;
				if (debug >= DEBUG_MED && frames == 96 && prs->id == 1)
				{
					outPersModel << height << " " << (int)colour->nRed << " " << (int)colour->nGreen << " " << (int)colour->nBlue << endl;
				}

			}
			else
			{
				outDebugFile << "DetectCC: Person point out of height range: Info: RPS coordinates: " << pntMap.rpX << ", " << pntMap.rpY <<
					". Height: " << height << ". MAX_HEIGHT: " << MODEL_MAX_HEIGHT << ". MIN_HEIGHT: " << MODEL_MIN_HEIGHT << ". Colour: " 
					<< (int)colour->nRed << ", " << (int)colour->nGreen << ", " << (int)colour->nBlue << endl;
			}
		}
	}
	//Create covariance matrix for all detections
	for (int i = 0; i < ttl_dtctPpl; i++)
	{
		buildColourCovariance(&dtctPpl[i]);
	}
}

void detectCC(Mat& bw, Person* dtctPpl, int& ttl_dtctPpl, const PointMapping* pntsMap, int ttlPnts, Mat& debugImg)
{

	Mat imgCpy = Mat(bw.size(), CV_8UC1);
	Mat cpy = Mat(bw.size(), CV_8UC1);
	convert16to8(&bw, imgCpy); //convert the image to a range between 0-255. And also inverts the values, so 0 belongs to high values in viceversa

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

				//if (rangeTmp > 4000)
				//	//thresh= 67900*exp((-0.00044)*rangeTmp); //low threshold
				//	thresh = -0.6*rangeTmp+7000;
				//	//thresh= 55000*exp((-0.00048)*rangeTmp); //low threshold
				//else
				//	thresh = -5*rangeTmp+25000;
				//float thresh = 68103*exp((-0.000446)*rangeTmp); //low threshold
	
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
					if (debug >= DEBUG_MED)//ONLY points that surpass the higher threshold are drawn in yellow
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

				if (debug >= DEBUG_MED)
				{
					ellipse(debugImg, prs->mean_RPS, Size(prs->sigmaX_RPS, prs->sigmaY_RPS), 0, 0, 360, Scalar::all(0), 2);
				}

				projectLocation2MoA(prs);

		/*		if (frames == debugFrame && prs->id == 0)
				{
					outDebugFile << "Colour model at frame " << frames << endl;
					for (int k = 0; k < MODEL_NBINS; k++)
					{
						outDebugFile << (float)prs->heightModel[k] << " " << (int)prs->colourModel[k].mean(0) << " " << (int)prs->colourModel[k].mean(1) << 
							" " << (int)prs->colourModel[k].mean(2) << endl;
					}
				}*/
			}
		}
		iter++;
	}

	//create appearance model
	buildAppearanceModels(dtctPpl, ttl_dtctPpl,  pntsMap,  ttlPnts);

}





/*
img : Mat(500,181, CV_16UC1)

*/
void ccDetection(Mat& img, Person* dtctPpl, int& ttl_dtctPpl, const PointMapping* pntsMap, int& ttlPnts)
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
	detectCC(img, dtctPpl, ttl_dtctPpl, pntsMap, ttlPnts, outDebug);
	if (debug >= DEBUG_MED)
	{
		//Show the change points in the thresholds
		int rowLow = RANGE_ROWS - ((int)3750/RANGE_STEP);	
		line(outDebug, Point(0,rowLow), Point(RANGE_COLS, rowLow), Scalar(0,0,255));
		imshow("Thresholds", outDebug);	
	}
}

Point findNearestPoint(Point pnt, Person* ppl, int ttlPpl)
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

void displayDetections(Person* trckPpl, int ttl_trckPpl, Mat& remapPolar, Mat& moa)
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


/*
The detected people is classified based on the area of their bounding boxes 
*/
void look4MergeSplits(Person* dtctPpl, int ttl_dtctPpl)
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
void predictState(Person*& p)
{
	p->stateMoA = p->A * p->stateMoA;

	if (debug > DEBUG_MED)
	{
		outDebugFile << "PREDICT STATE (COVARIANCE UPDATE)" << endl;
		printValuesF(&p->covMoA, "Covariance matrix (before prediction)", cout);
		printValuesF(&p->A, "Prediction model (A)", outDebugFile);
		printValuesF(&p->Q, "Error prediction cov (Q)", outDebugFile);
		Mat tmp = p->A * p->covMoA * p->A.t();
		printValuesF(&tmp, "(APA')", outDebugFile);
	}

	//CovMoA is the covariance projected from the detection
	p->covMoA = p->Q + p->A * p->covMoA * p->A.t();

	if (debug > DEBUG_MED)
	{
		
		printValuesF(&p->covMoA, "Covariance matrix (after prediction)", cout);
		outDebugFile << "-----------------------------------" << endl;
	}
}


///*
//out = (dPrs.mean - target.mean) * inv((target.gtCov + dPrs.covMoa)/2) * (dPrs.mean - target.mean)T
//*/
//float mahalanobis(const Person* target, const Person* dPrs)
//{
//	//Mean diff
//	Mat tgtMean = target->stateMoA(Rect(0,0, 1,2));
//	Mat dtcMean = dPrs->stateMoA(Rect(0,0,1,2));
//	Mat meanDiff = dtcMean-tgtMean;
//
//	//sum covariances
//	Mat tgtCov = target->covMoA(Rect(0,0,2,2));
//	Mat dtcCov = dPrs->covMoA(Rect(0,0,2,2));
//	Mat avgCov = (tgtCov + dtcCov)/2;
//
//	Mat out = meanDiff.t() * avgCov.inv() * meanDiff;
//
//	return out.at<float>(0,0);
//
//}


/*
out = (dPrs.mean - target.mean) * inv((target.gtCov + dPrs.covMoa)/2) * (dPrs.mean - target.mean)T
*/
float mahalanobis(const Person* target, const Person* dPrs)
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

/*
Define an area around the predicted position of the target, and filter the detections whithin the region
*/
void gateDetection(const Person* target,  Person* dtctPpl, int ttl_dtctPpl, Person** gPpl, int& ttl_gPpl)
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
Normalize histograms using only bins whith data in both models
*/
void normalizeModels(Person& p1, Person& p2)
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


/*
Out: true If the three componentes (RGB) of both colours belong to the same bins (in each channel respectively)
*/
bool compareColours(Scalar c1, Scalar c2)
{
	bool out = true;
	int i = 0;
	while (out && i < 3)
	{
		int bin1 = c1(i)/32; //bins of 32
		int bin2 = c2(i)/32;
		out = (bin1 == bin2);
		i++;
	}
	return out;
}


float bhattacharyaDist(const gaussianParam* trgt, const gaussianParam* cndt)
{
	//dist = 1/8 (m - m')T * Sigma^-1 * (m-m') + 1/2 *ln( det(Sigma) / (det(Sigma1)*det(Sigma2) )

	//Means subtraction
	Mat meanSub = Mat(3,1, CV_32F);
	meanSub.at<float>(0) = trgt->mean(0) - cndt->mean(0);
	meanSub.at<float>(1) = trgt->mean(1) - cndt->mean(1);
	meanSub.at<float>(2) = trgt->mean(2) - cndt->mean(2);

	//Pooled sigma
	Mat Cov = (trgt->cov + cndt->cov)/2;
	Mat CovInv = Cov.inv();
	if (debug == DEBUG_MED && frames == 79)
	{
		printValuesF(&trgt->cov, "Covariance target", outDebugFile);
		printValuesF(&cndt->cov, "Covariance CAndidate", outDebugFile);
		printValuesF(&Cov, "pooled Cov", outDebugFile);
		printValuesF(&CovInv, "pooled Cov inv", outDebugFile);
	}
	 
	
	Mat mahalanobisDist = meanSub.t() * CovInv * meanSub;
	float mDist = mahalanobisDist.at<float>(0);
	float term1 = mDist/8;

	float detCov = determinant(Cov);
	float detTrgtCov = determinant(trgt->cov);
	float detCndtCov = determinant(cndt->cov);

	if (debug == DEBUG_MED && frames == 79)
	{
		printValuesF(&trgt->cov, "Covariance target", outDebugFile);
		printValuesF(&cndt->cov, "Covariance CAndidate", outDebugFile);
		printValuesF(&Cov, "pooled Cov", outDebugFile);
		printValuesF(&(Mat)Cov.inv(), "pooled Cov inv", outDebugFile);
		outDebugFile << "Det Pooled cov: " << detCov << endl;
		outDebugFile << "Det trgt cov: " << detTrgtCov << endl;
		outDebugFile << "Det cndt cov: " << detCndtCov << endl;
	}

	if (detCov != 0 && detTrgtCov != 0 && detCndtCov != 0)
	{
		float term_21 = detCov/ ( sqrtf(detTrgtCov)*sqrtf(detCndtCov) );
		float term2 = log(term_21)/2;
		float out = term1 + term2;
		outDebugFile << out << endl;
		return out;
	}
	else
	{
		outDebugFile << "CompareColours. Error Determinant = 0. Info: Pooled Sigma det: " << detCov << ". Trgt Sigma det: " << detTrgtCov <<
			". Cndt Sigma det: " << detCndtCov << endl;
		return -1;
	}
}

float compareColours(const gaussianParam* trgt, const gaussianParam* cndt)
{
	printValuesF(&trgt->cov, "Covariance target", outDebugFile);
	printValuesF(&cndt->cov, "Covariance CAndidate", outDebugFile);
	printValuesF(&(Mat)trgt->cov.inv(), "Inverse Covariance target", outDebugFile);
	printValuesF(&(Mat)cndt->cov.inv(), "Inverse Covariance Candidate", outDebugFile);

	Mat invCov = trgt->cov.inv() + cndt->cov.inv();
	Mat Cov = trgt->cov + cndt->cov;

	printValuesF(&invCov, "Sum inverses covs", outDebugFile);
	printValuesF(&Cov, "Sum covs", outDebugFile);

	outDebugFile << "Target mean colour: " <<trgt->mean(0) << ", " << trgt->mean(1) << ", " << trgt->mean(2) << endl;
	outDebugFile << "Candidate mean colour: " <<cndt->mean(0) << ", " << cndt->mean(1) << ", " << cndt->mean(2) << endl;

	Mat meanSub = Mat(3,1, CV_32F);
	meanSub.at<float>(0) = trgt->mean(0) - cndt->mean(0);
	meanSub.at<float>(1) = trgt->mean(1) - cndt->mean(1);
	meanSub.at<float>(2) = trgt->mean(2) - cndt->mean(2);

	printValuesF(&meanSub, "Means subtraction" , outDebugFile);

	Mat  cc = (meanSub.t())*invCov*(meanSub);

	printValuesF(&cc, "expoinent Term " , outDebugFile);
	assert(cc.rows == 1);
	assert(cc.cols == 1);
	float c = (float)*cc.data; //itis a 1x1 marix
	outDebugFile << "Exponent term (II) " << c << endl;
	
	float norm = (2*CV_PI* sqrtf(determinant(Cov)));

	outDebugFile << "Normalization Term: " << norm << endl;

	float tmp = exp(-c/2);
	outDebugFile << "exp(-c/2): " << tmp << endl;

	if (norm == 0)
		outDebugFile << "CompareColours. Error: Division by zero. "<< endl;
	
	float pdf = (tmp)/norm;

	outDebugFile << "Final pdf: " << pdf << endl;
	return pdf;
}
float bhattacharyaDist_Cont(const gaussianParam* trgt, const gaussianParam* cndt)
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
		printValuesF(&trgt->cov, "Covariance target", outDebugFile);
		printValuesF(&cndt->cov, "Covariance CAndidate", outDebugFile);
		printValuesF(&S1, "pooled Cov(strgt+scndt)", outDebugFile);
		outDebugFile << "Det(Strg + Scndt): " << S1det << endl;
		printValuesF(&S2_Inv, "pooled Cov inv", outDebugFile);
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


void printPerson(const Person* p)
{
	outDebugFile << "********** Track id: " << p->id << endl;
	//printValuesF(&p->stateMoA, "State MoA", outDebugFile);
	//printValuesF(&p->covMoA, "Cov MoA", outDebugFile);
//	printValuesF(&p->gtArea, "Gating area" , outDebugFile);
	//printValuesF(&p->A , "Prediction model (A)", outDebugFile);
	//printValuesF(&p->Q, "Prediction error covariance (Q)", outDebugFile);
	//printValuesF(&p->R, "Measurement error covariance (R)", outDebugFile);
	//outDebugFile << "Lost: " << p->lost << ". Lost threshold: " << TRACKLOST_THRESHOLD << endl;
	for (int i = 0; i < MODEL_NBINS; i++)
	{
		outDebugFile << "Bin " << i << endl;
		outDebugFile << "Total Number Colours: " << p->colourModel[i].totalColours << endl;
		outDebugFile << "Mean Colour: " << p->colourModel[i].mean(0) << ", " << p->colourModel[i].mean(1) << ", " << p->colourModel[i].mean(2) << endl;
		printValuesF(&p->colourModel[i].cov, "Covariance Colour", outDebugFile);
	}

	outDebugFile << "*****************************************" << endl;
}



/*
A detected person is compared with the target using the appearance model
*/
float compareModels(const Person* target, const Person* candidate)
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
			float colourBinD = bhattacharyaDist_Cont(&colourT, &colourC); 

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
Look the closer detection with the target (location and appearance).
1- Defines a gating area (filter out detections far from the predicted location)
2- Compare appearance models

IN:
Target, dtectPpl, ttl_dtctPpl

OUT:
candidate, compCoeff (it will be used later to decide if the appearance update is performed or not.
*/
void association(const Person* target,  Person* dtctPpl, int ttl_dtctPpl, Person*& candidate, float& compCoeff)
{
	//Filter all detections within a gate region
	Person** gPpl = new Person*[ttl_dtctPpl];
	int ttl_gPpl = 0;
	gateDetection(target, dtctPpl, ttl_dtctPpl, gPpl, ttl_gPpl);
	
	for (int i = 0; i < ttl_gPpl; i++)
	{
		Person* v = gPpl[i];
		float c = compareModels(target, v);

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
void updateState(Person* trgt, const Person* msr, float dComp)
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
		printValuesF(&I, "Identity matrix", outDebugFile);
		printValuesF(&K, "K", outDebugFile);
		printValuesF(&H, "H", outDebugFile);
		printValuesF(&trgt->covMoA, "Covariance Matrix", outDebugFile); 
		printValuesF(&trgt->covMoA, "Covariance Matrix (Updated)", outDebugFile); 
	}

}


/*
Updates the model considereing the different bins
*/
void updateModel(Person* target, const Person* msr)
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
void checkEnd_NewTracks(Person*& trckPpl, int& ttl_trckPpl, Person*& dtctPpl, int& ttl_dtctPpl)
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

void drawPersonMean_debug(const Person& p, Mat* moa, Scalar color)
{
	Point pntMean = Point(p.stateMoA.at<float>(0,0),p.stateMoA.at<float>(1,0));

	cv::circle(*moa, pntMean, 2, color, -1);	
}


void drawPersonCov_debug(const Person& p, Mat* moa, Scalar color)
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

void tracking(Person*& trckPpl, int& ttl_trckPpl, Person*& dtctPpl, int& ttl_dtctPpl, Mat* moa)
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
			predictState(target);

			//debug draw the predict position and uncertainty
			if (debug == DEBUG_MED)
			{
				drawPersonCov_debug(*target, moa,Scalar(127,127,127));
				printValuesF(&target->covMoA, "CovMat", cout);
				//imshow(windMoA, *moa);
				//waitKey(0);
			}

			//Find the measurement generated by the target
			Person* measur = NULL;
			float compDist = -1;
			if (frames == 96 && target->id == 1)
				cout << "Stop dude" << endl;
			association(target, dtctPpl, ttl_dtctPpl, measur, compDist);

			if (measur != NULL)
			{
				measur->associated = true;
				//Update the position of the target using the Kalman equations.
				updateState(target, measur, compDist);

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



void copyPerson(Person& dst, const Person* src)
{
	src->stateMoA.copyTo(dst.stateMoA);
}


/*
Arg 1: 0:Video; 1:live
Arg 2: 0:No Record; 1:Record
*/
int main(int argc, char* argv[])
{
	
	bool saved = false;
	int fromVideo = 1;
	int recordOut = 0;
	int tilt = 0;
	tiltTXT >> tilt;
	cout << "Start" << endl;
	if (argc ==4)
	{
		sscanf(argv[1], "%d", &fromVideo);
		sscanf(argv[2], "%d", &recordOut);
		sscanf(argv[3], "%d", &debug);
	}

	cout << "Tilt: " << tilt;
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

	char* paths[3];
	paths[0] = "d:/Emilio/Tracking/DataSet/kinect0_calib.oni";
	paths[1] = "d:/Emilio/Tracking/DataSet/kinect1_calib.oni";
	paths[2] = "d:/Emilio/Tracking/DataSet/kinect2_calib.oni";

	/*paths[0] = "d:/Emilio/Tracking/DataSet/Dset2_workshop/kinect0_calib.oni";
	paths[1] = "d:/Emilio/Tracking/DataSet/Dset2_workshop/kinect1_calib.oni";
	paths[2] = "d:/Emilio/Tracking/DataSet/Dset2_workshop/kinect2_calib.oni";*/

	//Initialize resolutions MoA and Remap Polar space
	ActivityMap_Utils actMapCreator(DEPTH_SCALE, NUM_SENSORS);
	//float max_z_mt = ActivityMap_Utils::MAX_Z/1000.0;
	//float rangeVar = 2.6206*powf(max_z_mt,2)+ 0.6820*max_z_mt -0.2109; //Range at maximum depth in the remap polar space
	//MAX_RANGE = ActivityMap_Utils::MAX_Z/(cosf(KinectSensor::KINECT_HORIZ_FOV/2)) + rangeVar + KINECTS_DISPLACEMENT; 


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
		//VideoWriter	w("d:/Emilio/Tracking/DataSet/MoA2.mpg",CV_FOURCC('P','I','M','1'), 20.0, actMapCreator.getResolution(), true);
		//Size sz = actMapCreator.getResolution();
	//	Size sz = Size(181, 500);
		w.open("c:/Dropbox/Phd/Individual Studies/KinectDepthSensor/AlternativeSpace/RemapPolarSpace_Detection.mpg",CV_FOURCC('P','I','M','1'), 20.0, polarAlt_smooth_.size(), true);
		w1.open("c:/Dropbox/Phd/Individual Studies/KinectDepthSensor/AlternativeSpace/MoA_Detection.mpg",CV_FOURCC('P','I','M','1'), 20.0, actMapCreator.getResolution(), true);
	}	

	//namedWindow(windPolarName);
	cvSetMouseCallback(windPolarName, selectROI_callBack, (Mat*)&polar_);
	cvSetMouseCallback(windPolarName, selectPoint_callBack, (Mat*)&polar_);

	//Size of kernel: smooth rps;
	Mat kernel = Mat::ones(Size(5,27), CV_32F);

	//list<Person> people;
	//Person* dtctPpl = new Person[MAX_PEOPLE];
	//int ttl_dtctPpl = 0;
	Person* trckPpl = new Person[MAX_PEOPLE];
	int ttl_trckPpl = 0;
	//for detection to estimate the orientation of the ellipse
	//Person* pastPpl = new Person[MAX_PEOPLE];
	//int ttlPastPpl = 0;



	clock_t startTime = clock();
	clock_t startTotalTime = clock();
	clock_t startTime_tmp;
	int nPoints = 0;
	while (!bShouldStop)
	{		
		//cout << "Frames: " << frames << endl;

		if (frames == 5) 
			bgComplete = true;
		if (debug >= DEBUG_MED)
		{
			cout << "Frames " << frames << endl;
			outDebugFile << "Frame " << frames << endl;
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
				if (debug >= DEBUG_MED)//Draw the output of the foreground detection
				{
					if (frames == 96 && i == 1)
						imwrite("c:/Dropbox/Phd/Matlab/Model/rgb_1_Orig.jpg", rgbImages[1]);
					for (int c = 0; c < numberOfForegroundPoints[i]; c++)
					{
						XnPoint3D* p = &(pointsFore2D[i][c]); 
						uchar* ptr = rgbImages[i].ptr<uchar>(p->Y);
						ptr[3*(int)p->X] = 0;
						ptr[(3*(int)p->X)+1] = 0;
						ptr[(3*(int)p->X)+2] = 255;
					}
					if (frames == 96 && i == 1)
						imwrite("c:/Dropbox/Phd/Matlab/Model/rgb_1_Red.jpg", rgbImages[1]);
				}
			}
			if (nPoints > 0)
			{
				Person* dtctPpl = new Person[MAX_PEOPLE];
				int ttl_dtctPpl = 0;

				//list of 3D points + colour map to the RP space
				PointMapping* pntsMap = new PointMapping[nPoints];
				int ttlPnts = 0;

				for (int i = 0; i < NUM_SENSORS; i++)
				{
					startTime_tmp = clock();
					points3D[i] = new XnPoint3D[numberOfForegroundPoints[i]];
					//points3D[i] = kinects[i].arrayBackProject(pointsFore2D[i], numberOfForegroundPoints[i]);
					kinects[i].arrayBackProject(pointsFore2D[i], points3D[i], numberOfForegroundPoints[i]);
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
					updatePolarAlternateive(&polarAlt, &polar, pntsMap, ttlPnts, points3D[i], pointsFore2D[i], rgbMaps[i], numberOfForegroundPoints[i]);	
					totalIntervals[RPSPACE_ID] += clock() - startTime_tmp; //time debugging
					
					//startTime_tmp = clock();
					//updateActivityMap(*activityMap, *activityMap_Back, &actMapCreator, points3D[i], numberOfForegroundPoints[i], pointsFore2D[i]);
					//totalIntervals[MOA_ID] += clock() - startTime_tmp; //time debugging
					//delete []points3D[i];
				}
		
				//Todo: Create a method detection(polarAlt, moAPeople)
				startTime_tmp = clock();
				cv::filter2D(polarAlt, polarAlt_smooth, -1, kernel);
				totalIntervals[SMOOTH_ID] += clock() - startTime_tmp; //time debugging

				startTime_tmp = clock();
				ccDetection(polarAlt_smooth, dtctPpl, ttl_dtctPpl, pntsMap, ttlPnts); //Connected component detection
				totalIntervals[DET_ID] += clock() - startTime_tmp; //time debugging

				if (debug == DEBUG_HIGH)
				{
					//print bounding box areas					
					for (int i = 0; i < ttl_dtctPpl; i++)
					{
						const Person* p = &(dtctPpl[i]);
						float sgX = p->covMoA.at<float>(0,0);
						float sgY = p->covMoA.at<float>(1,1);
						outBboxModel << sgX*sgY << endl;

						if (frames == debugFrame)
						{
							printValuesF(&p->covMoA, "CovMoa(II)", outDebugFile);
							//printValuesF(&p->gtArea, "Gate area(II)", outDebugFile);
						}
					}

				}
				
				if (frames == 123)
				{
					outDebugFile << "****************BEGIN FRAME 96*******************" << endl;
					outDebugFile << "Detected person Id: " << dtctPpl[1].id << endl;
					printPerson(&dtctPpl[1]);
					
					outDebugFile << "Tracked person Id: " << trckPpl[1].id << endl;
					printPerson(&trckPpl[1]);
					outDebugFile << "****************END FRAME 96*******************" << endl;									
				}
				
				if (debug > DEBUG_MED && ttl_dtctPpl > 0)
				{
					for (int i = 0; i < ttl_dtctPpl; i++)
					{		
						//outDebugFile << "Printing detected person id: " << dtctPpl[i].id << endl;
						//printPerson(&dtctPpl[i]);
						//drawPersonCov_debug(dtctPpl[i], activityMap, Scalar(100, 0, 100));
						drawPersonMean_debug(dtctPpl[i], activityMap, Scalar(100,0,100));
					}
					imshow(windMoA, *activityMap);
					waitKey(0);
				}

				startTime_tmp = clock();
				tracking(trckPpl, ttl_trckPpl, dtctPpl, ttl_dtctPpl, activityMap);
				totalIntervals[TRACK_ID] += clock() - startTime_tmp; //time debugging
				if (debug > DEBUG_MED && ttl_trckPpl > 0) 
				{
					imshow(windMoA, *activityMap);
					waitKey(0);
				}
				//if (ttl_trckPpl > 2)
				//	cout << "stop dude" << endl;

				if (debug == DEBUG_HIGH)
				{
					for (int i = 0; i < ttl_trckPpl; i++)
					{
						Person  p = trckPpl[i];
						printPerson(&p);
					}
				}


				//For display purposes
				if (debug >= DEBUG_LOW)
				{
					convert16to8(&polarAlt_smooth, polarAlt_smooth_);
					convert16to8(&polarAlt, polarAlt_);
					convert16to8(&polar, polar_);
				}
				Mat *tmp = activityMap;
				if (!deleteBG)
					tmp = activityMap_Back;
				
				
				startTime_tmp = clock();
				//displayDetections(dtctPpl, ttl_dtctPpl, polarAlt_smooth_, pastPpl, ttlPastPpl, *tmp);
				displayDetections(trckPpl, ttl_trckPpl, polarAlt_smooth_, *tmp);
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

			if (recordOut == 1)
			{
				w << m;
				w1 << *activityMap;
			}
		}
		else
		{			
			startTime_tmp = clock();
			actMapCreator.createActivityMap(kinects, depthMaps, rgbMaps, trans, background, frames, MAX_RANGE, totalSubIntervalsMOA); 
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
		imshow(windMoA, *outMoA);
	
		if (debug >= DEBUG_LOW)
		{
			imshow(windPolarName, polar_);
			imshow("Polar Alt Smooth_", polarAlt_smooth_);
			imshow("Polar Alt", polarAlt_);
			imshow("rgb0", rgbImages[0]);
			imshow("rgb1", rgbImages[1]);
			imshow("rgb2", rgbImages[2]);
		}
		int c = waitKey(waitTime);
		
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
		//for (int i = 0; i < ttl_dtctPpl; i++)
		//{
		//	ttlPastPpl = ttl_dtctPpl;
		//	copyPerson(pastPpl[i], &dtctPpl[i]);
			//pastPpl[i] = dtctPpl[i];
		//}
		
		
		frames++;
	}
	//delete[] pastPpl;
	//delete[] dtctPpl;

	//for (int i = 0; i < ttl_trckPpl; i++)
	//	delete trckPpl[i];

	//delete[] trckPpl;

	totalIntervals[TOT_ID] = clock() - startTotalTime;
	//BUILD REPORT
	outDebugFile << "EXECUTION TIME REPORT" << endl;
	for (int i = 0; i < TOTAL_INTERVALS-1; i++)
	{
		float time_p = totalIntervals[i]*100/totalIntervals[TOT_ID];
		outDebugFile << titles[i] << ": " << time_p << " %" << endl;
	}

	outDebugFile << "PARTIAL EXECUTION TIME REPORT (MoA)" << endl;
	for (int i = 0; i < TOTAL_SUBINTERVAL_MOA; i++)
	{
		float time_p = totalSubIntervalsMOA[i]*100/totalIntervals[MOA_ID];
		outDebugFile << titles_subMOA[i] << ": " << time_p << " %" << endl;
	}


	outDebugFile << "PARTIAL EXECUTION TIME REPORT (UPDATE POLAR ALTERNATIVE)" << endl;
	for (int i = 0; i < TOTAL_SUBINTERVAL_RPS; i++)
	{
		float time_p = totalSubIntervalsRPS[i]*100/totalIntervals[RPSPACE_ID];
		outDebugFile << titles_subRPS[i] << ": " << time_p << " %" << endl;
	}


	double fps = frames/(double(totalIntervals[TOT_ID])/(double(CLOCKS_PER_SEC)));
	outDebugFile << "Total frames processed: " << frames << ". fps: " << fps << endl;

	

	
	for (int i = 0; i < NUM_SENSORS; i++)
	{
		kinects[i].stopDevice();
  		kinects[i].shutDown();
	}
	return 0;

}