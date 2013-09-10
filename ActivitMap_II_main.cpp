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
int debugFrame = -1;

vector<vector<float>> peopleRange;
Rect area (-1, -1, -1, -1);
char* windPolarName = "Polar";
char* windMoA = "Activity Map";
char* windPolarSmooth = "Polar Alt Smooth";
RNG rng(12345);

int debug = DEBUG_HIGH;

int frames = 0;

//Remap Polar Space
float MAX_RANGE;
float MAX_Z_TRNS;
int RANGE_ROWS = 500;
int RANGE_COLS = 181;

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
const int TOTAL_INTERVALS = 9;
const int BSUB_ID = 0;
const int RPSPACE_ID = 1;
const int MOA_ID = 2;
const int DET_ID = 3;
const int BACKPR_ID = 4;
const int PTRANS_ID = 5;
const int SMOOTH_ID = 6;
const int DISPLAY_ID = 7;
const int TOT_ID = 8;
char* titles[TOTAL_INTERVALS] = {"BACKGROUND SUBTRACTION", "REMAP POLAR SPACE", "MAP OF ACTIVITY", "DETECTION", "POINTS BACKPROJECTION", "POINTS TRANSFORMATION", "SMOOTH", "DISPLAY", " TOTAL"};
float totalIntervals[TOTAL_INTERVALS] = {0,0,0,0,0,0,0,0,0};

//Time interval sub-function
const int TOTAL_SUBINTERVAL = 4;
const int ANGLE_ID = 0;
const int RANGE_ID = 1;
const int UPDATE_ID = 2;
const int FEATURE_ID = 3;
char* titles_sub[TOTAL_SUBINTERVAL] = {"ANGLE", "RANGE", "UPDATE", "FEATURE"};
float totalSubIntervals[TOTAL_SUBINTERVAL] = {0,0,0,0};

//PERSON MODEL SETTINGS

const int MODEL_MAX_HEIGHT = 300;
const int MODEL_MIN_HEIGHT = -2000;
const int MODEL_BINRANGE = 230;
const int MODEL_NBINS = (MODEL_MAX_HEIGHT-MODEL_MIN_HEIGHT)/MODEL_BINRANGE;

struct Feature
{
	Scalar color;
	float height;
};
typedef multimap <int, Feature, less<int>> mmFS;

//struct BinFeature
//{
//	Scalar meanColor;
//	int total;
//};

struct Person
{
	int id;
   Point  mean;
   int sigmaY;
   int sigmaX;
   float heightModel[MODEL_NBINS];
   Scalar colourModel[MODEL_NBINS];
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


/*
polarAlt(out): Updates the positions where the 3D points project after the remapping.
polar(out): Points projected from the transform points into thep polar coordinate system
featureSpace(out): Feature space of colour and height. Each position in the polarAlt contain a set of features. Each feature corresponds from a point that projected onto that pixel

p3D(in): List of foreground points for a particular kinect
points2D(in): List of 2D points of the image plane for a particular kinect
rgbMap(in): Map of rgb colours for a particular kinect
nP(in): Number of foreground points.
*/
void updatePolarAlternateive(Mat* polarAlt, Mat* polar, mmFS& fs, const XnPoint3D* p3D, const XnPoint3D* points2D, const XnRGB24Pixel* rgbMap, const int nP)
{
	float toRad = 180/CV_PI;
	int pAltStep = polarAlt->step;
	int pStep = polar->step;

	ushort* ptrPAlt = (ushort*)polarAlt->data;
	ushort* ptrP = (ushort*)polar->data;

	int ushortSize = sizeof(ushort);
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
		totalSubIntervals[ANGLE_ID] += clock() - startTime; //time debugging
	
		startTime = clock(); //time debuggin
		float range_m = range/1000;
		float range_Alt = SCALE_RANGE_ALT * ((0.33*log(10+3*range_m))-0.80597); // three distances
		totalSubIntervals[RANGE_ID] += clock() - startTime; //time debugging

		startTime = clock(); //time debuggin
		int yPos = RANGE_ROWS - ((int)range_Alt/RANGE_STEP);	
		if (yPos < 0 || yPos >= RANGE_ROWS)
		{
			if (debug > DEBUG_NONE)
				outDebugFile << "UpdatePolarAlternative: BAD REMAPING (Remap Polar Space). Info: 3DPoint: " << X <<", "<< Y << ", " << Z << ". PolarPoint: " <<
					 range <<", "<< angle << ". RemapPolarPoint: " << range_Alt << ". RmapPolarPlanview: " << angle << ", " << yPos <<
					 ". SCALE_RANGE_ALT: " << SCALE_RANGE_ALT << ". RANGE_ROWS: " << RANGE_ROWS << ". RANGE_STEP: " << RANGE_STEP << endl;
			continue;
		}

		//int val = polarAlt->ptr<ushort>(yPos)[angle] - 1;
		//polarAlt->data * (yPos*polarAlt->step)[angle]++;
		//polarAlt->ptr<ushort>(yPos)[angle]++;
		(ptrPAlt + ((yPos*pAltStep)/ushortSize))[angle]++;
		totalSubIntervals[UPDATE_ID] += clock() - startTime; //time debugging


		startTime = clock(); //time debuggin
		//add a new feature
		int x2d = points2D[i].X;
		int y2d = points2D[i].Y;
		XnRGB24Pixel color = rgbMap[y2d*XN_VGA_X_RES+x2d]; //TODO: CONVERT TO HSV
		//Feature f;
		//f.height = (float)Y;
		//f.color = Scalar(color.nRed, color.nGreen, color.nBlue);
		//int key = yPos * RANGE_COLS + angle;
		//fs.insert(mmFS::value_type(key, f));
		totalSubIntervals[FEATURE_ID] += clock() - startTime; //time debugging


		if (debug > DEBUG_NONE)
		{
			yPos = RANGE_ROWS - ((int)range/RANGE_STEP);
			if (yPos < 0 || yPos >= RANGE_ROWS)
			{
				if (debug > DEBUG_NONE)
					outDebugFile << "UpdatePolarAlternative: BAD REMAPING (Polar Space). Info: 3DPoint: " << X <<", "<< Y << ", " << Z << ". PolarPoint: " <<
						 range <<", "<< angle << ". RANGE_ROWS: " << RANGE_ROWS << ". RANGE_STEP: " << RANGE_STEP << endl;
				continue;
			}
			//polar->ptr<ushort>(yPos)[angle]++;
			(ptrP + ((yPos*polar->step)/ushortSize))[angle]++;
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
	for (int i = 0; i < XN_VGA_Y_RES; i++)
	{
		ushort* ptr = depthMat.ptr<ushort>(i);
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
	while (i < (k.y + k.height) && !found)
	{
		const ushort* ptrImg = img->ptr<ushort>(i);
		int j = k.x;
		while (j < (k.x+k.width) && !found)
		{
			int tmp = ptrImg[j];
			found = tmp > val;
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

    for(int y=0; y < label_image.rows; y++) 
	{
        int *row = (int*)label_image.ptr(y);
        for(int x=0; x < label_image.cols; x++) 
		{
            if(row[x] > 0) 
			{
                continue;
            }

            cv::Rect rect;
            cv::floodFill(label_image, cv::Point(x,y), label_count, &rect, 0, 0, 4);

            std::vector <cv::Point2i> blob;

            for(int i=rect.y; i < (rect.y+rect.height); i++) 
			{
                int *row2 = (int*)label_image.ptr(i);
                for(int j=rect.x; j < (rect.x+rect.width); j++) 
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
void initPerson(Person& p)
{
//	Scalar c (0,0,0);
	for (int i = 0; i < MODEL_NBINS; i++)
	{
		p.heightModel[i] = 0;
//		p.colourModel[i] = 0;
	}
}


/*
Perform a connected component labeling on the remap polar coordinate space.

bw: Mat(500,181, CV_16UC1);
*/
void detectCC(Mat& bw, list<Person>* people, mmFS& fs, Mat& debugImg)
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

	std::pair <mmFS::iterator, mmFS::iterator> ret;
	int idPers = 0;
	while (iter != blobs.end())
	{
		bool pass = false;
		vector<Point2i> b = *iter;
		//if (b.size() > 10) //rejection of small regions
		{
			x = y = xx = yy = ww = 0;
			for (int i = 0; i < b.size(); i++)
			{
				Point2i p = b[i];
								
				float rangeTmp = (bw.rows-p.y)*RANGE_STEP; //binSize
				float thresh;
				if (rangeTmp > 2300)
					//thresh= 67900*exp((-0.00044)*rangeTmp); //low threshold
					thresh= 66900*exp((-0.00048)*rangeTmp); //low threshold
				else
					thresh = 15000;
				//float thresh = 68103*exp((-0.000446)*rangeTmp); //low threshold
				int w = bw.ptr<ushort>(p.y)[p.x];
				if (w > thresh) //check that there is at least one pixel that get the higher threshold
						pass = true;

				x += (p.x*w);
				y += (p.y*w);
				ww += w;
			}
			if (pass)
			{
				Person p;
				initPerson(p);
				p.mean = Point(x/ww, y/ww);
				p.id = idPers++;
				for (int i = 0; i < b.size(); i++)
				{
					Point2i point = b[i];
					//debug 
					if (debug > DEBUG_MED)//ONLY points that surpass the higher threshold are drawn in yellow
					{
						debugImg.ptr<uchar>(point.y)[point.x*3] = 0;
						debugImg.ptr<uchar>(point.y)[point.x*3+1] = 255;
						debugImg.ptr<uchar>(point.y)[point.x*3+2] = 255;
					}

					int w = bw.ptr<ushort>(point.y)[point.x];

				
					xx += w*powf(point.x-p.mean.x,2);
					yy += w*powf(point.y-p.mean.y,2);

					//get all the features (colour, height) associated. Multimap (key = point.y*180+point.x, values = feature(colour, height))
					//For each feature update the two fields of the model for the 
					//corresponding bin (depends on the height): the number of points 
					//in that particular bin and the avg colour (CAi+1 = (xi+1 + iCAi)/i+1).
					
					int key = point.y*bw.cols + point.x;
					if (fs.count(key) > 0)
					{
						ret = fs.equal_range(point.y*bw.cols + point.x);
						for (mmFS::iterator iter = ret.first; iter != ret.second; iter++)
						{
							Feature feat = iter->second;
							if (debug > DEBUG_NONE && frames == debugFrame)
							{
								outHeights << feat.height << endl;
							}
							//calculate position in the vector (depends on the number of bins and min and max heights.
							if (feat.height < MODEL_MAX_HEIGHT && feat.height > MODEL_MIN_HEIGHT)
							{
								int bin = (MODEL_MAX_HEIGHT - feat.height)/MODEL_BINRANGE;
								//update the model (height, colour)
								if (p.heightModel[bin] > 0)	
								{
									p.colourModel[bin](0) = (feat.color[0] + p.heightModel[bin]*p.colourModel[bin](0))/(p.heightModel[bin]+1);
									p.colourModel[bin](1) = (feat.color[1] + p.heightModel[bin]*p.colourModel[bin](1))/(p.heightModel[bin]+1);
									p.colourModel[bin](2) = (feat.color[2] + p.heightModel[bin]*p.colourModel[bin](2))/(p.heightModel[bin]+1);
								}
								else
									p.colourModel[bin] = feat.color;


								p.heightModel[bin]++;

								if (debug > DEBUG_MED && frames == debugFrame && p.id == 0)
								{
									outPersModel << feat.height << " " << feat.color[0] << " " << feat.color[1] << " " << feat.color[2] << endl;
								}

							}
							else
							{
								outDebugFile << "DetectCC: Person point out of height range: Info: RPS coordinates: " << point.x << ", " << point.y << ". Key: " << key <<
									". Height: " << feat.height << ". MAX_HEIGHT: " << MODEL_MAX_HEIGHT << ". MIN_HEIGHT: " << MODEL_MIN_HEIGHT << ". Colour: " 
									<< feat.color(0) << ", " << feat.color(1) << ", " << feat.color(2) << endl;
							}
						}
					}

				}

				p.sigmaY = sqrtf(yy/ww);
				p.sigmaX = sqrtf(xx/ww);
				normalizeHeightModel(p.heightModel);
				if (frames == debugFrame && p.id == 0)
				{
					outDebugFile << "Color model at frame " << frames << endl;
					for (int k = 0; k < MODEL_NBINS; k++)
					{
						outDebugFile << p.heightModel[k] << " " << p.colourModel[k](0) << " " << p.colourModel[k](1) << " " << p.colourModel[k](2) << endl;
					}
				}
				people->push_back(p);
			}
		}
		iter++;
	}
//	imshow("test0", imgCpy);
}


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

void printValuesF(const Mat* m, char* title)
{
	cout << title << endl;
	for (int i = 0; i < m->rows; i++)
	{
		const float* ptr = m->ptr<float>(i);
		for (int j = 0; j < m->cols; j++)
		{
			cout << (float)ptr[j] << " ";
		}
		cout << endl;
	}
	cout << endl;
}


/*
img : Mat(500,181, CV_16UC1)

*/
void ccDetection(Mat& img, list<Person>& people, mmFS& fs)
{
	Mat imgCpy1, imgCpy2;
	img.copyTo(imgCpy1);

	//First threshold
	for (int i = 0; i < img.rows; i++)
	{
		ushort* ptr = img.ptr<ushort>(i);
		for (int j = 0; j < img.cols; j++)
		{
			int val = ptr[j];
			float range = (img.rows-(i))*RANGE_STEP; //binSize
			
			//linearize version of the threshold
			//float thresh;
			//if (range < 4500)
			//	thresh = -5.6*range+32219;
			//else
			//	thresh = -0.73*range+769;

			float thresh = 50000*exp((-0.0006)*range); //low threshold
			if (val < thresh)
				ptr[j] = 0;
		}
	}
	img.copyTo(imgCpy2);
	
	Mat outDebug = Mat(RANGE_ROWS, 181, CV_8UC3);	
	if (debug > DEBUG_MED)
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
	detectCC(img, &people, fs, outDebug);
	if (debug > DEBUG_MED)
	{
		imshow("Thresholds", outDebug);	
	}
}

void displayDetections(list<Person>& people, Mat& remapPolar, Mat& moa)
{
	list<Person>::iterator iterLocs = people.begin();
	while(iterLocs != people.end())
	{
		Person p = *iterLocs;
		if (debug > DEBUG_NONE)
		{
			circle(remapPolar, p.mean, 2, Scalar::all(0), -1);
			ellipse(remapPolar, p.mean, Size(p.sigmaX*2, p.sigmaY*2), 0,0,360, Scalar::all(0));
		}

		//Mean projection to MoA
		Point cMoA, pMean;
		cMoA = convertBack(&p.mean);
		XnPoint3D p3D;
		p3D.X = cMoA.x; p3D.Y = ActivityMap_Utils::CEILING_THRESHOLD - 1; p3D.Z = cMoA.y;
		pMean = ActivityMap_Utils::findMoACoordinate(&p3D, MAX_RANGE);
		if (pMean.x == -1)
		{
			if (debug > DEBUG_NONE)
			{
				outDebugFile << "displayDetections: person located out of MoA range. INFO: RPS point: " << p.mean.x << ", " << p.mean.y <<
					". 3D MoA point: " << cMoA.x << ", " << cMoA.y << ". Range: " << sqrtf(pow(p3D.X,2) + pow(p3D.Z,2)) <<". MAX_RANGE: " << 
					MAX_RANGE << ". MAX_Z_TRANS: " << ActivityMap_Utils::MAX_Z_TRANS << ". CEILING_THRESH: " << ActivityMap_Utils::CEILING_THRESHOLD << 
					". FLOOR_THRESH: " << ActivityMap_Utils::FLOOR_THRESHOLD << endl;
			}

			iterLocs++;
			continue; //TODO: ERROR LOG
		}
		//Covariance projection to MoA
		float sigmaAlphaRad = p.sigmaX*2*CV_PI/180;
		float sigmaRange = p.sigmaY*2;
		float meanAlpha = p.mean.x*CV_PI/180; //rad
		float meanRange = p.mean.y; //mm
						
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
		int sigmaX_II = sqrtf(covCartessian.at<float>(0,0));
		int sigmaY_II = sqrtf(covCartessian.at<float>(1,1));

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

		circle(moa, pMean, 2, Scalar(0,0,255));
		ellipse(moa, pMean, Size(sigmaY_II, sigmaX_II), -p.mean.x, 0, 360, Scalar(0,0,255));
		iterLocs++;
	}
	people.clear();
}


/*
Arg 1: 0:Video; 1:live
Arg 2: 0:No Record; 1:Record
*/
int main(int argc, char* argv[])
{

	bool saved = false;
	int fromVideo = 1;
	int recordOut = 1;
	int tilt = 0;
	tiltTXT >> tilt;
	cout << "Start" << endl;
	if (argc ==3)
	{
		sscanf(argv[1], "%d", &fromVideo);
		sscanf(argv[2], "%d", &recordOut);
	}

	char* paths[3];
	/*paths[0] = "d:/Emilio/Tracking/DataSet/kinect0_calib.oni";
	paths[1] = "d:/Emilio/Tracking/DataSet/kinect1_calib.oni";
	paths[2] = "d:/Emilio/Tracking/DataSet/kinect2_calib.oni";*/

	paths[0] = "d:/Emilio/Tracking/DataSet/Dset2_workshop/kinect0_calib.oni";
	paths[1] = "d:/Emilio/Tracking/DataSet/Dset2_workshop/kinect1_calib.oni";
	paths[2] = "d:/Emilio/Tracking/DataSet/Dset2_workshop/kinect2_calib.oni";

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



	namedWindow(windMoA);
	Mat *activityMap, *activityMap_Back;
	Mat whiteBack, colorMap;
	Mat background = Mat(actMapCreator.getResolution(), CV_8UC3);
	Mat backgroundPolar = Mat(actMapCreator.getResolution().height+150, 181, CV_8UC3);

	//initialize Feature space
	mmFS fs;


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
	int TotalFrames_2 = 900;

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

	namedWindow(windPolarName);
	cvSetMouseCallback(windPolarName, selectROI_callBack, (Mat*)&polar_);
	cvSetMouseCallback(windPolarName, selectPoint_callBack, (Mat*)&polar_);

	//Size of kernel: smooth rps;
	Mat kernel = Mat::ones(Size(5,27), CV_32F);

	list<Person> people;
	clock_t startTime = clock();
	clock_t startTotalTime = clock();
	clock_t startTime_tmp;
	int nPoints = 0;
	while (!bShouldStop && frames < TotalFrames_2)
	{		

		if (debug != DEBUG_NONE)
		{
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
		
		list<Point> locations;
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
				if (debug > DEBUG_LOW)//Draw the output of the foreground detection
				{
					if (frames == debugFrame && i == 0)
						imwrite("c:/Dropbox/Phd/Matlab/Model/rgb_0_Orig.jpg", rgbImages[0]);
					for (int c = 0; c < numberOfForegroundPoints[i]; c++)
					{
						XnPoint3D* p = &(pointsFore2D[i][c]); 
						uchar* ptr = rgbImages[i].ptr<uchar>(p->Y);
						ptr[3*(int)p->X] = 0;
						ptr[(3*(int)p->X)+1] = 0;
						ptr[(3*(int)p->X)+2] = 255;
					}
					if (frames == debugFrame && i == 0)
						imwrite("c:/Dropbox/Phd/Matlab/Model/rgb_0_Red.jpg", rgbImages[0]);
				}
			}
			if (nPoints > 0)
			{
				for (int i = 0; i < NUM_SENSORS; i++)
				{
					startTime_tmp = clock();
					points3D[i] = kinects[i].arrayBackProject(pointsFore2D[i], numberOfForegroundPoints[i]);
					totalIntervals[BACKPR_ID] += clock() - startTime_tmp; //time debugging

					startTime_tmp = clock();
					kinects[i].transformArray(points3D[i], numberOfForegroundPoints[i]);
					totalIntervals[PTRANS_ID] += clock() - startTime_tmp; //time debugging
					
					//Create alternative representation
					startTime_tmp = clock();
					updatePolarAlternateive(&polarAlt, &polar, fs, points3D[i], pointsFore2D[i], rgbMaps[i], numberOfForegroundPoints[i]);	
					totalIntervals[RPSPACE_ID] += clock() - startTime_tmp; //time debugging
					
					startTime_tmp = clock();
					updateActivityMap(*activityMap, *activityMap_Back, &actMapCreator, points3D[i], numberOfForegroundPoints[i], pointsFore2D[i]);
					totalIntervals[MOA_ID] += clock() - startTime_tmp; //time debugging
				}
		
				//Todo: Create a method detection(polarAlt, moAPeople)
				startTime_tmp = clock();
				filter2D(polarAlt, polarAlt_smooth, -1, kernel);
				totalIntervals[SMOOTH_ID] += clock() - startTime_tmp; //time debugging

				startTime_tmp = clock();
				ccDetection(polarAlt_smooth, people, fs); //Connected component detection
				totalIntervals[DET_ID] += clock() - startTime_tmp; //time debugging

				if (!fs.empty())
						fs.clear();
				//polarAlt_smooth contains the actual number of points projected
				//so if I select a roi i could retrieve all the information
				//if i could retrieve the mean and the variance will be very useful
								
				//For display purposes
				if (debug > DEBUG_NONE)
				{
					convert16to8(&polarAlt_smooth, polarAlt_smooth_);
					convert16to8(&polarAlt, polarAlt_);
					convert16to8(&polar, polar_);
				}
				Mat *tmp = activityMap;
				if (!deleteBG)
					tmp = activityMap_Back;
				
				startTime_tmp = clock();
				displayDetections(people, polarAlt_smooth_, *tmp);
				totalIntervals[DISPLAY_ID] += clock() - startTime_tmp; //time debugging

				if (debug > DEBUG_LOW)//Show the comparison between the smoothed and the original remap polar space
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
					cvSetMouseCallback(windMoA, selectPoint_callBack, (Mat*)activityMap);
					Mat** images = new Mat*[2];
					images[0] = &imgDebug;
					images[1] = activityMap;
					cvSetMouseCallback("Smooth comparison", selectPoint2_callBack, images);
					imshow("Smooth comparison", imgDebug);
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
			actMapCreator.createActivityMap(kinects, depthMaps, rgbMaps, trans, background, frames, MAX_RANGE); 
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
	
		if (debug > DEBUG_NONE)
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
		
		frames++;
	}


	totalIntervals[TOT_ID] = clock() - startTotalTime;
	//BUILD REPORT
	outDebugFile << "EXECUTION TIME REPORT" << endl;
	for (int i = 0; i < TOTAL_INTERVALS-1; i++)
	{
		float time_p = totalIntervals[i]*100/totalIntervals[TOT_ID];
		outDebugFile << titles[i] << ": " << time_p << " %" << endl;
	}

	outDebugFile << "PARTIAL EXECUTION TIME REPORT (UPDATE POLAR ALTERNATIVE)" << endl;
	for (int i = 0; i < TOTAL_SUBINTERVAL; i++)
	{
		float time_p = totalSubIntervals[i]*100/totalIntervals[RPSPACE_ID];
		outDebugFile << titles_sub[i] << ": " << time_p << " %" << endl;
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