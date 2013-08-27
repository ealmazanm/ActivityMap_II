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

const int NUM_SENSORS = 3;

ifstream tiltTXT("D:\\CameraCalibrations\\extrinsics\\tilt.txt");
ofstream outStd("d:/Emilio/Kinect/DepthSensorStudy/PeopleVariance.txt");
vector<vector<float>> peopleRange;
Rect area (-1, -1, -1, -1);
char* windPolarName = "Polar";
char* windMoA = "Activity Map";
char* windPolarSmooth = "Polar Alt Smooth";
int step = 0;
RNG rng(12345);

int debug = DEBUG_HIGH;

//debug
int DEPTH_STEP, X_STEP;
int frames = 0;


struct Person
{
   Point  mean;
   int sigmaY;
   int sigmaX;
	   
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
							float range = i*step;
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
		
		float X = ActivityMap_Utils::findCoordinate_inv(x, ActivityMap_Utils::MIN_X, ActivityMap_Utils::MAX_X, X_STEP);
		int yy = moa->rows - y;
		float Z = ActivityMap_Utils::findCoordinate_inv(yy, ActivityMap_Utils::MIN_Z, ActivityMap_Utils::MAX_Z, DEPTH_STEP);
						

		int step = 11650/500;
		float range = sqrtf(pow(X,2) + pow(Z,2));
		int angle = 90;
		if (X != 0 && Z != 0)
			if (X > 0)
				angle = (atanf(Z/abs(X)))*180/CV_PI;
			else
				angle = 180-((atanf(Z/abs(X)))*180/CV_PI);

		float range_m = range/1000;
		float range_Alt = (int)25896 * ((0.33*log(10+3*range_m))-0.80597);
		int yAlt = 500 - ((int)range_Alt/step);	

		//Calculate range
		float rangeAlt_back = (500 - yAlt)*step;
		float range_back = (-3.83328*(0.869577-exp(0.000117018*rangeAlt_back)))*1000;
	
		float X_Back = range_back*cos(angle*CV_PI/180); 
		float Z_Back = range_back*sin(angle*CV_PI/180);

		int x_Back =  ActivityMap_Utils::findCoordinate(X_Back, ActivityMap_Utils::MIN_X, ActivityMap_Utils::MAX_X, X_STEP);
		int yC =  ActivityMap_Utils::findCoordinate(Z_Back, ActivityMap_Utils::MIN_Z, ActivityMap_Utils::MAX_Z, DEPTH_STEP);
		int y_Back = (YRes-1) - yC; //flip around X axis.

		circle(tmp, Point(x_Back, y_Back), 2, Scalar(0,255,0));
		
		imshow("Tmp", tmp);
		waitKey(0);
		
	}
}


void selectPoint2_callBack(int event, int x, int y, int flags, void* param)
{
	Mat** images = (Mat**)param;
	Mat* moa = images[1];
	Mat* polarSmooth = images[0];
	
	int step = 11650/500;
	if (event == CV_EVENT_LBUTTONDOWN) 
	{
		circle(*polarSmooth, Point(x,y), 2, Scalar::all(225));
		
		imshow(windPolarSmooth, *polarSmooth);
		waitKey(0);

		//Calculate range
		float rangeAlt_back = (500 - y)*step;
		float range_back = (-3.83328*(0.869577-exp(0.000117018*rangeAlt_back)))*1000;
	
		float X_Back = range_back*cos(x*CV_PI/180); 
		float Z_Back = range_back*sin(x*CV_PI/180);

		int x_Back =  ActivityMap_Utils::findCoordinate(X_Back, ActivityMap_Utils::MIN_X, ActivityMap_Utils::MAX_X, X_STEP);
		int yC =  ActivityMap_Utils::findCoordinate(Z_Back, ActivityMap_Utils::MIN_Z, ActivityMap_Utils::MAX_Z, DEPTH_STEP);
		int y_Back = (YRes-1) - yC; //flip around X axis.

		circle(*moa, Point(x_Back, y_Back), 2, Scalar(0,255,0));
		
		imshow(windMoA, *moa);
		waitKey(0);
		
	}
}


void updatePolarAlternateive(Mat* polarAlt, Mat* polar, const XnPoint3D* p3D, const int nP)
{

	//max range (mm)
	double MaxRange = 11650;
	step = ceil(MaxRange/polarAlt->rows);
	for (int i = 0; i < nP; i++)
	{
		XnPoint3D p = p3D[i];
		float range = sqrtf(pow(p.X,2) + pow(p.Z,2));
		int angle = 90;
		if (p.X != 0 && p.Z != 0)
			if (p.X > 0)
				angle = (atanf(p.Z/abs(p.X)))*180/CV_PI;
			else
				angle = 180-((atanf(p.Z/abs(p.X)))*180/CV_PI);

	
		float range_m = range/1000;
		float range_Alt = (int)25896 * ((0.33*log(10+3*range_m))-0.80597); // three distances

		int yPos = polarAlt->rows - ((int)range_Alt/step);	
		if (yPos < 0 || yPos >= polarAlt->rows)
			cout << "Hey" << endl;

		//int val = polarAlt->ptr<ushort>(yPos)[angle] - 1;
		polarAlt->ptr<ushort>(yPos)[angle]++;
		
		yPos = polar->rows - ((int)range/step);
		if (yPos < 0 || yPos >= polarAlt->rows)
			cout << "Hey" << endl;
		polar->ptr<ushort>(yPos)[angle]++;
	}
}

void updateActivityMap(Mat& activityMap, Mat& activityMap_back, const ActivityMap_Utils* am, const XnPoint3D* p3D, const int nP, const XnPoint3D* points2D, const XnRGB24Pixel* rgbMap)
{
	Mat hightMap = Mat::Mat(activityMap.size(), CV_32F);
	Utils::initMatf(hightMap, -5000);
	for (int i = 0; i < nP; i++)
	{
		int xCoor = am->findCoordinate(p3D[i].X, ActivityMap_Utils::MIN_X, ActivityMap_Utils::MAX_X, am->xStep);
		int yC = am->findCoordinate(p3D[i].Z, ActivityMap_Utils::MIN_Z, ActivityMap_Utils::MAX_Z, am->depthStep);
		int yCoor = (YRes-1) - yC; //flip around X axis.

		XnRGB24Pixel color = rgbMap[(int)points2D[i].Y*XN_VGA_X_RES+(int)points2D[i].X];

		uchar* ptr = activityMap.ptr<uchar>(yCoor);
		uchar* ptr_back = activityMap_back.ptr<uchar>(yCoor);
		float* ptrH = hightMap.ptr<float>(yCoor);

		if (ptrH[xCoor] < (float)p3D[i].Y)
		{
			ptrH[xCoor] = (float)p3D[i].Y;

			ptr[3*xCoor] = color.nBlue;
			ptr[3*xCoor+1] = color.nGreen;
			ptr[3*xCoor+2] = color.nRed;

			ptr_back[3*xCoor] = color.nBlue;
			ptr_back[3*xCoor+1] = color.nGreen;
			ptr_back[3*xCoor+2] = color.nRed;
		}
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
This function is only called when there value in kernel is > 0
*/
void look4Centr(Point& centr, Rect& kernel, Mat& copyImg, Mat& original)
{
	Mat nullMat = Mat::zeros(kernel.width, kernel.height, CV_16UC1);
	centr.x = kernel.x+kernel.width/2;
	centr.y = kernel.y+kernel.height/2;
	int sumX, sumY, sumVals;
	Point ctrTmp(-1,-1);
	int minDist = 5;
	int maxNumIters = 5;
	int iter = 0;
	do
	{
		if (ctrTmp.x != -1)
		{
			centr.x = ctrTmp.x;
			centr.y = ctrTmp.y;
		}
		//Calculate centroid at kernel position		
		sumX = sumY = sumVals = 0;
		for (int i = kernel.y; i < kernel.y+kernel.height; i++)
		{
			ushort* ptr = original.ptr<ushort>(i);
			for (int j = kernel.x; j < kernel.x+kernel.width; j++)
			{
				int val = (int)ptr[j];
				sumX += j*val;
				sumY += i*val;
				sumVals += val;
			}
		}
		ctrTmp.x = sumX/sumVals;
		ctrTmp.y = sumY/sumVals;
		//clean copy
		Mat roi = copyImg(kernel);
		nullMat.copyTo(roi);
		//update kernel to new position
		kernel.x = ctrTmp.x - kernel.width/2;
		kernel.y = ctrTmp.y - kernel.height/2;
		//Check boundaries of the new kernel
		assert(kernel.x+kernel.width < original.cols);
		assert(kernel.y+kernel.height < original.rows);
		assert(kernel.x >= 0 && kernel.y >= 0);
		iter++;
	}while(distf(centr, ctrTmp) > minDist && iter < maxNumIters);

	centr.x = ctrTmp.x;
	centr.y = ctrTmp.y;
	//Update original
	copyImg.copyTo(original);
}


/*
p = Point in the remap polar space
out = Correspoding point in the Polar coordinate space
*/
Point remap2Polar(const Point* p)
{
	Point out;
	//Calculate range
	float rangeAlt = (500 - p->y)*23.3;
	out.y = (-3.83328*(0.869577-exp(0.000117018*rangeAlt)))*1000;
	out.x = p->x;
	return out;

}

Point convertBack(const Point* p)
{
	Point out;
	//Calculate range
	float rangeAlt = (500 - p->y)*23.3;
	float range = (-3.83328*(0.869577-exp(0.000117018*rangeAlt)))*1000;
	out.x = range*cos((p->x)*CV_PI/180); //+4 there is a misplacement of around 4 degrees in the estimation (ToDo: find out why)
	out.y = range*sin((p->x)*CV_PI/180);
	return out;
}


Point convertBack_Polar(const Point* p)
{
	Point out;
	//Calculate range
	float rangeAlt = (500 - p->y)*23.3;
	float range = (-3.83328*(0.869577-exp(0.000117018*rangeAlt)))*1000;
			
	int step = 11650/500;
	out.y = 500 - ((int)range/step);
	out.x = p->x;

	return out;
}



/*
polarAlt : Mat(500,181, CV_16UC1);
polarAlt_smooth : Mat(500,181, CV_16UC1);
sz : Size of the kernel
*/
void uniformConvolution(const Mat* polarAlt, Mat& polarAlt_smooth, Size sz)
{
	Rect k = Rect(0,0, sz.width, sz.height);

	int endRow = polarAlt->rows - sz.height;
	int endCol = polarAlt->cols - sz.width;
	Utils::initMat1s(polarAlt_smooth, 0);
	for (int i = 0; i < endRow; i++)
	{
		ushort* ptrSmooth = polarAlt_smooth.ptr<ushort>(i+sz.height/2);
		for (int j = 0; j < endCol; j++)
		{
			k.x = j;
			k.y = i;
			Mat roi = (*polarAlt)(k);
			ptrSmooth[j+sz.width/2] = sum(roi).val[0];
		}
	}
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

void modeFinder(const Mat* img, Size sz, list<Point>& locations)
{
	Mat imgCpy = Mat(img->size(), CV_8UC1);

	Rect k = Rect(0,0, sz.width, sz.height);
	TermCriteria term = TermCriteria(TermCriteria::COUNT, 50, 0.1);
	int endRow = img->rows - sz.height;
	int endCol = img->cols - sz.width;
	for (int i = 0; i < endRow; i += sz.height)
	{
		k.y = i;
		for (int j = 0; j < endCol; j += sz.width)
		{
			k.x = j;		
			Rect kcpy (k.x, k.y, k.width, k.height);

			meanShift(*img, kcpy, term);
			//RotatedRect out = CamShift(*img, kcpy, term);
			Point p (kcpy.x+kcpy.width/2, kcpy.y+kcpy.height/2);
			float range = (img->rows-(p.y))*23.3; //binSize
			float thresh = 68103*exp((-0.0004687)*range);
			int val = img->ptr<ushort>(p.y)[(int)p.x];
			if (val > thresh)
			{
				if (find(locations.begin(), locations.end(), p) == locations.end())
					locations.push_back(p);
			}
		}
	}
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
Perform a connected component labeling on the remap polar coordinate space.

bw: Mat(500,181, CV_16UC1);
*/
void detectCC(Mat& bw, list<Person>* people, list<Person>* peoplePolar, Mat& debugImg)
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


	while (iter != blobs.end())
	{
		bool pass = false;
		vector<Point2i> b = *iter;
		//if (b.size() > 10) //rejection of small regions
		{
			x = y = xx = yy = ww = 0;
			range = alpha = rr = aa = 0;
			for (int i = 0; i < b.size(); i++)
			{
				Point2i p = b[i];
								
				float rangeTmp = (bw.rows-p.y)*23.3; //binSize
				float thresh = 68103*exp((-0.000446)*rangeTmp); //low threshold
				int w = bw.ptr<ushort>(p.y)[p.x];
				if (w > thresh) //check that there is at least one pixel that get the higher threshold
						pass = true;


				Point2i pPolar = remap2Polar(&p);				
				range += pPolar.y*w;
				alpha += pPolar.x*w;

				x += (p.x*w);
				y += (p.y*w);
				ww += w;
			}
			if (pass)
			{
				Person p, pPolar;
				p.mean = Point(x/ww, y/ww);
				pPolar.mean = Point(alpha/ww, range/ww);

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

					Point2i pointPolar = remap2Polar(&point);
					int w = bw.ptr<ushort>(point.y)[point.x];

					aa += w*powf(pointPolar.x-pPolar.mean.x,2);
					rr += w*powf(pointPolar.y-pPolar.mean.y,2);

					xx += w*powf(point.x-p.mean.x,2);
					yy += w*powf(point.y-p.mean.y,2);
				}

				pPolar.sigmaY = sqrtf(rr/ww);
				pPolar.sigmaX = sqrtf(aa/ww);
				peoplePolar->push_back(pPolar);

				p.sigmaY = sqrtf(yy/ww);
				p.sigmaX = sqrtf(xx/ww);
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


void growRegions2(Mat& img, const Mat* imgCpy2, const Mat* imgCpy1)
{
	Rect k = Rect(0,0, 5, 5);

	//Find non zero points in imgCpy
	for (int i = k.height/2; i < imgCpy2->rows-k.height/2; i++)
	{
		const ushort* ptr2 = imgCpy2->ptr<ushort>(i);
		k.y = i - k.height/2;;
		for (int j = k.width/2; j < imgCpy2->cols-k.width/2; j++)
		{
			k.x = j - k.width/2;
			if (ptr2[j] > 0)
			{

				float range = (img.rows-(i))*23.3; //binSize
				float thresh = 68103*exp((-0.00055)*range); //low threshold

				Mat roi2 = (*imgCpy2)(k);
				Mat roi1 = (*imgCpy1)(k);
				Mat roi = img(k);
				/*printValuesS(&roi2, "Roi2");
				printValuesS(&roi1, "Roi1");
				printValuesS(&roi, "Roi");*/

				for (int y = 0; y < k.height; y++)
				{
					ushort *p2 = roi2.ptr<ushort>(y);
					ushort *p1 = roi1.ptr<ushort>(y);
					ushort *p = roi.ptr<ushort>(y);
					for (int x = 0; x < k.width; x++)
					{
						if (p2[x] == 0 && p1[x] > thresh)
							p[x] = p1[x];
					}
				}
			//	printValuesS(&roi, "Roi");

	/*			Mat mask2, mask1, mask;	
				roi2.copyTo(mask1);
				threshold(roi2, mask1, 1, 10, CV_THRESH_BINARY_INV);
				printValuesS(&mask1, "Mask1");

				
				threshold(roi1, mask2, thresh, 1, CV_THRESH_BINARY);	
				printValuesS(&mask2, "Mask2");
				
				bitwise_xor(mask1, mask2, mask);
				printValuesS(&mask, "Mask");

				Mat roi = img(k);
				add(roi1, roi2, roi, mask);*/
			}

		}
	}
}

void growRegions(Mat& img, const Mat* imgCpy2, const Mat* imgCpy1)
{
	Rect kernel = Rect(0,0, 5,5);
	//for (int i = 0; i < img.rows-kernel.height; i++)
	for (int i = img.rows-kernel.height-1; i >= 0; i--)
	{
		ushort* ptr1 = img.ptr<ushort>(i);
		const ushort* ptr2 = imgCpy2->ptr<ushort>(i);

		kernel.y = i;
		//for (int j = 0; j < img.cols-kernel.width; j++)
		for (int j = img.cols-kernel.width-1; j >= 0 ; j--)
		{
			kernel.x = j;
			int val = ptr2[j];
			if (val == 0)
			{
				Mat roi = (*imgCpy2)(kernel);
				int sumRoi = sum(roi).val[0];
				if (sumRoi > 0)
					ptr1[j] = imgCpy1->ptr<ushort>(i)[j];
			}
		}
	}
}

/*
img : Mat(500,181, CV_16UC1)

*/
void ccDetection(Mat& img, list<Person>& people, list<Person>& peoplePolar)
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
			float range = (img.rows-(i))*23.3; //binSize
			float thresh = 60000*exp((-0.00065)*range); //low threshold
			if (val < thresh)
				ptr[j] = 0;
		}
	}
	img.copyTo(imgCpy2);
	//grow image using a 5x5 kernel. If it is close to a region with a non zero value
	//then the original value is recovered and updated
	//growRegions(img, &imgCpy2, &imgCpy1);
	//growRegions2(img, &imgCpy2, &imgCpy1);
	
	Mat outDebug = Mat(500, 181, CV_8UC3);	
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
	detectCC(img, &people, &peoplePolar, outDebug);
	if (debug > DEBUG_MED)
	{
		imshow("Thresholds", outDebug);
			//print out at frame 35
		if (frames == 35)
		{
			imwrite("c:/Dropbox/Phd/Individual Studies/KinectDepthSensor/AlternativeSpace/Frame35_lowThreshold.jpg", outDebug);
		}
	}
}

/*
img : Mat(500,181, CV_16UC1);
*/
void nms(const Mat* img, Size sz, list<Point>& locations)
{
	Mat imgCpy = Mat(img->size(), CV_8UC1);
	Rect k = Rect(0,0, sz.width, sz.height);

	int endRow = img->rows - sz.height;
	int endCol = img->cols - sz.width;
	for (int i = 0; i < endRow; i++)
	{
		k.y = i;
		int row = i+(sz.height/2);
		const ushort* ptr = img->ptr<ushort>(row);
		for (int j = 0; j < endCol; j++)
		{
			k.x = j;			
			int val = ptr[j+(sz.width/2)];
			
			float range = (img->rows-(row))*23.3; //binSize
			float thresh = 68103*exp((-0.0004687)*range);

			//convert16to8(img, imgCpy);
			//rectangle(imgCpy, k, Scalar::all(127));
			//imshow("test", imgCpy);
			//waitKey(1);

			if (val > thresh && isMaximum(img, k))
			{
				Point p;
				p.x = k.x+(k.width/2);
				p.y = k.y+(k.height/2);
				locations.push_back(p);
			}
		}
	}
}


/*
Arg 1: 0:Video; 1:live
Arg 2: 0:No Record; 1:Record
*/
int main(int argc, char* argv[])
{
	//ofstream outKinect[NUM_SENSORS];
	//outKinect[0].open("D:/kinect0.txt", ios::out);
	//outKinect[1].open("D:/kinect1.txt", ios::out);
	//outKinect[2].open("D:/kinect2.txt", ios::out);



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
	paths[0] = "d:/Emilio/Tracking/DataSet/Dset2_workshop/kinect0_calib.oni";
	paths[1] = "d:/Emilio/Tracking/DataSet/Dset2_workshop/kinect1_calib.oni";
	paths[2] = "d:/Emilio/Tracking/DataSet/Dset2_workshop/kinect2_calib.oni";

	//paths[0] = "d:/Emilio/Tracking/DataSet/Cuantization_515.oni";

	ActivityMap_Utils actMapCreator(NUM_SENSORS);

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

	namedWindow(windMoA);
	Mat *activityMap, *activityMap_Back;
	Mat whiteBack, colorMap;
	Mat background = Mat(actMapCreator.getResolution(), CV_8UC3);
	//int height = sqrtf(powf(actMapCreator.getResolution().width/2,2.0) + powf(actMapCreator.getResolution().height,2.0));
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

	for (int i = 0; i < NUM_SENSORS; i++)
	{
		depthImages[i] = Mat(XN_VGA_Y_RES, XN_VGA_X_RES, CV_8UC3);
		rgbImages[i] = Mat(XN_VGA_Y_RES, XN_VGA_X_RES, CV_8UC3);
		depthMat[i] = Mat(XN_VGA_Y_RES, XN_VGA_X_RES, CV_16U);
		pointsFore2D[i] = new XnPoint3D[MAX_FORGROUND_POINTS];
		numberOfForegroundPoints[i] = 0;
	}

	Point centres[10];
	double angles[10];
	
	bool first = true;
	int cont = 0;
	clock_t total = 0;


//	bool printPoints = false;
	int TotalFrames_4 = 1670;
	int TotalFrames_2 = 1790;

	Mat polar = Mat(500,181, CV_16UC1);
	Mat polarAlt= Mat(500,181, CV_16UC1);
	Mat polarAlt_smooth= Mat(500,181, CV_16UC1);
	Mat tmp = Mat(500, 181, CV_8UC1);
	Mat polarAlt_ = Mat(500, 181, CV_8UC1);
	Mat polarAlt_smooth_ = Mat(500, 181, CV_8UC1);
	Mat polar_ = Mat(500, 181, CV_8UC1);
	Mat polarText = Mat(500, 360, CV_8UC1);
	Mat m = Mat(polarAlt_smooth_.size(), CV_8UC3);

	
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
	int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;

	//Size of kernel;
	int kWidth = 5;
	int kHeight = 27;
	Size kSize = Size(kWidth, kHeight);
	int sWidth = 21;
	int sHeight = 21;

	//DEBUG
	DEPTH_STEP = actMapCreator.depthStep;
	X_STEP = actMapCreator.xStep;

	list<Person> people, peoplePolar;
	while (!bShouldStop && frames < TotalFrames_2)
	{
		
		Utils::initMat1s(polarAlt, 0);
		Utils::initMat1s(polar, 0);

		cout << "Frame: " << frames << endl;
		for (int i = 0; i < NUM_SENSORS; i++)
			kinects[i].waitAndUpdate();
		
		list<Point> locations;
		for (int i = 0; i < NUM_SENSORS;  i++)
		{
//			kinects[i].turnUpsideDown(depthMaps_ud[i], rgbMaps_ud[i]);
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

	//	if (frames > 330)
	//	{
			int nPoints  = 0;
			if (bgComplete && trans) //Trans must be true
			{
				cont++;
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

					
						for (int c = 0; c < numberOfForegroundPoints[i]; c++)
						{
							XnPoint3D* p = &(pointsFore2D[i][c]); 
							uchar* ptr = rgbImages[i].ptr<uchar>(p->Y);
							ptr[3*(int)p->X] = 0;
							ptr[(3*(int)p->X)+1] = 0;
							ptr[(3*(int)p->X)+2] = 255;
							//cout << c << " ";
						}
					

				}
				for (int i = 0; i < NUM_SENSORS; i++)
				{
					points3D[i] = kinects[i].arrayBackProject(pointsFore2D[i], numberOfForegroundPoints[i]);
					kinects[i].transformArray(points3D[i], numberOfForegroundPoints[i]);

					//Create alternative representation
					updatePolarAlternateive(&polarAlt, &polar, points3D[i], numberOfForegroundPoints[i]);				
					updateActivityMap(*activityMap, *activityMap_Back, &actMapCreator, points3D[i], numberOfForegroundPoints[i], pointsFore2D[i], rgbMaps[i]);
				}
				if (nPoints > 0)
				{
					//Convolve the image with a gaussian kerel
					//Size sSize = Size(sWidth, sHeight); //kernel size for convolution
					
					uniformConvolution(&polarAlt, polarAlt_smooth, kSize); //smooth the image
					ccDetection(polarAlt_smooth, people, peoplePolar); //Connected component detection


					convert16to8(&polarAlt_smooth, polarAlt_smooth_);
					convert16to8(&polarAlt, polarAlt_);
					convert16to8(&polar, polar_);

					if (debug > DEBUG_LOW)
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
						imshow("comparison", imgDebug);
					}

					list<Person>::iterator iterLocs = people.begin();
					list<Person>::iterator iterPolar = peoplePolar.begin();
					while(iterLocs != people.end())
					{
						Person p = *iterLocs;
						Person pPolar = *iterPolar;
						//p.height *= 2;
						circle(polarAlt_smooth_, p.mean, 2, Scalar::all(0), -1);
						ellipse(polarAlt_smooth_, p.mean, Size(p.sigmaX*2, p.sigmaY*2), 0,0,360, Scalar::all(0));

						//Mean of the distribution
						Point cMoA, pMean;
						cMoA = convertBack(&p.mean);

						pMean.x = actMapCreator.findCoordinate(cMoA.x, ActivityMap_Utils::MIN_X, ActivityMap_Utils::MAX_X, actMapCreator.xStep);
						int y = actMapCreator.findCoordinate(cMoA.y, ActivityMap_Utils::MIN_Z, ActivityMap_Utils::MAX_Z, actMapCreator.depthStep);
						pMean.y = (YRes-1) - y; //flip around X axis.

						

						//Error propagation
						float sigmaAlphaRad = pPolar.sigmaX*CV_PI/180;
						float sigmaRange = pPolar.sigmaY;
						float alpha = pPolar.mean.x*CV_PI/180; //rad
						float range = pPolar.mean.y; //mm

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
						int sigmaX = sqrtf( powf(cosf(alpha)*sigmaRange/20,2) +  powf(range*sinf(alpha)*sigmaAlphaRad/20,2));
						int sigmaY = sqrtf( powf(sinf(alpha)*sigmaRange/20,2) + powf(range*cosf(alpha)*sigmaAlphaRad/20,2));
						//end Option2



						circle(*activityMap, pMean, 2, Scalar(0,0,255));
						
						ellipse(*activityMap, pMean, Size(sigmaY*3, sigmaX*3), -p.mean.x, 0, 360, Scalar(0,0,255));

						iterLocs++;
						iterPolar++;
					}
					people.clear();
					peoplePolar.clear();


				}

				////BEGIN DEBUG
				////recover in a file the raw data
				//if (frames == 34 || frames == 35)
				//{
				//	ofstream outFrame;
				//	if (frames == 34)
				//	{
				//		imwrite("d:/Frame34.jpg", polarAlt_smooth_);
				//		outFrame.open("d:/Frame34.txt");
				//	}
				//	else
				//	{
				//		imwrite("d:/Frame35.jpg", polarAlt_smooth_);
				//		outFrame.open("d:/Frame35.txt");
				//	}
				//	for (int i = 0; i < polarAlt_smooth.rows; i++)
				//	{
				//		ushort *ptr = polarAlt_smooth.ptr<ushort>(i);
				//		for (int j = 0; j < polarAlt_smooth.cols; j++)
				//		{
				//			outFrame << (int)ptr[j] << " " ;
				//		}
				//		outFrame << endl;
				//	}
				//	outFrame.close();
				//}

				////END DEBUG
	

				if (deleteBG)
					imshow(windMoA, *activityMap);
				else
					imshow(windMoA, *activityMap_Back);
			
					for (int i = 0; i < polarAlt_smooth_.rows; i++)
					{
						uchar* p = polarAlt_smooth_.ptr<uchar>(i);
						uchar* s = m.ptr<uchar>(i);
						for (int j = 0; j < polarAlt_smooth_.cols; j++)
						{
							s[3*j] = p[j];
							s[3*j+1] = p[j];
							s[3*j+2] = p[j];
						}
					}



					if (debug > DEBUG_LOW)
						addGrid(polarAlt_smooth_, kSize);
					
					cvSetMouseCallback(windMoA, selectPoint_callBack, (Mat*)activityMap);
					Mat** images = new Mat*[2];
					images[0] = &m;
					images[1] = activityMap;
					cvSetMouseCallback(windPolarSmooth, selectPoint2_callBack, images);

					imshow(windPolarSmooth, m);
					
					if (recordOut == 1)
					{
						w << m;
						w1 << *activityMap;
						//w <<  depthImages[1];
					}
			}
			else
			{
			
				actMapCreator.createActivityMap(kinects, depthMaps, rgbMaps, trans, background, frames);

				if (recordOut == 1 && trans)
						w << background;
				imshow("Activity Map", background);
			}

		

		imshow(windPolarName, polar_);
		//imshow("Polar Alt Smooth", polarAlt_smooth_);
		imshow("Polar Alt", polarAlt_);
		imshow("rgb0", rgbImages[0]);
		imshow("rgb1", rgbImages[1]);
		imshow("rgb2", rgbImages[2]);
		int c = waitKey(waitTime);
		//if (frames == 284)
		//	c = 13;

		switch (c)
		{
		case 119: //(w) increment width of the kernel
			{
				if ((kWidth + 2) < polarAlt.cols)
				{
					kWidth += 2;
					cout << "New kernel size: " << kWidth << ", " << kHeight << endl;
				}
				else
				{
					cout << "kernel too wide" << endl;
				}
				break;
			}
		case 115: //(s) decrease width of the kernel
			{
				if ((kWidth - 2) > 0)
				{
					kWidth -= 2;
					cout << "New kernel size: " << kWidth << ", " << kHeight << endl;
				}
				else
				{
					cout << "kernel width cannot be 0" << endl;
				}
				break;
			}
		case 104: //(h) increment height of the kernel
			{
				if ((kHeight + 2) < polarAlt.rows)
				{
					kHeight += 2;
					cout << "New kernel size: " << kWidth << ", " << kHeight << endl;
				}
				else
				{
					cout << "kernel too high" << endl;
				}
				break;
			}
		case 110: //(n) decrease height of the kernel
			{
				if ((kHeight - 2) > 0)
				{
					kHeight -= 2;
					cout << "New kernel size: " << kWidth << ", " << kHeight << endl;
				}
				else
				{
					cout << "kernel height cannot be 0 or less" << endl;
				}
				break;
			}



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
		case 13: //enter
			{
				if (waitTime == 0)
					waitTime=1;
				else
				{
					//imwrite("c:/Dropbox/Phd/Individual Studies/KinectDepthSensor/AlternativeSpace/MoA_Detection_Good.jpg", *activityMap);
					//imwrite("c:/Dropbox/Phd/Individual Studies/KinectDepthSensor/AlternativeSpace/RemapPolarSpace_Detection_Good.jpg", polarAlt_smooth_);
					//imwrite("c:/Dropbox/Phd/Individual Studies/KinectDepthSensor/AlternativeSpace/RemapSpace.jpg", polarAlt_);
					//imwrite("c:/Dropbox/Phd/Individual Studies/KinectDepthSensor/AlternativeSpace/Detection_MoA.jpg", *activityMap);
					
					/*imwrite("c:/Dropbox/Phd/AnnualMonitoringReport/images/depth1.jpg", depthImages[0]);
					imwrite("c:/Dropbox/Phd/AnnualMonitoringReport/images/depth2.jpg", depthImages[1]);
					imwrite("c:/Dropbox/Phd/AnnualMonitoringReport/images/depth3.jpg", depthImages[2]);
					imwrite("c:/Dropbox/Phd/AnnualMonitoringReport/images/rgb1.jpg", rgbImages[0]);
					imwrite("c:/Dropbox/Phd/AnnualMonitoringReport/images/rgb2.jpg", rgbImages[1]);
					imwrite("c:/Dropbox/Phd/AnnualMonitoringReport/images/rgb3.jpg", rgbImages[2]);
					imwrite("c:/Dropbox/Phd/AnnualMonitoringReport/images/MoA.jpg", *activityMap);*/
					
					waitTime=0;
				}
			//	deleteBG = !deleteBG;
			//	printPoints = true;
				break;
			}
		case 2490368:
			{
				tilt += 5;
				for (int i = 0; i < NUM_SENSORS; i++)
					kinects[i].tilt(tilt);
				cout << "tilt: " << tilt << endl;
				break;
			}
		case 2621440:
			{
				tilt -= 5;
				for (int i = 0; i < NUM_SENSORS; i++)
					kinects[i].tilt(tilt);
				cout << "tilt: " << tilt << endl;
				break;
			}
		}
		frames++;
	}

	////Calculation of the variance
	//vector<vector<float>>::iterator iterPeople = peopleRange.begin();
	//int nP = 1;
	//while (iterPeople != peopleRange.end())
	//{
	//	vector<float> person = *iterPeople;

	//	vector<float>::iterator iterPerson = person.begin();
	//	float r = 0;
	//	while (iterPerson != person.end())
	//	{
	//		float range = *iterPerson;
	//		r += range;
	//		iterPerson++;
	//	}
	//	float meanRange = r/person.size();
	//	float rr = 0;
	//	iterPerson = person.begin();
	//	while (iterPerson != person.end())
	//	{
	//		float range = *iterPerson;
	//		rr += powf(range-meanRange, 2);
	//		iterPerson++;
	//	}
	//	float stdRange = sqrtf(rr/person.size());

	//	cout << "Person " << nP << ". Mean: " << meanRange << ". Std: " << stdRange << endl;
	//	outStd << meanRange << " " << stdRange << endl;

	//	nP++;
	//	iterPeople++;
	//}


	
	for (int i = 0; i < NUM_SENSORS; i++)
	{
		kinects[i].stopDevice();
  		kinects[i].shutDown();
	}
	return 0;

}