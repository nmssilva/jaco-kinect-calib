#include <stdio.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <sstream>

using namespace cv;
using namespace std;

float verifyCircle(cv::Mat dt, cv::Point2f center, float radius, std::vector<cv::Point2f> & inlierSet)
{
 unsigned int counter = 0;
 unsigned int inlier = 0;
 float minInlierDist = 2.0f;
 float maxInlierDistMax = 100.0f;
 float maxInlierDist = radius/25.0f;
 if(maxInlierDist<minInlierDist) maxInlierDist = minInlierDist;
 if(maxInlierDist>maxInlierDistMax) maxInlierDist = maxInlierDistMax;

 // choose samples along the circle and count inlier percentage
 for(float t =0; t<2*3.14159265359f; t+= 0.05f)
 {
     counter++;
     float cX = radius*cos(t) + center.x;
     float cY = radius*sin(t) + center.y;

     if(cX < dt.cols)
     if(cX >= 0)
     if(cY < dt.rows)
     if(cY >= 0)
     if(dt.at<float>(cY,cX) < maxInlierDist)
     {
        inlier++;
        inlierSet.push_back(cv::Point2f(cX,cY));
     }
 }

 return (float)inlier/float(counter);
}


inline void getCircle(cv::Point2f& p1,cv::Point2f& p2,cv::Point2f& p3, cv::Point2f& center, float& radius)
{
  float x1 = p1.x;
  float x2 = p2.x;
  float x3 = p3.x;

  float y1 = p1.y;
  float y2 = p2.y;
  float y3 = p3.y;

  // PLEASE CHECK FOR TYPOS IN THE FORMULA :)
  center.x = (x1*x1+y1*y1)*(y2-y3) + (x2*x2+y2*y2)*(y3-y1) + (x3*x3+y3*y3)*(y1-y2);
  center.x /= ( 2*(x1*(y2-y3) - y1*(x2-x3) + x2*y3 - x3*y2) );

  center.y = (x1*x1 + y1*y1)*(x3-x2) + (x2*x2+y2*y2)*(x1-x3) + (x3*x3 + y3*y3)*(x2-x1);
  center.y /= ( 2*(x1*(y2-y3) - y1*(x2-x3) + x2*y3 - x3*y2) );

  radius = sqrt((center.x-x1)*(center.x-x1) + (center.y-y1)*(center.y-y1));
}



std::vector<cv::Point2f> getPointPositions(cv::Mat binaryImage)
{
 std::vector<cv::Point2f> pointPositions;

 for(unsigned int y=0; y<binaryImage.rows; ++y)
 {
     //unsigned char* rowPtr = binaryImage.ptr<unsigned char>(y);
     for(unsigned int x=0; x<binaryImage.cols; ++x)
     {
         //if(rowPtr[x] > 0) pointPositions.push_back(cv::Point2i(x,y));
         if(binaryImage.at<unsigned char>(y,x) > 0) pointPositions.push_back(cv::Point2f(x,y));
     }
 }

 return pointPositions;
}


inline void getTheCircle(cv::Mat threshold,cv::Mat frame, string n){
    vector<Point2f> edgePositions;
    edgePositions = getPointPositions(threshold);

    // create distance transform to efficiently evaluate distance to nearest edge
    Mat dt;
    distanceTransform(255-threshold, dt,CV_DIST_L1, 3);

    //TODO: maybe seed random variable for real random numbers.

    unsigned int nIterations = 0;

    Point2f bestCircleCenter;
    float bestCircleRadius;
    float bestCirclePercentage = 0;
    int minRadius = 35;   // TODO: ADJUST THIS PARAMETER TO YOUR NEEDS, otherwise smaller circles wont be detected or "small noise circles" will have a high percentage of completion
    createTrackbar("minRadius", "thr", &minRadius, 100);
    //float minCirclePercentage = 0.2f;
    float minCirclePercentage = 0.05f;  // at least 5% of a circle must be present? maybe more...

    int maxNrOfIterations = edgePositions.size();   // TODO: adjust this parameter or include some real ransac criteria with inlier/outlier percentages to decide when to stop

    for(unsigned int its=0; its< maxNrOfIterations; ++its)
    {
        //RANSAC: randomly choose 3 point and create a circle:
        //TODO: choose randomly but more intelligent, 
        //so that it is more likely to choose three points of a circle. 
        //For example if there are many small circles, it is unlikely to randomly choose 3 points of the same circle.
        unsigned int idx1 = rand()%edgePositions.size();
        unsigned int idx2 = rand()%edgePositions.size();
        unsigned int idx3 = rand()%edgePositions.size();

        // we need 3 different samples:
        if(idx1 == idx2) continue;
        if(idx1 == idx3) continue;
        if(idx3 == idx2) continue;

        // create circle from 3 points:
        Point2f center; float radius;
        getCircle(edgePositions[idx1],edgePositions[idx2],edgePositions[idx3],center,radius);

        // inlier set unused at the moment but could be used to approximate a (more robust) circle from alle inlier
        vector<Point2f> inlierSet;

        //verify or falsify the circle by inlier counting:
        float cPerc = verifyCircle(dt,center,radius, inlierSet);

        // update best circle information if necessary

        if(cPerc >= bestCirclePercentage)
            if(radius >= minRadius)
                {
                    bestCirclePercentage = cPerc;
                    bestCircleRadius = radius;
                    bestCircleCenter = center;
                }

    }

    // draw if good circle was found
    if(bestCirclePercentage >= minCirclePercentage)
        if(bestCircleRadius >= minRadius){
            //circle(frame, bestCircleCenter,bestCircleRadius, Scalar(255,255,0),1);
            circle(threshold, bestCircleCenter,bestCircleRadius, Scalar(255,255,0),1);
        }
    imshow("thr "+n,threshold);
    //imshow(n,frame);
}

inline void process(cv::Mat frame, string n, int iLowH ,int iHighH , int iLowS ,int iHighS ,int iLowV ,int iHighV ,int saturation ,int contrast,int erodeAmount, int dilateAmount){
    Mat f;
    Mat HSV;
    frame.convertTo(f, CV_8UC1, contrast*1.0 / 10, -saturation); 
    Mat threshold;

    cvtColor(f,HSV,CV_BGR2HSV);
    inRange(HSV,Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV),threshold);

    Mat stucElem = getStructuringElement(MORPH_ELLIPSE, Size(2, 2));
    for (int i =0; i< erodeAmount;i++)
        erode(threshold, threshold, stucElem, Point(0, 0), 1);
    for (int i =0; i< dilateAmount;i++)
        dilate(threshold, threshold, stucElem, Point(0, 0), 1);
    imshow("thr "+n,threshold);    
    
    //getTheCircle(threshold,f,n);
}

int main(int argc, char** argv )
{

    int iLowH = 0;
    int iHighH = 15;

    int iLowS = 239; 
    int iHighS = 255;

    int iLowV = 72;
    int iHighV = 207;


    int saturation = 125;
    int contrast = 14;
    int erodeAmount = 3;
    int dilateAmount = 3;
   
    
    //Mat HSV;
    Mat frame;
    //Mat threshold;

    //initialize window 
    while(1){
    	VideoCapture captRefrnc("output.mpg");
    	if (!captRefrnc.isOpened())
    	{
    		cout << "Could not open reference " << "output.mpg" << endl;
    		return -1;
    	}
        /*for(;;){   
            namedWindow("thr", CV_WINDOW_AUTOSIZE);
            createTrackbar("LowH", "thr", &iLowH, 255); //Hue (0 - 179)
            createTrackbar("HighH", "thr", &iHighH, 255);

            createTrackbar("LowS", "thr", &iLowS, 255); //Saturation (0 - 255)
            createTrackbar("HighS", "thr", &iHighS, 255);

            createTrackbar("LowV", "thr", &iLowV, 255);//Value (0 - 255)
            createTrackbar("HighV", "thr", &iHighV, 255);

            createTrackbar("contrast", "thr", &contrast, 20);
            createTrackbar("erodeAmount", "thr", &erodeAmount, 20);
            createTrackbar("dilateAmount", "thr", &dilateAmount, 20);

            if (!captRefrnc.read(frame))             
                break;

            frame.convertTo(frame, CV_8UC1, contrast*1.0 / 10, -saturation); 


            cvtColor(frame,HSV,CV_BGR2HSV);
            inRange(HSV,Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV),threshold);
            
            Mat stucElem = getStructuringElement(MORPH_ELLIPSE, Size(2, 2));
            for (int i =0; i< erodeAmount;i++)
                erode(threshold, threshold, stucElem, Point(0, 0), 1);
            for (int i =0; i< dilateAmount;i++)
                dilate(threshold, threshold, stucElem, Point(0, 0), 1);
            imshow("thr",threshold);    

            getTheCircle(threshold,frame,"orange");
            



            char key = cvWaitKey(10);
            if (key == 27) // ESC
                break;
        }*/
        for(;;){   
            namedWindow("thr", CV_WINDOW_AUTOSIZE);
            

            if (!captRefrnc.read(frame))             
                break;

            process(frame, "Orange", iLowH , iHighH ,iLowS ,iHighS ,iLowV ,iHighV ,saturation ,contrast,erodeAmount,dilateAmount);
            process(frame, "Green", 48 , 96 ,91 ,239 ,51 ,227 ,127 ,15,3,2);
            process(frame, "Pink", 129 , 166 ,92 ,242 ,92 ,255 ,106 ,14,3,2);
            process(frame, "Blue", 92 , 111 ,155 ,255 ,109 ,255 ,200 ,20,5,3);

            char key = cvWaitKey(10);
            if (key == 27) // ESC
                break;
        }
    }
	// Closes all the frames
	destroyAllWindows();
    return 0;
}
