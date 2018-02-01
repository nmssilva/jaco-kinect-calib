#include <stdio.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <sstream>

//#include <ros/ros.h>
//#include <ros/package.h>

//using namespace ros;
using namespace cv;
using namespace std;


inline Mat findSurface(Mat src){
    
}


inline Mat findLargestBlob(Mat src){
	int largest_area=0;
	int largest_contour_index=0;
	Rect bounding_rect;

	Mat thr(src.rows,src.cols,CV_8UC1); 
	Mat dst(src.rows,src.cols,CV_8UC1,Scalar::all(0));

	vector<vector<Point> > contours; // Vector for storing contour
	vector<Vec4i> hierarchy;

	findContours( thr, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE ); // Find the contours in the image

	for( int i = 0; i< contours.size(); i++ ) // iterate through each contour. 
		{
		double a=contourArea( contours[i],false);  //  Find the area of contour
		if(a>largest_area){
		largest_area=a;
		largest_contour_index=i;                //Store the index of largest contour
		bounding_rect=boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
		}

	}

	Scalar color( 255,255,255);
	drawContours( dst, contours,largest_contour_index, color, CV_FILLED, 8, hierarchy ); // Draw the largest contour using previously stored index.
	
	return dst;
}

inline Mat process(cv::Mat frame, string n, int iLowH ,int iHighH , int iLowS ,int iHighS ,int iLowV ,int iHighV ){
    Mat f;
    Mat HSV;
    frame.convertTo(f, CV_8UC1, 1, 1); 
    Mat threshold;

    cvtColor(f,HSV,CV_BGR2HSV);
    inRange(HSV,Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV),threshold);

    Mat stucElem = getStructuringElement(MORPH_ELLIPSE, Size(2, 2));
    
    medianBlur ( threshold, threshold, 5 );

    Mat labels, stats, centroids;
	connectedComponentsWithStats(threshold, labels, stats, centroids, 8, CV_32S);
    
    Mat tmp= findLargestBlob(threshold);
    medianBlur ( tmp, tmp, 5 );

	imshow(n,tmp);

    return tmp;    
    
    //getTheCircle(threshold,f,n);
}

int main(int argc, char** argv )
{

    //ros::init(argc, argv, "processVideo");
    /*
    int iLowH = 0;
    int iHighH = 79;

    int iLowS = 0; 
    int iHighS = 114;

    int iLowV = 182;
    int iHighV = 245;
    */

    
   
    
    //Mat HSV;
    
    //Mat threshold;

    //initialize window 
    while(1){
        Mat frame;
        //VideoCapture captRefrnc(ros::package::getPath("jaco_kinect_calib")+"/src/output.mpg");
    	//VideoCapture captRefrnc(ros::package::getPath("jaco_kinect_calib")+"/src/output.mpg");

        string filename = "/home/mikael/catkin_ws/src/jaco_kinect_calibration/output.mpg";

    	VideoCapture captRefrnc(filename);

    	if (!captRefrnc.isOpened())
    	{
    		cout << "Could not open reference " << filename << endl;
    		return -1;
    	}
        for(;;){ 
            /*          
            namedWindow("thr", CV_WINDOW_AUTOSIZE);
            createTrackbar("LowH", "thr", &iLowH, 255); //Hue (0 - 179)
            createTrackbar("HighH", "thr", &iHighH, 255);

            createTrackbar("LowS", "thr", &iLowS, 255); //Saturation (0 - 255)
            createTrackbar("HighS", "thr", &iHighS, 255);

            createTrackbar("LowV", "thr", &iLowV, 255);//Value (0 - 255)
            createTrackbar("HighV", "thr", &iHighV, 255);

            */

            if (!captRefrnc.read(frame))             
                break;
            //process(frame, "Orange", iLowH , iHighH ,iLowS ,iHighS ,iLowV ,iHighV );
            
            Mat orange, green, pink, blue, surface;
            
            orange = process(frame, "Orange",0,23,119,255,142,255);
            green = process(frame, "Green",35,79,36,115,92,255);
            pink = process(frame, "Pink", 128,236,59,143,116,255);
            blue = process(frame, "blue", 87,122,63,129,92,255);

            surface = process(frame, "surface", 0,52,54,129,181,255);

            Mat final;

            bitwise_or(orange,green,final);
            bitwise_or(final,pink,final);
            bitwise_or(final,blue,final);
            bitwise_or(final,surface,final);
            
            frame.copyTo(final, final);
            
            imshow("final",final);
            imshow("original",frame);
            //findSurface(frame);
            char key = cvWaitKey(10);
            if (key == 27) // ESC
                break;
        }
        
    }
	// Closes all the frames
	destroyAllWindows();
    return 0;
}