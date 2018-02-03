#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/point_cloud2_iterator.h>

#include <stdio.h>
#include <math.h>       /* fmod */
#include <opencv2/opencv.hpp>

#include <iostream>
#include <sstream>
#include <stdint.h>


using namespace sensor_msgs;
using namespace cv;
using namespace std;

ros::Publisher pub;
bool rotation_set = false;
int p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;

int getH(int r, int g, int b){

    int max,min,delta;

    if(r >= g && r >= b)
    {
        max = r;
    }

    if(g >= r && g >= b)
    {
        max = g;
    }

    if(b >= r && b >= g) {

        max = b;
    }

    if(r <= g && r <= b)
    {
        min = r;
    }

    if(g <= r && g <= b)
    {
        min = g;
    }

    if(b <= r && b <= g) {
        min = b;
    }

    delta = max - min;

    if(delta == 0){
        return 0;
    }


    int result;

    if(max == r){
        result = (int)((60/1.41)*(fmod(((g-b)/(float)delta),6)))%256;
    }

    if(max == g){
        result = (int)((60/1.41)*(((b-r)/(float)delta+2)))%256;
    }

    if(max == b){
        result = (int)((60/1.41)*(((r-g)/(float)delta+4)))%256;
    }

    if(result < 0 ){
        return 256-result;
    }
    else return result;
}

int getS(int r, int g, int b){

    int max,min,delta;

    if(r >= g && r >= b)
    {
        max = r;
    }

    if(g >= r && g >= b)
    {
        max = g;
    }

    if(b >= r && b >= g) {
        max = b;
    }

    if(r <= g && r <= b)
    {
        min = r;
    }

    if(g <= r && g <= b)
    {
        min = g;
    }

    if(b <= r && b <= g) {
        min = b;
    }

    delta = max - min;

    if(max == 0){
        return 0;
    }
    else{
        return (int)((delta*1.0/max)*255);
    }
}

int getV(int r, int g, int b){

    int max,min,delta;

    if(r >= g && r >= b)
    {
        return r;
    }

    if(g >= r && g >= b)
    {
        return g;
    }

    else{ //if(b >= r && b >= g) {
        return b;
    }
}

void filtergreen(const sensor_msgs::PointCloud2Ptr& input){
    for (int i=0; i < input -> height; i++)
    {
        for (int j=0; j < input -> width; j++)
        {

            float x,y,z;
            x = y = z = 0;

            unsigned char *pt;

            //unsigned int p = input -> data + input -> row_step*i + j*input -> point_step;

            pt = (input -> data).data() + input -> row_step*i + j*input -> point_step;

            //cout << (int)input -> data[input -> row_step*i + j*input -> point_step] << endl;
            //cout << (int)input -> data[input -> row_step*i + j*input -> point_step+1] << endl;
            //cout << (int)input -> data[input -> row_step*i + j*input -> point_step+2] << endl;
            //cout << (int)input -> data[input -> row_step*i + j*input -> point_step+3] << endl;

            memcpy(&x,pt,4);

            memcpy(&y,pt+4,4);

            memcpy(&z,pt+8,4);

            int r,g,b,a;
            r = g = b = a = 0;

            // representation is in BRGA
            b = input -> data[input -> row_step*i + j*input -> point_step + 16];
            g = input -> data[input -> row_step*i + j*input -> point_step + 17];
            r = input -> data[input -> row_step*i + j*input -> point_step + 18];
            a = input -> data[input -> row_step*i + j*input -> point_step + 19];

            int h,s,v;
            h = s = v = 0;

            h = getH(r,g,b);
            s = getS(r,g,b);
            v = getV(r,g,b);

            //cout << " r:" << r << " g: " << g << " b: " << b << endl;
            //cout << " h:" << h << " s: " << s << " v: " << v << endl;


            if(h >=70 && h <=100 && s >= 36 && s <= 115 && v >= 92 && v <= 255){ //green HSV values
                /*
                input -> data[input -> row_step*i + j*input -> point_step + 16] = 0;
                input -> data[input -> row_step*i + j*input -> point_step + 17] = 0;
                input -> data[input -> row_step*i + j*input -> point_step + 18] = 255;
                */
            }
            else{ // send them into the void

                //x nan
                input -> data[input -> row_step*i + j*input -> point_step] = 0;
                input -> data[input -> row_step*i + j*input -> point_step + 1] = 0;
                input -> data[input -> row_step*i + j*input -> point_step + 2] = 192;
                input -> data[input -> row_step*i + j*input -> point_step + 3] = 127;
                //y nan
                input -> data[input -> row_step*i + j*input -> point_step + 4] = 0;
                input -> data[input -> row_step*i + j*input -> point_step + 5] = 0;
                input -> data[input -> row_step*i + j*input -> point_step + 6] = 192;
                input -> data[input -> row_step*i + j*input -> point_step + 7] = 127;
                //z nan
                input -> data[input -> row_step*i + j*input -> point_step + 8] = 0;
                input -> data[input -> row_step*i + j*input -> point_step + 9] = 0;
                input -> data[input -> row_step*i + j*input -> point_step + 10] = 192;
                input -> data[input -> row_step*i + j*input -> point_step + 11] = 127;

            }

            //cout << "i:" << i << " j:" << j << endl;
            //cout << "x:" << x << " y:" << y << " z:" << z << endl;
            //cout << "r:" << r << " g:" << g << " b:" << b << endl;

            //cout << input -> data[input -> row_step*i + j*input -> point_step] << endl;
        }
    }
}

void filterpaper(const sensor_msgs::PointCloud2Ptr& input){


    bool yellowdetect[input -> height * input -> width];
    bool yellow_todelete[input -> height * input -> width];

    for (int i=0; i < input -> height; i++)
    {
        for (int j=0; j < input -> width; j++)
        {

            float x,y,z;
            x = y = z = 0;

            unsigned char *pt;

            //unsigned int p = input -> data + input -> row_step*i + j*input -> point_step;

            pt = (input -> data).data() + input -> row_step*i + j*input -> point_step;

            //cout << (int)input -> data[input -> row_step*i + j*input -> point_step] << endl;
            //cout << (int)input -> data[input -> row_step*i + j*input -> point_step+1] << endl;
            //cout << (int)input -> data[input -> row_step*i + j*input -> point_step+2] << endl;
            //cout << (int)input -> data[input -> row_step*i + j*input -> point_step+3] << endl;

            memcpy(&x,pt,4);

            memcpy(&y,pt+4,4);

            memcpy(&z,pt+8,4);

            int r,g,b,a;
            r = g = b = a = 0;
            // representation is in BRGA
            b = input -> data[input -> row_step*i + j*input -> point_step + 16];
            g = input -> data[input -> row_step*i + j*input -> point_step + 17];
            r = input -> data[input -> row_step*i + j*input -> point_step + 18];
            a = input -> data[input -> row_step*i + j*input -> point_step + 19];

            int h,s,v;
            h = s = v = 0;

            h = getH(r,g,b);
            s = getS(r,g,b);
            v = getV(r,g,b);

            //cout << " r:" << r << " g: " << g << " b: " << b << endl;
            //cout << " h:" << h << " s: " << s << " v: " << v << endl;


            if((h >=25 && h <= 35 && s >= 50 && s <= 86 && v >= 190 && v <= 230)){ //yellow paper HSV values

                yellowdetect[i*640+j] = true;

            }
            else{ // send them into the void

                yellowdetect[i*640+j] = false;

                //x nan
                input -> data[input -> row_step*i + j*input -> point_step] = 0;
                input -> data[input -> row_step*i + j*input -> point_step + 1] = 0;
                input -> data[input -> row_step*i + j*input -> point_step + 2] = 192;
                input -> data[input -> row_step*i + j*input -> point_step + 3] = 127;
                //y nan
                input -> data[input -> row_step*i + j*input -> point_step + 4] = 0;
                input -> data[input -> row_step*i + j*input -> point_step + 5] = 0;
                input -> data[input -> row_step*i + j*input -> point_step + 6] = 192;
                input -> data[input -> row_step*i + j*input -> point_step + 7] = 127;
                //z nan
                input -> data[input -> row_step*i + j*input -> point_step + 8] = 0;
                input -> data[input -> row_step*i + j*input -> point_step + 9] = 0;
                input -> data[input -> row_step*i + j*input -> point_step + 10] = 192;
                input -> data[input -> row_step*i + j*input -> point_step + 11] = 127;

            }

            //cout << "i:" << i << " j:" << j << endl;
            //cout << "x:" << x << " y:" << y << " z:" << z << endl;
            //cout << "r:" << r << " g:" << g << " b:" << b << endl;

            //cout << input -> data[input -> row_step*i + j*input -> point_step] << endl;


        }
    }

    //detect unsignificant points
    for (int i=0; i < input -> height; i++) {
        for (int j = 0; j < input->width; j++) {
            if(yellowdetect[i*640+j] == 1){
                if(!(j==0 | j==479 | i%480==0 | i % 480 == 479)){ // not in border
                    if(yellowdetect[i*640+j-1] == false ) yellow_todelete[i*640+j] = true; // left
                    else if (yellowdetect[i*640+j+1] == false ) yellow_todelete[i*640+j] = true; // right
                    else if (yellowdetect[i*640+j-input -> width] == false ) yellow_todelete[i*640+j] = true; // up
                    else if (yellowdetect[i*640+j+input -> width] == false ) yellow_todelete[i*640+j] = true; // down
                    else yellow_todelete[i*640+j] = false;
                }
            }

        }
    }

    // SEND THEM TO THE VOID
    for (int i=0; i < input -> height; i++) {
        for (int j = 0; j < input->width; j++) {
            if((j==0 | j==479 | i%480==0 | i % 480 == 479 | yellow_todelete[i*640+j])){
                //x nan
                input -> data[input -> row_step*i + j*input -> point_step] = 0;
                input -> data[input -> row_step*i + j*input -> point_step + 1] = 0;
                input -> data[input -> row_step*i + j*input -> point_step + 2] = 192;
                input -> data[input -> row_step*i + j*input -> point_step + 3] = 127;
                //y nan
                input -> data[input -> row_step*i + j*input -> point_step + 4] = 0;
                input -> data[input -> row_step*i + j*input -> point_step + 5] = 0;
                input -> data[input -> row_step*i + j*input -> point_step + 6] = 192;
                input -> data[input -> row_step*i + j*input -> point_step + 7] = 127;
                //z nan
                input -> data[input -> row_step*i + j*input -> point_step + 8] = 0;
                input -> data[input -> row_step*i + j*input -> point_step + 9] = 0;
                input -> data[input -> row_step*i + j*input -> point_step + 10] = 192;
                input -> data[input -> row_step*i + j*input -> point_step + 11] = 127;
            }

        }
    }

}

void cloud_cb (const sensor_msgs::PointCloud2Ptr& input){

    // Create a container for the data.
    sensor_msgs::PointCloud2 output;
    sensor_msgs::PointCloud2 yellow;
    sensor_msgs::PointCloud2 green;

    // Do data processing here...

    if(rotation_set){
        filtergreen(input);
    }else{
        filterpaper(input);
    }

    //debug
    //for(int i=0; i < 4; i++)
    //    cout << input -> fields[i].name << " - " << (int)(input -> fields[i].datatype) << " - " << input -> fields[i].count << " - " << input -> fields[i].offset << endl;

    // Publish the data.

    output = *input;

    //cout << output -> data[input -> row_step + input -> point_step + 11] << endl;

    pub.publish (output);
}

int main (int argc, char** argv){
    // Initialize ROS
    ros::init (argc, argv, "jaco_kinect_calibration_node");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/output/filtered_cloud", 1);

    // Spin
    ros::spin ();
}