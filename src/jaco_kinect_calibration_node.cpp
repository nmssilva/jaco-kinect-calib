#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/point_cloud2_iterator.h>

#include <stdio.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <sstream>
#include <stdint.h>

using namespace sensor_msgs;
using namespace cv;
using namespace std;

ros::Publisher pub;

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

    if(max == r){
        return 60*(((g-b)/delta)%6);
    }

    if(max == g){
        return 60*(((b-r)/delta)+2);
    }

    else{ //if(max = b){
        return 60*(((r-g)/delta)+4);
    }
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

void cloud_cb (const sensor_msgs::PointCloud2Ptr& input){

    // Create a container for the data.
    sensor_msgs::PointCloud2 output;

    // Do data processing here...

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


            if(h >=35 && h <=79 && s >= 36 && s <= 115 && v >= 92 && v <= 255){
                cout << "FOUND at: " << i << ", " << j << endl;
                input -> data[input -> row_step*i + j*input -> point_step + 16] = 0;
                input -> data[input -> row_step*i + j*input -> point_step + 17] = 0;
                input -> data[input -> row_step*i + j*input -> point_step + 18] = 255;
            }

            //cout << "i:" << i << " j:" << j << endl;
            //cout << "x:" << x << " y:" << y << " z:" << z << endl;
            //cout << "r:" << r << " g:" << g << " b:" << b << endl;

            //cout << input -> data[input -> row_step*i + j*input -> point_step] << endl;
        }
    }

    //debug
    //for(int i=0; i < 4; i++)
    //    cout << input -> fields[i].name << " - " << (int)(input -> fields[i].datatype) << " - " << input -> fields[i].count << " - " << input -> fields[i].offset << endl;

    // Publish the data.

    output = *input;

    pub.publish (output);
}

int main (int argc, char** argv){
    // Initialize ROS
    ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/output/filtered_cloud", 1);

    // Spin
    ros::spin ();
}