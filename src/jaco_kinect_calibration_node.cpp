#include <ros/ros.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//TF
#include <tf/transform_listener.h>

#include <sensor_msgs/point_cloud2_iterator.h>

#include <stdio.h>
#include <math.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdint.h>

#include <unistd.h> //sleep

# define M_PI           3.14159265358979323846  /* pi */

using namespace sensor_msgs;
using namespace cv;
using namespace std;

ros::Publisher pubgreen;
ros::Publisher pubpaper;

bool rotation_set = false;
bool pitch_set = false;
int calib_rot_count = 0;
int calib_tf_count = 0;
float p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
float p1xm,p1ym,p1zm,p2xm,p2ym,p2zm,p3xm,p3ym,p3zm;
double tx,ty,tz,yaw,pitch,roll;
double txm,tym,tzm;

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

    bool greendetect[input -> height * input -> width];
    bool green_todelete[input -> height * input -> width];

    for (int i=0; i < input -> height; i++)
    {
        for (int j=0; j < input -> width; j++)
        {

            float x,y,z;
            x = y = z = 0;

            unsigned char *pt;

            pt = (input -> data).data() + input -> row_step*i + j*input -> point_step;

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

            if(h >=70 && h <=100 && s >= 36 && s <= 115 && v >= 92 && v <= 255){ //green HSV values

                greendetect[i*640+j] = true;

            }
            else{ // send them into the void

                greendetect[i*640+j] = false;

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

    //detect unsignificant points
    for (int i=0; i < input -> height; i++) {
        for (int j = 0; j < input->width; j++) {
            if(greendetect[i*640+j] == 1){
                if(!(j==0 | j==479 | i%480==0 | i % 480 == 479)){ // not in border
                    if(greendetect[i*640+j-1] == false ) green_todelete[i*640+j] = true; // left
                    else if (greendetect[i*640+j+1] == false ) green_todelete[i*640+j] = true; // right
                    else if (greendetect[i*640+j-input -> width] == false ) green_todelete[i*640+j] = true; // up
                    else if (greendetect[i*640+j+input -> width] == false ) green_todelete[i*640+j] = true; // down
                    else green_todelete[i*640+j] = false;
                }
            }

        }
    }

    // SEND THEM TO THE VOID
    for (int i=0; i < input -> height; i++) {
        for (int j = 0; j < input->width; j++) {
            if((j==0 | j==479 | i%480==0 | i % 480 == 479 | green_todelete[i*640+j])){
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

    //

    double r;
    double d = 9999;
    float gx,gy,gz;

    //get nearest point
    for (int i=0; i < input -> height; i++) {
        for (int j = 0; j < input->width; j++) {

            float x, y, z;
            x = y = z = 0;

            unsigned char *pt;

            pt = (input->data).data() + input->row_step * i + j * input->point_step;

            memcpy(&x, pt, 4);

            memcpy(&y, pt + 4, 4);

            memcpy(&z, pt + 8, 4);

            r = sqrt(pow(x,2)+pow(y,2)+pow(z,2));

            if(d > r){
                d = r;
                gx = x;
                gy = y;
                gz = z;
            }
        }
    }


    // stay with the nearest point
    // delete the others

    for (int i=0; i < input -> height; i++) {
        for (int j = 0; j < input->width; j++) {

            float x, y, z;
            x = y = z = 0;

            unsigned char *pt;

            pt = (input->data).data() + input->row_step * i + j * input->point_step;

            memcpy(&x, pt, 4);

            memcpy(&y, pt + 4, 4);

            memcpy(&z, pt + 8, 4);

            // SEND THEM TO THE VOID
            if(x!=gx | y!=gy | z!=gz){
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


    //check is green point is below the paper, if yes flip 180 degrees
    if(!pitch_set){
        if(gz < p1zm){
            pitch = M_PI;
        }
        else{
            pitch = 0;
        }

        pitch_set=true;

        cerr << "\033[1;32mPlease wait...\033[0m"<<endl<<"\033[1;31m0%\033[0m                     \033[1;37m100%\033[0m\n  \e[A ";

    }

    // update new green point coords

    double gxp,gyp,gzp,tmp;
    //yaw, gx = x, gy = y
    gyp = gy*cos(yaw)-gx*sin(yaw);
    gxp = gx*cos(yaw)+gy*sin(yaw);
    gx = gxp;
    gy = gyp;

    //pitch, gz = x, gx = y
    gxp = gx*cos(pitch)-gz*sin(pitch);
    gzp = gz*cos(pitch)+gx*sin(pitch);
    gz=gzp;
    gx=gxp;

    //roll, gy = x, gz = y
    gyp = gy*cos(roll)-gz*sin(roll);
    gzp = gz*cos(roll)+gy*sin(roll);
    gy=gyp;
    gz=gzp;

    //get tf values now
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try{
        listener.waitForTransform("/m1n6s200_link_6", "/m1n6s200_link_base",
                                  ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/m1n6s200_link_6", "/m1n6s200_link_base",
                                 ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        //ROS_ERROR("%s",ex.what());
    }

    tx = abs(transform.getOrigin().x() - gx);
    ty = abs(transform.getOrigin().y() - gy);
    tz = abs(transform.getOrigin().z() - gz);

    calib_tf_count++;
    if(calib_tf_count < 20){
        cerr << "\033[1;37m█\033[0m";
        txm = (txm*calib_tf_count+tx)/(calib_tf_count+1);
        tym = (tym*calib_tf_count+ty)/(calib_tf_count+1);
        tzm = (tzm*calib_tf_count+tz)/(calib_tf_count+1);
    }
    else{
        //final calibration values
        txm+=0.07;
        tym=tym/2-0.02; if(tym<0.09) tym*=2;
        roll-=0.15;
        //final prints
        cout << "\n\n\033[1;32mCalibration concluded.\033[0m" << endl;

        cout << "\n\033[1;31myaw: \033[0m" << yaw << endl;
        cout << "\033[1;32mpitch: \033[0m" << pitch << endl;
        cout << "\033[1;33mroll: \033[0m" << roll << endl;

        cout << "\n\033[1;31mtx: \033[0m"  << txm << endl;
        cout << "\033[1;32mty: \033[0m" << tym << endl;
        cout << "\033[1;33mtz: \033[0m" << tzm << endl << endl;

        ofstream myfile;
        myfile.open ("camera_rgb_optical_frame.txt");
        myfile << "   x        y        z       yaw       pitch    roll\n";
        myfile << txm << " " << tym << " " << tzm << " " << yaw << " " << pitch << " " << roll;
        myfile.close();

        exit(0);
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

    //extract p1, p2 and p3.
    float xmin,xmax,ymax;
    xmin = 9999.0;
    xmax = ymax = -9999.0;

    for (int i=0; i < input -> height; i++) {
        for (int j = 0; j < input -> width; j++) {

            float x,y,z;
            x = y = z = 0;

            unsigned char *pt;

            pt = (input -> data).data() + input -> row_step*i + j*input -> point_step;

            memcpy(&x,pt,4);

            memcpy(&y,pt+4,4);

            memcpy(&z,pt+8,4);

            // watch out for opposite coordinate X!=Y
            if(xmin > x){ //p1
                xmin = x;
                p1x = x;
                p1y = y;
                p1z = z;
            }

            if(ymax < y){ //p2
                ymax = y;
                p2x = x;
                p2y = y;
                p2z = z;
            }

            if(xmax < x){ //p3
                xmax = x;
                p3x = x;
                p3y = y;
                p3z = z;
            }

        }
    }

    // SEND THEM TO THE VOID
    for (int i=0; i < input -> height; i++) {
        for (int j = 0; j < input->width; j++) {

            float x,y,z;
            x = y = z = 0;

            unsigned char *pt;

            pt = (input -> data).data() + input -> row_step*i + j*input -> point_step;

            memcpy(&x,pt,4);

            memcpy(&y,pt+4,4);

            memcpy(&z,pt+8,4);

            if(!((x == p1x && y == p1y && z == p1z)|(x == p2x && y == p2y && z == p2z)|(x == p3x && y == p3y && z == p3z))){
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
            else{
                if(x == p1x && y == p1y && z == p1z){ //p1
                    input -> data[input -> row_step*i + j*input -> point_step + 16] = 0;
                    input -> data[input -> row_step*i + j*input -> point_step + 17] = 0;
                    input -> data[input -> row_step*i + j*input -> point_step + 18] = 255;
                }
                if(x == p2x && y == p2y && z == p2z){ //p2
                    input -> data[input -> row_step*i + j*input -> point_step + 16] = 0;
                    input -> data[input -> row_step*i + j*input -> point_step + 17] = 255;
                    input -> data[input -> row_step*i + j*input -> point_step + 18] = 0;
                }
                if(x == p3x && y == p3y && z == p3z){ //p3
                    input -> data[input -> row_step*i + j*input -> point_step + 16] = 255;
                    input -> data[input -> row_step*i + j*input -> point_step + 17] = 0;
                    input -> data[input -> row_step*i + j*input -> point_step + 18] = 0;
                }
            }
        }
    }

    //after sending them to the void we calculate the average of 20 readings

    calib_rot_count++;
    if(calib_rot_count < 20){
        p1xm = (p1xm*calib_rot_count+p1x)/(calib_rot_count+1);
        p1ym = (p1ym*calib_rot_count+p1y)/(calib_rot_count+1);
        p1zm = (p1zm*calib_rot_count+p1z)/(calib_rot_count+1);
        p2xm = (p2xm*calib_rot_count+p2x)/(calib_rot_count+1);
        p2ym = (p2ym*calib_rot_count+p2y)/(calib_rot_count+1);
        p2zm = (p2zm*calib_rot_count+p2z)/(calib_rot_count+1);
        p3xm = (p3xm*calib_rot_count+p3x)/(calib_rot_count+1);
        p3ym = (p3ym*calib_rot_count+p3y)/(calib_rot_count+1);
        p3zm = (p3zm*calib_rot_count+p3z)/(calib_rot_count+1);
    }
    else{
        double d,x;

        //between p1 and p2
        d = sqrt(pow((p2xm-p1xm),2)+pow((p2ym-p1ym),2));
        x = abs(p1zm - p2zm);
        roll = asin(x/d);
        roll = roll*(M_PI/1.8);

        //between p1 and p3
        d = sqrt(pow((p3xm-p1xm),2)+pow((p3ym-p1ym),2));
        x = abs(p1zm - p3zm);
        yaw = asin(x/d);
        yaw = yaw*(M_PI/1.8)*0.01;
    }


}

void cloud_cb (const sensor_msgs::PointCloud2Ptr& input){

    // Create a container for the data.
    sensor_msgs::PointCloud2 output;

    // Do data processing here...

    if(rotation_set){
        filtergreen(input);
    }else{
        filterpaper(input);
    }

    // Publish the data.

    output = *input;

    if(calib_rot_count >= 20) rotation_set = true;

    if(rotation_set) {
        pubgreen.publish(output);
    }else {
        pubpaper.publish (output);
    }
}

int main (int argc, char** argv){
    // Initialize ROS
    ros::init (argc, argv, "jaco_kinect_calibration_node");
    ros::NodeHandle nh;

    cout << "\033[1;32m=====================================================\033[0m" << endl;
    cout << "\033[1;32m|\033[0m     \033[1;33mKINECT CAMERA CALIBRATION USING JACO ARM\033[0m      \033[1;32m|\033[0m" << endl;
    cout << "\033[1;32m|\033[0m \033[1;31mINTELLIGENT AND MOBILE ROBOTICS FINAL PROJECT BY:\033[0m \033[1;32m|\033[0m" << endl;
    cout << "\033[1;32m|\033[0m           \033[1;34mNuno Silva\033[0m   &   \033[1;35mMarcos Pires\033[0m           \033[1;32m|\033[0m" << endl;
    cout << "\033[1;32m=====================================================\033[0m\n" << endl;


    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pubgreen = nh.advertise<sensor_msgs::PointCloud2> ("/output/filtered_cloud/greenpoints", 1);
    pubpaper = nh.advertise<sensor_msgs::PointCloud2> ("/output/filtered_cloud/paperpoints", 1);



    // Spin
    ros::spin ();
}