/*
 * Copyright (c) 2012, Debashish Chakraborty, Arash Mamoudzadeh.
 * All rights reserved.
 */
 
 //used header files
#include <turtlebot_node/TurtlebotSensorState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/rate.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include <vector>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/CvBridge.h>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <mainproj/matcher.h>


//-----------PUBLIC PARAMETERS--------------------------------
	using namespace std; //standard library
	using namespace cv;	//opencv library
	namespace enc = sensor_msgs::image_encodings;	//ROS color encoding for images 
	//double dist, angle, obsdist, obsang;
	volatile int bumperState; //bumper sensor of turtlebot
	float data[100000];			//image matrix of depth values
	float range_max, range_min;		//laser scan range limit
	float maxdepth_vals[1000];		//matrix of maximum depth values
	double crsp_angles[1000];		//angles corresponding to maximum depth values
	//necessary control parameters used through out the code.
	int control;	
	int control2;
	int control4;
	int control5;
	int control6;
	//
	int maxcounter; 	//counter for maximum values
	float avgmax;		// average of all the maximums in 360 turn
	sensor_msgs::CvBridge bridge_;		//used to convert ROS to Opencv images
	cv::Mat cv_image(480,640,CV_8UC3);	// images matrix from kinect camera
	float obsang, obsdist, dist, angle;		//distance and angle measurements of the robot's movements
	

//------------------------------------------------------------
class TurtlebotTeleop
{
public:
	int detectGo (cv::Mat, cv::Mat);	//function to do object detection
	void move(double);					//object that directs the robot to move out of the maze
  void publish(double, double);			//function to publish speed commands to turtlebot
	float maxrange();					//calculates the maximum value among given data
	float minrange();					// calculates the minimum value among given data
	float avgrange();					//calculates the average values among given data
  TurtlebotTeleop();					//object of TurtlebotTeleop class
	void keyLoop();						// the main fuction where all the pieces of the project are combined 
  double linear_, angular_;				//linear and angular speeds from ROS sensor state
	void imageCb(const sensor_msgs::ImageConstPtr&);	//call back function to capture and convert kinect image to opencv image
	void callback01(const turtlebot_node::TurtlebotSensorState::ConstPtr&);		//call back where the bumper sensor and distances are measured
	void callback02(const sensor_msgs::LaserScan::ConstPtr&);					//call back where data is used to capture the depth values from laser scan
private:
  ros::NodeHandle nh_,ph_;
  ros::Time first_publish_;
  ros::Time last_publish_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  boost::mutex publish_mutex_;
};
//------------------------------------------------------------
//a constructor for TurtlebotTeleop class executed only when a new object of TurtlebotTeleop is created
TurtlebotTeleop::TurtlebotTeleop():
  ph_("~"),
  linear_(0),
  angular_(0),
  l_scale_(1.0),
  a_scale_(1.0)
{
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}
//--------------------------------------------------------------
// shuts down ros and exists
int kfd = 0;
struct termios cooked, raw;
void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}
//----------------------------------------------------------------
//call backs
void TurtlebotTeleop::imageCb(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;	
	cv_ptr = cv_bridge::toCvCopy(msg, enc::RGB8);		//convert ROS to opencv images
	cv_image = cv_ptr -> image;
	if (!cv_image.data) {
	 puts("error");
	}	
}

void TurtlebotTeleop::callback01(const turtlebot_node::TurtlebotSensorState::ConstPtr& msg){
			   bumperState = msg->bumps_wheeldrops;		//set status of bumper sensor from the robot
			//	cliff_fl = msg->cliff_front_left;
			//	cliff_fr = msg->cliff_front_right;
			//calculate distances published by the sensor state from robot
				dist += msg->distance;					
				angle +=msg->angle;
				obsdist += msg->distance;
				obsang +=msg->angle;
			//
}

void TurtlebotTeleop::callback02(const sensor_msgs::LaserScan::ConstPtr& msg){
			int i;
			for (i = 0; i<11; i++){
			  data[i] = msg->ranges[i+175];		//choosing depth values from 85 to 95 degrees for further consideration
			}
				range_min = msg->range_min;		//limitation of laser scan 
				range_max = msg->range_max;		//limitation of laser scan
}

//----------------------------------------------------------
int main(int argc, char** argv)
{
control6 = 0;
control5 = 0;
control4 = 0;
control = 0;
control2 = 0;
//dist = 0;
//angle= 0;
avgmax =0;
	ros::init(argc, argv, "turtlebot_teleop");
	TurtlebotTeleop turtlebot_teleop;
	ros::NodeHandle n; 		//creating node from ROS for subscriptions
	signal(SIGINT,quit);
//attaching keyloop function to our class object so it runs in parallel with others like call backs
	boost::thread my_thread(boost::bind(&TurtlebotTeleop::keyLoop, &turtlebot_teleop)); 		
//callback01 - sensor_state - subscribing to "turtlebot_node/sensor_state" topic to access the buplish speed and bumper sensor valus
	ros::Subscriber sens_sub_ = n.subscribe("turtlebot_node/sensor_state", 10000, &TurtlebotTeleop::callback01, &turtlebot_teleop);
//callback02 - scan - subscribing to "scan" topic published by kinect.launch to access the depth values
	ros::Subscriber depth_sub_ = n.subscribe("scan",1,&TurtlebotTeleop::callback02, &turtlebot_teleop);
//callback03 - image_mono	-	subscribing to "camera/rgb/image_color" topic to access RGB images colors from ROS
	ros::Subscriber img_sub_ = n.subscribe("camera/rgb/image_color", 1, &TurtlebotTeleop::imageCb, &turtlebot_teleop);
	ros::spin();
	my_thread.interrupt() ;
	my_thread.join() ;
	return(0);
}

//----------------------------------------------------------------
void TurtlebotTeleop::keyLoop()
{
	//this is where robot is commanded to move outside of the maze
	// avgmax and control6 make sure that robot stops when it reaches outside of the gate
	while (avgmax<2.9 &&control6 == 0){ 
	move(1);
	//this part makes sure if robot hits a wall, it moves back and continue its movement
	if (bumperState !=0){
	control = 1;
	obsdist = 0;
	}
	while(control == 1){
		if(obsdist >-0.15){
		publish(0,-0.1);
		}else
		control = 0;
	}
	}
	//
	//
	//hardcoding the robot to do a square walk and reach the intended area where the objects are located
publish(0,0);
	ros::Rate lr(2);
int j=0;
	while(j<1 && bumperState == 0){
	publish(0,0.5);
	j++;
	lr.sleep();
	}
float olddist = dist;
int qq=0;
	while(qq<4){
      publish(-1,0);
      qq++;
      lr.sleep();
	}
int dp=0;
	while(dp<7){
      publish(0,1);
      dp++;
      lr.sleep();
	}
int tt=0;
	while(tt<3){
      publish(-1,0);
      tt++;
      lr.sleep();
	}
dist = 0;
int uu=0;
	while(dist<(olddist+0.4)){
      publish(0,1);
      uu++;
      lr.sleep();
	}
//

/*
//another method of reaching the intended area by moving in an arc shape
		int i=0;
	angle = 0;
	while(i<16 && bumperState == 0){
			//printf("angle= %f\n", angle);
	publish(-0.72,0.76);
	i++;
	lr.sleep();
	}
*/

//------------------------------
	std::vector<int> dg;
	cv::Mat rgb_2 = imread( "/home/turtlebot/mainproj/templ.jpg");	//reading the template of weetbix box
	dist = 0;
	ros::Rate rate(2);
	
/*
	//a different method of reaching our intended area by first 
	//locating the closest point from the object and then hardcoding the angle of movement.
	publish(0,0);
	int ww=0;
	std::vector<float> check;
	while (ww<12){
		publish(-1,0);
		float avgeach=0;
		int nn=0;
		while (nn<11){
			avgeach = avgeach+data[nn];
			nn++;
		}
		printf("avgeach= %f\n", avgeach);
		check.push_back(avgeach);
		ww++;
		rate.sleep();
	}
	
	float mincheck = 100;
	int checkidx = 0;
	int mm=0;
	while(mm<12){
		if(check[mm]< mincheck){
				mincheck = check[mm];
				checkidx = mm;
		}
		mm++;
	}
	int jj=0;
	while (jj < checkidx){
		publish(-1,0);
		jj++;
		rate.sleep();
	}
	
	int kk=0;
	while (kk < 1){
		publish(1,0);
		kk++;
		rate.sleep();
	}
	
	int bo=0;
	while (bo < (int)(mincheck+1)){
		publish(0,1);
		bo++;
		rate.sleep();
	}
*/


	//here the robot turns 360 degrees and computes the amount of match
	//between the weetbix box and kinect images every 10 degrees  and then
	//finds the angle corresponding to maximum matches and then turns to it and moves to hit the weetbix box 
	publish(0,0);
	int b=0;
	while (b<33){
		cv::Mat rgb_1 = cv_image;
		dg.push_back(detectGo (rgb_1,rgb_2)); //use of detecGo function for matching calculations
		printf("dg= %d\n", dg[b]);
		publish(-0.5,0);
		b++;
		rate.sleep();
     }
     publish(0,0);
     int maxmatch=0;
     int maxmatchidx=0;
     int c=0;
     while(c<33){
		if(dg[c]>= maxmatch) {
				maxmatch = dg[c];
				maxmatchidx = c;
		}
		c++;
     }
     printf("WEETBIX BOX MATCH DETECTED AT= %d\n", (360*maxmatchidx/33));
     int d=0;
     while(d < (maxmatchidx)){
        publish(-0.5,0);
        d++;
        rate.sleep();
     }
     
 	while(bumperState==0){
 	publish(0,1);
 	}
 	publish(0,0);
 	//........................................................
	//here the robot  moves backward then turns 360 degrees and computes the amount of match
	//between the tank and kinect images every 10 degrees  and then
	//finds the angle corresponding to maximum matches and then turns to it and moves to hit the tank 
 	std::vector<int> dg2;
	cv::Mat rgb_3 = imread( "/home/turtlebot/mainproj/templ2.jpg");	//reading the template for tank
	int aa=0;
	ros::Rate lrate(2);
	while(aa<5){
		publish(0,-1);
		aa++;
		rate.sleep();
	}
	int bb=0;
	while (bb<33){
		cv::Mat rgb_1 = cv_image;
		dg2.push_back(detectGo (rgb_1,rgb_3));	//use of detectGo to calculate matches
		printf("dg2= %d\n", dg2[bb]);
		publish(-0.5,0);
		bb++;
		rate.sleep();
  }
    publish(0,0);
    int maxmatch2=0;
    int maxmatchidx2=0;
    int cc=0;
  while(cc<33){
		if(dg2[cc]>= maxmatch2) {
				maxmatch2 = dg2[cc];
				maxmatchidx2 = cc;
		}
		cc++;
  }
     printf("FUEL TANK MATCH DETECTED AT= %d\n", (360*maxmatchidx2/33));
     int dd=0;
  while(dd<(maxmatchidx2-6)){
        publish(-0.5,0);
        dd++;
        rate.sleep();
  }
     
 	while(bumperState==0){
 		publish(0,1);
 	}
 	publish(0,0);
  return;
}

//-------------------------------------------------------------
void TurtlebotTeleop::publish(double angular, double linear)  
{
    geometry_msgs::Twist vel;
    vel.angular.z = angular;//a_scale_*angular;
    vel.linear.x = linear;//l_scale_*linear;
    vel_pub_.publish(vel);    
  return;
}
//---------------------------------------------------------------
float TurtlebotTeleop::maxrange(){
float max = 0;
int i;
for (i = 0; i<11; i++){
		if(data[i]>max && data[i]<=10) {
				max = data[i];
		}
}
return max;
}
//---------------------------------------------------------------
float TurtlebotTeleop::avgrange(){
float avg = 0;
int i;
for (i = 0; i<11; i++){
		if(data[i]<=10) {
		avg = avg + data[i];
		}
}
avg = avg/11;
return avg;
}
//----------------------------------------------------------------
float TurtlebotTeleop::minrange(){
float min = 11;
int i;
for (i = 0; i<11; i++){
		if(data[i]<min && data[i]>0) {
		min = data[i];
		}
}
return min;
}
//----------------------------------------------------------------
void TurtlebotTeleop::move(double scale)
{
float mini=0;
maxcounter = 0;
int control3 = 0;
float max =0;
int mxangidx =0;
float preavgmax =0;
ros::Rate loop_rate(2);
int i=0;
//........................
//turning 360 and scanning and getting the maximum depth at each 30 deg turn
while(i<12 && bumperState == 0){
publish (1,0);
max = maxrange();
mini = minrange();
printf("minrange= %f\n", mini);
maxdepth_vals[i] = max;
i++;
loop_rate.sleep();
}
//........................
//calculating the overall maximum depth in 360 turn with it corresponding angle
float maxdepth = 0;
int t=0;
while(t<12){
preavgmax = (preavgmax + maxdepth_vals[t]) ;
printf("maxdepth= %f\n", maxdepth_vals[t]);
    if (maxdepth_vals[t]>3){
        maxcounter++;
    }
		if(maxdepth_vals[t]> maxdepth){
				maxdepth = maxdepth_vals[t];
				mxangidx = t;
		}
t++;
}
//.......................

printf("maxcounter= %d\n", maxcounter);
avgmax = preavgmax/12;
printf("maxdepth= %f\n", maxdepth);
printf("mxangidx= %d\n", mxangidx);
printf("avgmax= %f\n", avgmax);

//............................
//comparing the depth of the angle before maxdepth and 
//after maxdepth to adjust the angle to be away from the edges
if(maxdepth_vals[mxangidx+2]< maxdepth_vals[mxangidx]){
mxangidx = mxangidx-1;
}else {mxangidx = mxangidx+1;}
//.............................
//turning to the corresponding angle
int j=0;
while (j <= mxangidx && bumperState == 0){
publish(1,0);
j++;
loop_rate.sleep();
}
//.............................
//this is to adjust the motion to upon finding the location of exit
//so that the robot first ends up close to the door and right in front of it
int o=0;
if(maxdepth>5 && control2==0 && avgmax<=2.9 && bumperState==0){
    puts("inside");
	while(o<1){
		publish(-0.8,0);
		o++;
		loop_rate.sleep();
	}
	publish(0,0);
	float mxim = maxrange();
	printf("mxim= %f\n", mxim);
	dist=0;
	while(bumperState==0 && dist<(mxim-0.84)){
	printf("dist= %f\n", dist);
				publish(0,1);
				loop_rate.sleep();
	}
	control2 = 1;
	control3 = 1;
	control4 = 1;
	control5 = 1;
}
//.................................
//moving forward accoriding to the maximum distance found
int v=0;
while(v<((int)maxdepth*scale) && bumperState == 0 && avgmax<=2.9 && control3==0){
puts("yyyy");
if(control5==1){
puts("xxxx");
publish (0.6,0);
control6 = 1;
control5=0;
dist = 0;
}else{
publish(0,1);
}
v++;
loop_rate.sleep();
}
//................................
publish(0,0);
return;
}
//----------------------------------------------------------------------------------------------------------------
int TurtlebotTeleop::detectGo (cv::Mat rgb_test, cv::Mat rgb_templ){

	//uses RobustMatcher class from matcher.h header file to accurately apply template matching for a given image an template
	RobustMatcher detector;
	std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
	std::vector<DMatch> matches;
	cv::Mat fundemental =  detector.match(rgb_test,rgb_templ, matches,keypoints_1, keypoints_2);
	
/*  //for displaying purposes
	cv::Mat img_matches;

	drawMatches(rgb_test, keypoints_1,rgb_templ,keypoints_2,matches,img_matches,Scalar::all(-1),Scalar::all(-1),vector<char>(),DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	namedWindow("image",CV_WINDOW_AUTOSIZE);
	cvStartWindowThread();
	imshow("image",img_matches);
	*/
	int counting =matches.size(); 	
return counting;	//returns the number of match points for each given image
}

