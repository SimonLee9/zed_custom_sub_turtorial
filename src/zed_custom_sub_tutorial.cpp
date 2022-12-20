///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2018, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/**
 * 
 * This tutorial demonstrates simple receipt of ZED depth messages over the ROS system.
 */


// Add, Auto moving
#include <string>
#include <iostream>
#include <cmath>
#include <algorithm>

#include <ros/ros.h>

#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>

#include <zed_interfaces/Object.h>
#include <zed_interfaces/ObjectsStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>



// Add, from GPS & UTM
#include <math.h>
#include <vector>
#include "ros/ros.h"
#include "tf/transform_datatypes.h"//for tf, getYaw
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include <GeographicLib/TransverseMercator.hpp>

#include <nav_msgs/Path.h>
#include "geometry_msgs/Point.h"



#include <ros/package.h>
#include <fstream>
#include <utility>
#include <vector>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>

#include "sensor_msgs/LaserScan.h"

#include <std_msgs/UInt16MultiArray.h>
/**
 * Subscriber callback
 */

using namespace std;

int cmd_vel; // inital speed of vehicle 

int cmd_vel_def;
int cmd_vel_add;

int cmd_vel_imu;
int cmd_vel_dis;
int cmd_vel_obj;
int cmd_vel_key;

int buz_alarm;

int temp_vel;
char A;
char B;

double GTPX = 1;
double GTPY = 1;

double GCPX = 1;
double GCPY = 1;

//double distance;


struct basic{
  double bx = 0.0;//-594929.9431329881;//east
  double by = 0.0;//-4139043.529676078;//north
  double bz = 0.0;//unit in m
};//origin point

//std::string path = ros::package::getPath("zed_custom_sub_tutorial");

void objectListCallback(const zed_interfaces::ObjectsStamped::ConstPtr& msg);

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
   /*
    ROS_INFO("Accel: %.3f,%.3f,%.3f [m/s^2] - Ang. vel: %.3f,%.3f,%.3f [deg/sec] - Orient. Quat: %.3f,%.3f,%.3f,%.3f",
           msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z, msg->angular_velocity.x,
           msg->angular_velocity.y, msg->angular_velocity.z, msg->orientation.x, msg->orientation.y, msg->orientation.z,
           msg->orientation.w);
  */
  
    if (msg->linear_acceleration.y>0.25 || msg->angular_velocity.x>0.25 || msg->angular_velocity.y> 0.25) // || : or
    //if (msg->linear_acceleration.y>0.25 || msg->angular_velocity.y> 0.25) // || : or
    {
        cmd_vel_imu=-30;
    }
    else
    {
	cmd_vel_imu=0;
    }
   
}


void utmCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  //ROS_INFO_STREAM("PrintTest");
  
  double px = msg->pose.position.x ;
  double py = msg->pose.position.y ;
  double pz = msg->pose.position.z ;

  GCPX = px ;//Get Current Position X
  GCPY = py; //Get Current Position Y

  //ROS_INFO("Position data x: [%f]", px);
  //ROS_INFO("Position data y: [%f]", py);
  //ROS_INFO("Position data z: [%f]", pz);
}



void targetpointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{


  double wx = msg->pose.position.x;
  double wy = msg->pose.position.y;
  double wz = msg->pose.position.z;

  GTPX = wx ; //Get Target Position X
  GTPY = wy ; //Get Target Position Y

  //ROS_INFO("Target Position data x: [%f]", GTPX);
  //ROS_INFO("Target Position data y: [%f]", GTPY);
  //ROS_INFO("Target Position data z: [%f]", wz);
}


void distancecb()
{


  targetpointCallback;
  utmCallback;


ROS_INFO("Target Position data x: [%f]", GTPX);
ROS_INFO("Target Position data y: [%f]", GTPY);

ROS_INFO("current Position data x: [%f]", GCPX);
ROS_INFO("current Position data y: [%f]", GCPY);


  //ROS_INFO("Target Position data z: [%f]", wz);
  //int x, y;
  //double DistanceX = GTPX - GCPX;
  //ROS_INFO("******************DisttanceXXXXXXXXXXXXXXX*************: [%f]m", DistanceX);

  double distance = sqrt(pow(GTPX - GCPX,2) + pow(GTPY - GCPY,2));
  ROS_INFO("******************Disttance*******************: [%f]m", distance);

    int goal_reached;
		
  
	if (distance < 2)
		{
			goal_reached = 1;
		}
	else 
		{
			goal_reached = 0;
		}

	if (!goal_reached)
		{
			cmd_vel_dis=8;
		}

	else
		{
			cmd_vel_dis=-50;
		}
   //ROS_INFO("cmd_vel: [%d]", cmd_vel);
}



void scanCallback(const sensor_msgs::LaserScan& msg)
{


}

void yolodarknetCallback(const std_msgs::UInt16& msg)
{


}


void controlkeyCallback(const std_msgs::UInt16MultiArray& msg)
{
  int straight = msg.data[0];
  int heading = msg.data[1];
  //ROS_INFO("***** vel_key *****=%d",straight);

  cmd_vel_key = msg.data[0]-90;

}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "total_cmd_subscriber");

  ros::NodeHandle n;

  ros::Subscriber subImu = n.subscribe("/zed2/zed_node/imu/data", 10, imuCallback);

  ros::Subscriber subObjList = n.subscribe("/zed2/zed_node/obj_det/objects", 1, objectListCallback);
    
  ros::Subscriber utm_sub = n.subscribe("/utm",1,utmCallback);

  ros::Subscriber targetpoint_sub = n.subscribe("/targetposition",1,targetpointCallback);

  ros::Subscriber scan_sub = n.subscribe("/scan",1,scanCallback);

  ros::Subscriber yolo_sub = n.subscribe("/yolo_darknet",1,yolodarknetCallback);

  ros::Subscriber controlkey_sub = n.subscribe("/control_key",10,controlkeyCallback);

  ros::Publisher chatter_pub = n.advertise<std_msgs::UInt16>("/control",1000); // ZED_detection : is topic
  //ros::Publisher chatter_pub = n.advertise<std_msgs::UInt16MultiArray>("/control",1000); // ZED_detection : is topic

  ros::Publisher buz_pub = n.advertise<std_msgs::UInt16>("/alarm",1000); // ZED_detection : is topic

  std_msgs::UInt16 msg;

  std_msgs::UInt16 buz_msg;
  //std_msgs::UInt16MultiArray msg;
  ros::Rate loop_rate(10);

  
  //targetpointCallback();
  //utmCallback();
  cmd_vel_def =90;
  
  //buz_alarm = 0;
			
		
  while (ros::ok()){

      msg.data = cmd_vel;
      cmd_vel = cmd_vel_def + cmd_vel_imu + cmd_vel_dis + cmd_vel_obj + cmd_vel_key;
      
      //cmd_vel =  msg.data[0] ;
      buz_msg.data = buz_alarm;


      ROS_INFO("***** Velocity-PWM *****=%d",cmd_vel);
      ROS_INFO("***** vel_imu *****=%d",cmd_vel_imu);
      ROS_INFO("***** vel_dis *****=%d",cmd_vel_dis);
      ROS_INFO("***** vel_obj *****=%d",cmd_vel_obj);
      ROS_INFO("***** vel_key *****=%d",cmd_vel_key);
      chatter_pub.publish(msg);
        distancecb();
        //objectListCallback;
        imuCallback;

      buz_pub.publish(buz_msg);

    ros::spinOnce(); 
    
    loop_rate.sleep();

    
    
  }



  return 0;
}

void objectListCallback(const zed_interfaces::ObjectsStamped::ConstPtr& msg)
{
  cmd_vel_obj=8;	
  buz_alarm = 0;

  for (int i = 0; i < msg->objects.size(); i++)
  { 

    if (msg->objects[i].label_id == -1) // ÀÎ½Ä ´ë»óÃ¼ ¾øÀ½

      continue; // ·çÇÁ ¸öÃ¼ ³¡À¸·Î Á¡ÇÁÇÑ´Ù.
      cmd_vel_obj=2; 
      
      /*
      ROS_INFO_STREAM(msg->objects[i].label << " [" << msg->objects[i].label_id << "] - Pos. ["
                                          << msg->objects[i].position[0] << "," << msg->objects[i].position[1] << ","
                                          << msg->objects[i].position[2] << "] [m]"
                                          << "- Conf. " << msg->objects[i].confidence
                                          << " - Tracking state: " << static_cast<int>(msg->objects[i].tracking_state));
      */

    

    if (msg->objects[i].position[0] < 1){


      cmd_vel_obj=-25;
      buz_alarm = 392;
     ROS_INFO("*******Warnning :: Person | Vehicle"); 
    }
		
  
  } 
    

  //ROS_INFO("***** Velocity-PWM *****=%d",cmd_vel);
  
}
