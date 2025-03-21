#define _USE_MATH_DEFINES

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include <cmath>
#include <tf/transform_broadcaster.h>

#include "first_project/Odom.h"
#include "first_project/ResetOdom.h"

class odom_node{
	
	ros::NodeHandle n;

	ros::Subscriber sub;
	ros::Publisher pub_odom;
	ros::Publisher pub_odom_custom;

	bool firstCall = true; // see usage in inputCallback()
	double dt;
	ros::Time now;

	nav_msgs::Odometry odomOld;
	nav_msgs::Odometry odomNew;
	first_project::Odom odomCustom;

	ros::ServiceServer service;

	tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Quaternion q;

	const double INIT_X = 0.0; // these might need to be changed when parameters are used instead
	const double INIT_Y = 0.0;
	const double INIT_TH = 0.0;

	const float FR_WHEEL_DIST = 2.8; //front to rear wheels distance in meters
 	// # define TF_EULER_DEFAULT_ZYX //makes it so the constructor for Quaternions takes angles in the RPY (instead of YPR) order	
	// const tf::Transform tf_front_left = tf::Transform(
	// 		tf::Quaternion(.81, 0, 3.14),
	// 		tf::Vector3(3.7, .93, 0)
	// 		);
	// const tf::Transform tf_front_right = tf::Transform(
	// 		tf::Quaternion(-.76, 0, 3.14),
	// 		tf::Vector3(3.7, -.93, 0)
	// 		);
	// const tf::Transform tf_rear_left = tf::Transform(
	// 		tf::Quaternion(2.38, 0, 3.14),
	// 		tf::Vector3(0, .93, 0)
	// 		);
	// const tf::Transform tf_rear_right = tf::Transform(
	// 		tf::Quaternion(-2.3, 0, 3.14),
	// 		tf::Vector3(.1, -.8, 0)
	// 		);

public:
	odom_node(){
		sub = n.subscribe("/speed_steer", 1, &odom_node::inputCallback, this);
		pub_odom = n.advertise<nav_msgs::Odometry>("/odometry", 1);
		pub_odom_custom = n.advertise<first_project::Odom>("/custom_odometry", 1);
		service = n.advertiseService("reset_odom", &odom_node::reset, this);

		now = ros::Time::now();

		if (!n.getParam("/starting_x", odomOld.pose.pose.position.x)){
			odomOld.pose.pose.position.x = INIT_X;
			ROS_INFO("x position set to INIT_X");
		} else {
			ROS_INFO("x position set to starting_x");
		}
		if (!n.getParam("/starting_y", odomOld.pose.pose.position.y)){
			odomOld.pose.pose.position.y = INIT_Y;
			ROS_INFO("y position set to INIT_Y");
		} else {
			ROS_INFO("y position set to starting_y");
		}
		if (!n.getParam("/starting_th", odomOld.pose.pose.orientation.z)){
			odomOld.pose.pose.orientation.z = INIT_TH;
			ROS_INFO("th angle set to INIT_TH");
		} else {
			ROS_INFO("th angle set to starting_th");
		}
		odomOld.header.stamp = now;
		odomOld.header.frame_id = "odom";
		odomOld.child_frame_id = "base_link";

		odomNew.pose.pose.position.x = 0.0;
		odomNew.pose.pose.position.y = 0.0;
		odomNew.pose.pose.orientation.z = 0.0;
		odomNew.twist.twist.linear.x = 0.0;
		odomNew.twist.twist.linear.y = 0.0;
		odomNew.twist.twist.angular.z = 0.0;
		odomNew.header.stamp = now;
		odomNew.header.frame_id = "odom";
		odomNew.child_frame_id = "base_link";
	}

	void inputCallback(const geometry_msgs::Quaternion::ConstPtr& input){
		ROS_DEBUG("inputCallback() called");
		now = ros::Time::now();
		// first calulation is omitted (to be more precise, the timestamps of old and new odometry are set to almost the same value, resulting in a dt of pretty much zero) since no time has passed yet between datapoints
		if (firstCall){
			firstCall = false;
			odomOld.header.stamp = now;
		}
		calcOdom(input);
		genCustomMsg();
		pub_odom.publish(odomNew);
		pub_odom_custom.publish(odomCustom);
		genTf();
	}

	void calcOdom(const geometry_msgs::Quaternion::ConstPtr& input){
		odomNew.header.stamp = now;
		dt = (odomNew.header.stamp - odomOld.header.stamp).toSec();
		// assumption: speed in bag is the rear wheel speed, addendum: speed of front and back wheel should be the same
		ROS_DEBUG("v = %f", (float) input->x);
		ROS_DEBUG("alpha = %f = %f deg", (float) input->y, 180 / M_PI * (float) input->y);
		// ROS_DEBUG("x_old = %f", (float) odomOld.pose.pose.position.x);
	       	// ROS_DEBUG("th_old = %f = %f deg", (float) odomNew.pose.pose.orientation.z, 180 / M_PI * (float) odomNew.pose.pose.orientation.z);
		ROS_DEBUG("dt = %f", (float) dt);
		// odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + cos(odomOld.pose.pose.orientation.z) * input->x * dt;
		// odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + sin(odomOld.pose.pose.orientation.z) * input->x * dt;
		// odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z + input->x * tan(input->y) * dt / FR_WHEEL_DIST;
		// odomNew.twist.twist.linear.x = cos(odomOld.pose.pose.orientation.z) * input->x;
		// odomNew.twist.twist.linear.y = sin(odomOld.pose.pose.orientation.z) * input->x;
		odomNew.twist.twist.angular.z = input->x * tan(input->y) / FR_WHEEL_DIST;
		if (odomOld.twist.twist.angular.z == 0){
			// RUNGE_KUTTA INTEGRATION
			ROS_DEBUG("Using Runge-Kutta integration");
			odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + input->x * dt * cos(odomOld.pose.pose.orientation.z + odomOld.twist.twist.angular.z * dt / 2);
			odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + input->x * dt * sin(odomOld.pose.pose.orientation.z + odomOld.twist.twist.angular.z * dt / 2);
        		odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z; //since angular velocity is 0, the orientation will not change
		} else {
			// EXACT INTEGRATION
			ROS_DEBUG("Using exact integration");
			odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z + odomOld.twist.twist.angular.z * dt;
			odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + input->x / odomOld.twist.twist.angular.z * (sin(odomNew.pose.pose.orientation.z) - sin(odomOld.pose.pose.orientation.z));
			odomNew.pose.pose.position.y = odomOld.pose.pose.position.y - input->x / odomOld.twist.twist.angular.z * (cos(odomNew.pose.pose.orientation.z) - cos(odomOld.pose.pose.orientation.z));
		}

		if (odomNew.pose.pose.orientation.z > M_PI){
			odomNew.pose.pose.orientation.z -= 2 * M_PI;
		} else if (odomNew.pose.pose.orientation.z < -M_PI) {
			odomNew.pose.pose.orientation.z += 2 * M_PI;
		}

		odomOld = odomNew;
	}

	void genCustomMsg(){
		odomCustom.x = odomNew.pose.pose.position.x;
		odomCustom.y = odomNew.pose.pose.position.y;
		odomCustom.th = odomNew.pose.pose.orientation.z;
		odomCustom.timestamp = std::to_string(odomNew.header.stamp.toSec());
	}

	void genTf(){
		transform.setOrigin(tf::Vector3(
					odomNew.pose.pose.position.x,
					odomNew.pose.pose.position.y,
					0
					));
		q.setRPY(0, 0, odomNew.pose.pose.orientation.z);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, now, "odom", "base_link"));

		// br.sendTransform(tf::StampedTransform(tf_front_left, now, "base_link", "sick_front_left"));
		// br.sendTransform(tf::StampedTransform(tf_front_right, now, "base_link", "sick_front_right"));
		// br.sendTransform(tf::StampedTransform(tf_rear_left, now, "base_link", "sick_rear_left"));
		// br.sendTransform(tf::StampedTransform(tf_rear_right, now, "base_link", "sick_rear_right"));
	}

	bool reset(first_project::ResetOdom::Request &req, first_project::ResetOdom::Response &res){
		ROS_DEBUG("reset callback function called");
		odomOld.pose.pose.position.x = 0.0;
		odomOld.pose.pose.position.y = 0.0;
		odomOld.pose.pose.orientation.z = 0.0;
		odomOld.twist.twist.linear.x = 0.0;
		odomOld.twist.twist.linear.y = 0.0;
		odomOld.twist.twist.angular.z = 0.0;

		res.resetted = true;

		return true;
	}
};

int main (int argc, char **argv){
	
	ros::init(argc, argv, "odom_node");
	odom_node odom_node_instance;
	ros::spin();
	return 0;

}
