#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <queue>
#include <math.h>

#define V_MAX		1.0 // m/sec
#define V_MIN		0.1 // m/sec
#define A_MAX		0.2 // m/sec^2
#define OMEGA_MAX	0.2 // rad/sec
#define ALPHA_MAX	0.2 // rad/sec^2
#define CMD_FREQ	100 // Hz

/* PathCoord
 * Values for how the robot should move
 */
class PathCoord {
public:
	const double	relative_heading;
	const double	linear_distance;

	PathCoord(double phi, double x) :
		relative_heading(phi),
		linear_distance(x) { }
};

/* RobotCommands
 *
 */
class RobotCommands {
private:
	// singleton
	static RobotCommands *instance;
	std::queue<PathCoord *> path_queue;
	ros::Publisher cmd_publisher;
	ros::Rate sleep_timer;
	// Odometry
	ros::Subscriber		odom_subscriber;
	ros::Time			odom_lastCallback;
	double				odom_deltaCallback;
	nav_msgs::Odometry	odom_latest;

	static void odomCallback(const nav_msgs::Odometry& odom_rcvd) {
		// track timing of this callback function
		RobotCommands::instance->odom_deltaCallback = (ros::Time::now() - RobotCommands::instance->odom_lastCallback).toSec();
		instance->odom_lastCallback = ros::Time::now();
		// warn about large callback times
		if (instance->odom_deltaCallback > 0.15)
			ROS_WARN("Long time since last callback: %lf", instance->odom_deltaCallback);
		
		// store the newest, desirable odometry data
		instance->odom_latest = odom_rcvd;
		
		// output data for debugging
		ROS_INFO("Current odometry: x = %f, y = %f, v = %f, omega = %f",
			odom_rcvd.pose.pose.position.x,
			odom_rcvd.pose.pose.position.y,
			odom_rcvd.twist.twist.linear.x,
			odom_rcvd.twist.twist.angular.z);
	}
		
	double calculateVelocity(const double distance_goal,
							const double distance_traveled,
							const double current_twist,
							const double vmax,
							const double amax) {
		double v,
			distance_remaining = fabs(distance_goal - distance_traveled);
		// should we stop soon?
		// time to halt = v / amax
		// distance to halt = 0.5 * amax * t^2
		// so distance to halt = 0.5 * v^2 / amax
		double distance_to_halt = 0.5 * current_twist * current_twist / amax;
		if (distance_remaining <= distance_to_halt) { // then slow down!
			// from equations above,
			v = ((distance_goal > distance_traveled)? 1.0: -1.0) * sqrt(2.0 * amax * distance_remaining);
		}
		else { // otherwise go as fast as possible
			v = vmax;
		}
		// adjust v based off of current velocity so that amax is not violated
		if (fabs(v - current_twist) > amax * CMD_FREQ) {
			v = current_twist + amax * CMD_FREQ;
		}
		return v;			
	}
	
public:

	RobotCommands(ros::NodeHandle nh):
		sleep_timer(CMD_FREQ)
	{
		instance = this;
		cmd_publisher = nh.advertise<geometry_msgs::Twist>("/robot0/cmd_vel",1);
		odom_subscriber = nh.subscribe("/robot0/odom", 1, odomCallback);
		// set angular velocity to ridiculous number so we can tell when it is overwritten with a realistic number
		odom_latest.twist.twist.angular.z = 1e10;
	} 
	
	void add(PathCoord *coord) {
		path_queue.push(coord);
	}
	
	void run() {
		geometry_msgs::Twist cmd_twist; // command to send to robot
		// Initialize cmd_twist
		cmd_twist.linear.x = 0.0;
		cmd_twist.linear.y = 0.0;
		cmd_twist.linear.z = 0.0;
		cmd_twist.angular.x = 0.0;
		cmd_twist.angular.y = 0.0;
		cmd_twist.angular.z = 0.0;
		
		// wait for the first odom callback to populate the data
		odom_lastCallback = ros::Time::now();
		ROS_INFO("Waiting for odometry data");
		while (odom_latest.twist.twist.angular.z > 400) { // 400 would be 64 rev/sec
			sleep_timer.sleep();
			ros::spinOnce();
		}
		ROS_INFO("Received initial odometry data. Proceeding.");
		
		// some variables we'll need in the loop 
		PathCoord *curr;
		nav_msgs::Odometry odom_start;
		double v, dx, dy,
			   roll, pitch, yaw,
			   distance_traveled,
			   distance_remaining;
		while (ros::ok() &&
			   !path_queue.empty()){
			// fetch target values
			curr = path_queue.front();
			// save the starting position
			odom_start = odom_latest;
			// first, rotate to the desired heading
			distance_traveled = 0.0;
			distance_remaining = fabs(curr->relative_heading);
			cmd_twist.linear.x = 0.0; // don't move linearly
			while (ros::ok() &&
				   distance_remaining > 0.0) {
				ros::spinOnce(); // allow callbacks to populate fresh data
				v = calculateVelocity(curr->relative_heading,
									  distance_traveled,
									  odom_latest.twist.twist.angular.z,
									  OMEGA_MAX, ALPHA_MAX);
				if (curr->relative_heading < 0.0)
					v = -v;
				ROS_INFO("Cmd omega: %f", v);
				cmd_twist.angular.z = v;
				// send command
				cmd_publisher.publish(cmd_twist);
				// update the distance traveled
				distance_traveled = acos(2.0*pow(odom_latest.pose.pose.orientation.z * odom_start.pose.pose.orientation.z + odom_latest.pose.pose.orientation.w * odom_start.pose.pose.orientation.w, 2)-1.0);
				distance_remaining = fabs(curr->relative_heading) - fabs(distance_traveled);
				ROS_INFO("Relative heading = %f, Yaw = %f, Distance remaining = %f", curr->relative_heading, distance_traveled, distance_remaining);
				sleep_timer.sleep(); // sleep
			}
			// stop rotating			
			cmd_twist.angular.z = 0.0; // don't rotate
			cmd_publisher.publish(cmd_twist);
			// now, go the desired distance
			distance_traveled = 0.0;
			distance_remaining = curr->linear_distance;
			while (ros::ok() &&
				   distance_remaining > 0.0) {
				ros::spinOnce(); // allow callbacks to populate fresh data
				v = calculateVelocity(curr->linear_distance,
									  distance_traveled,
									  odom_latest.twist.twist.linear.x,
									  V_MAX, A_MAX);
				if (v < 0.0) v = 0.0; // let's not go backwards
				ROS_INFO("Cmd velocity: %f; distance remaining: %f", v, distance_remaining);
				cmd_twist.linear.x = v;
				// send command
				cmd_publisher.publish(cmd_twist);
				// update the distance traveled
				dx = odom_latest.pose.pose.position.x - odom_start.pose.pose.position.x;
				dy = odom_latest.pose.pose.position.y - odom_start.pose.pose.position.y;
				distance_traveled = sqrt(dx*dx + dy*dy);
				distance_remaining = curr->linear_distance - distance_traveled;
				sleep_timer.sleep(); // sleep
			}
			cmd_twist.linear.x = 0.0; // stop moving
			cmd_publisher.publish(cmd_twist);
			// We've reached our destination, so get rid of the PathCoord
			path_queue.pop();
		}
	}
};

RobotCommands *RobotCommands::instance;

int main(int argc, char **argv) {
	ros::init(argc,argv,"robot_commander_dtb37"); // name of this node 
	ros::NodeHandle nh; // two lines to create a publisher object that can talk to ROS
	//stdr "robot0" is expecting to receive commands on topic: /robot0/cmd_vel

	PathCoord pc1(0.0, 4.8),
			  pc2(-M_PI/2.0 + 0.05, 12.2),
			  pc3(-M_PI/2.0 + 0.05, 8.5);
			  
	RobotCommands rc(nh);
	rc.add(&pc1);
	rc.add(&pc2);
	rc.add(&pc3);
	rc.run();

	ROS_INFO("Destination reached!");

	return 0;
} 