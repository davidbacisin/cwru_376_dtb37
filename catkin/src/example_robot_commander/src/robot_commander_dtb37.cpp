#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <math.h>

#define V_MAX		5.0 // m/sec
#define V_MIN		0.1 // m/sec
#define A_MAX		0.1 // m/sec^2
#define OMEGA_MAX	1.0 // rad/sec
#define ALPHA_MAX	0.5 // rad/sec^2
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
	RobotCommands() {} // cannot create instances
	
	static std::queue<PathCoord *> path_queue;
	static ros::Publisher cmd_publisher;
	static ros::Rate sleep_timer(CMD_FREQ);
	// Odometry
	static ros::Subscriber		odom_subscriber;
	static ros::Time			odom_lastCallback;
	static double				odom_deltaCallback;
	static nav_msgs::Odometry	odom_latest;
	
	static void odomCallback(const nav_msgs::Odometry& odom_rcvd) {
		// track timing of this callback function
		odom_deltaCallback = (ros::Time::now() - odom_lastCallback).toSec();
		odom_lastCallback = ros::Time::now();
		// warn about large callback times
		if (odom_deltaCallback > 0.15)
			ROS_WARN("Long time since last callback: %lf", odom_deltaCallback);
		
		// store the newest, desirable odometry data
		odom_latest.twist.twist.linear.x = odom_rcvd.twist.twist.linear.x;
		odom_latest.twist.twist.angular.z = odom_rcvd.twist.twist.angular.z;
		odom_latest.pose.pose.position.x = odom_rcvd.pose.pose.position.x;
		odom_latest.pose.pose.position.y = odom_rcvd.pose.pose.position.y;
		odom_latest.pose.pose.orientation.z = odom_rcvd.pose.pose.orientation.z;
		odom_latest.pose.pose.orientation.w = odom_rcvd.pose.pose.orientation.w;
		
		// output data for debugging
		ROS_INFO("Current odometry: x = %f, y = %f, phi = %f, v = %f, omega = %f",
			odom_latest.pose.pose.position.x,
			odom_latest.pose.pose.position.y,
			0.0,
			odom_latest.twist.twist.linear.x,
			odom_latest.twist.twist.angular.z);
	}
	
	static double calculateVelocity(const double distance_goal,
									const double distance_traveled,
									const double current_twist,
									const double vmax,
									const double amax) {
		double v,
			distance_remaining = distance_goal - distance_traveled;
		// should we stop soon?
		// time to halt = v / amax
		// distance to halt = 0.5 * amax * t^2
		// so distance to halt = 0.5 * v^2 / amax
		double distance_to_halt = 0.5 * current_twist * current_twist / amax;
		if (distance_remaining <= 0.0) { // we're here, so stop
			v = 0.0;
		}
		else if (distance_remaining <= distance_to_halt) { // then slow down!
			// from equations above,
			v = sqrt(2.0 * amax * distance_remaining);
		}
		else { // otherwise go as fast as possible
			v = vmax;
		}
		// adjust v based off of current velocity so that amax is not violated
		v -= current_twist;
		if (v > amax * CMD_FREQ) {
			v = amax * CMD_FREQ;
		}
		else if (v < 0.0) { // let's not go backwards
			v = 0.0;
		}
		return v;			
	}
	
public:
	static void init(ros::NodeHandle nh) {
		cmd_publisher = nh.advertise<geometry_msgs::Twist>("/robot0/cmd_vel",1);
		odom_subscriber = nh.subscribe("/robot0/odom", 1, odomCallback);
		// set angular velocity to ridiculous number so we can tell when it is overwritten with a realistic number
		odom_latest.twist.twist.angular.z = 1e10;
	}
	
	static void add(PathCoord *coord) {
		path_queue.push(coord);
	}
	
	static void run() {
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
		double dx, dy,
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
			distance_remaining = curr->relative_heading;
			cmd_twist.linear.x = 0.0; // don't move linearly
			while (ros::ok() &&
				   distance_remaining > 0.0) {
				ros::spinOnce(); // allow callbacks to populate fresh data
				v = calculateVelocity(curr->relative_heading,
									  distance_traveled,
									  odom_latest.twist.twist.angular.z,
									  OMEGA_MAX, ALPHA_MAX);
				ROS_INFO("Cmd omega: %f", v);
				cmd_twist.angular.z = v;
				// send command
				cmd_publisher.publish(cmd_twist);
				// update the distance traveled
				distance_traveled = 2.0 * (atan2(odom_latest.pose.pose.orientation.z,
												 odom_latest.pose.pose.orientation.w)
										 - atan2(odom_start.pose.pose.orientation.z,
												 odom_start.pose.pose.orientation.w));
				distance_remaining = curr->relative_heading - distance_traveled;
				sleep_timer.sleep(); // sleep
			}
			// now, go the desired distance
			distance_traveled = 0.0;
			distance_remaining = curr->linear_distance;
			cmd_twist.angular.z = 0.0; // don't rotate
			while (ros::ok() &&
				   distance_remaining > 0.0) {
				ros::spinOnce(); // allow callbacks to populate fresh data
				v = calculateVelocity(curr->linear_distance,
									  distance_traveled,
									  odom_latest.twist.twist.linear.x,
									  V_MAX, A_MAX);
				ROS_INFO("Cmd velocity: %f", v);
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
			// We've reached our destination, so get rid of the PathCoord
			path_queue.pop();
		}
	}
};

int main(int argc, char **argv) {
	ros::init(argc,argv,"robot0_commander"); // name of this node 
	ros::NodeHandle nh; // two lines to create a publisher object that can talk to ROS
	//stdr "robot0" is expecting to receive commands on topic: /robot0/cmd_vel

	PathCoord pc1(0.0, 4.8),
			  pc2(PI/2.0, 12.2),
			  pc3(PI/2.0, 0.0);
			  
	RobotCommands::init(nh);
	RobotCommands::add(&pc1);
	// RobotCommands::add(&pc2);
	// RobotCommands::add(&pc3);
	RobotCommands::run();

	ROS_INFO("Destination reached!");

	return 0;
} 