#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <queue>

/* PathCoord
 * Iteration counts and values for how the robot should move
 */
class PathCoord {
	public:
		// number of iterations for the loop
		const int		iter;
		// value for linear x movement
		const float	linear_x;
		// value for angular z movement
		const float	angular_z;
	
		PathCoord(int i, float x, float z) :
			iter(i),
			linear_x(x),
			angular_z(z) { }
};

int main(int argc, char **argv)
{
ros::init(argc,argv,"robot0_commander"); // name of this node 
ros::NodeHandle nh; // two lines to create a publisher object that can talk to ROS
//stdr "robot0" is expecting to receive commands on topic: /robot0/cmd_vel
// commands are of type geometry_msgs/Twist, but they use only velocity in x dir and
//  yaw rate in z-dir; other 4 fields will be ignored
ros::Publisher cmd_publisher = nh.advertise<geometry_msgs::Twist>("/robot0/cmd_vel",1);
// change topic to command abby...
//ros::Publisher cmd_publisher = nh.advertise<geometry_msgs::Twist>("abby/cmd_vel",1);
int timer_freq = 100;
ros::Rate sleep_timer(timer_freq); //let's make a 100Hz timer

//create a variable of type "Twist", as defined in: /opt/ros/hydro/share/geometry_msgs
// any message published on a ROS topic must have a pre-defined format, so subscribers know how to
// interpret the serialized data transmission
geometry_msgs::Twist twist_cmd;
// look at the components of a message of type geometry_msgs::Twist by typing:
// rosmsg show geometry_msgs/Twist
// It has 6 fields.  Let's fill them all in with some data:
twist_cmd.linear.x = 0.0;
twist_cmd.linear.y = 0.0;
twist_cmd.linear.z = 0.0;
twist_cmd.angular.x = 0.0;
twist_cmd.angular.y = 0.0;
twist_cmd.angular.z = 0.0;

// timer test...print out a message every 1 second
ROS_INFO("count-down");
for (int j=3;j>0;j--) {
    ROS_INFO("%d",j);
    for (int i = 0; i<timer_freq;i++)
        sleep_timer.sleep();
}

std::queue<PathCoord *> path_queue;

// this memory will be freed in the loop
path_queue.push(new PathCoord(12 * timer_freq, 0.4, 0.0)); // start with 12 seconds with linear_x at 0.4 m/sec to move 4.8 m
path_queue.push(new PathCoord( 5 * timer_freq, 0.0, -0.314)); // 5 seconds with angular_z at -0.314 rad/sec to rotate PI/2
path_queue.push(new PathCoord(24.4 * timer_freq, 0.5, 0.0)); // 24.4 seconds at 0.5 m/sec to move 12.2 m
path_queue.push(new PathCoord( 5 * timer_freq, 0.0, -0.314)); // 5 seconds with angular_z at -0.314 rad/sec to rotate PI/2
// keep this coord at the end to stop movement
path_queue.push(new PathCoord( 1, 0.0, 0.0));

while (!path_queue.empty()){
	// fetch target values
	PathCoord *curr = path_queue.front();
	twist_cmd.linear.x = curr->linear_x;
	twist_cmd.angular.z = curr->angular_z;
	for (int i=0; i<curr->iter; i++){
		cmd_publisher.publish(twist_cmd); // really, should only need to publish this once, but no harm done
		sleep_timer.sleep(); // sleep for (remainder of) 1/timer_freq sec
	}
	path_queue.pop();
	// free memory
	delete curr;
}

ROS_INFO("my work here is done");

return 0;
} 