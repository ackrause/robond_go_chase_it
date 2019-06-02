#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

class DriveBot {
public:
	DriveBot() {
		// Inform ROS master that we'll be publishing a message of type geometry_msgs::Twist on the
		// robot actuation topic with a publishing queue size of 10
		_motor_command_pub = _n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

		// Define a command_robot service
		_command_robot = _n.advertiseService("/ball_chaser/command_robot", &DriveBot::handle_drive_request, this);
		ROS_INFO("Ready to send wheel commands");
	}

	bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res) {
		float req_linear_x = (float) req.linear_x;
		float req_angular_z = (float) req.angular_z;

		// Log the request for debugging
		ROS_INFO("DriveToTargetRequest received - linear_x:%1.2f, angular_z:%1.2f", req_linear_x, req_angular_z);

		// Create a motor_command object of type geometry_msgs::Twist
        geometry_msgs::Twist motor_command;
        
        // Set wheel velocities
        motor_command.linear.x = req_linear_x;
        motor_command.angular.z = req_angular_z;
        
        // Publish angles to drive the robot
        _motor_command_pub.publish(motor_command);

        // Return a response message
        res.msg_feedback = "Velocities set - linear_x: " + std::to_string(req_linear_x) + ", angular_z: " + std::to_string(req_angular_z);
        ROS_INFO_STREAM(res.msg_feedback);

        return true;
	}

private:
	// motor command publisher
	ros::Publisher _motor_command_pub;

	// Node handler
	ros::NodeHandle _n;

	// Command Robot service
	ros::ServiceServer _command_robot;
};

int main(int argc, char** argv) {
	// initialize the drive_bot node
	ros::init(argc, argv, "drive_bot");
	DriveBot DriveBotNode;

	// Handle ROS communication events
	ros::spin();

	return 0;
}
