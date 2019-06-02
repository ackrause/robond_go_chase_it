#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

class ProcessImage {
public:
	ProcessImage() {
		// Set up client to request services from command_robot
		_client = _n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

		// Subscribe to /camera/rgb/image_raw topic to read the camera data 
		// inside the process_image_callback function
		_img_sub = _n.subscribe("/camera/rgb/image_raw", 10, &ProcessImage::find_and_follow_white_ball, this);

		// ensure that the first command sent to robot is not debounced
		_current_linear_velocity = -999999;
		_current_angular_velocity = -999999;
	}

private:
	const float STOP = 0;						// stop linear and/or angular movement
	const float FORWARD = 0.125;	 			// "max" forward velocity
	const float TURN_COUNTERCLOCKWISE = 0.5; 	// "max" counter-clockwise rotational velocity
	const float TURN_CLOCKWISE = -0.5; 			// "max" clockwise rotational velocity

	const int MAX_RGB_VALUE = 255;	// maximum value for a color channel in image

	// current linear and angular velocities (used for basic drive command de-bouncing)
	float _current_linear_velocity;
	float _current_angular_velocity;

	// Client for requesting services
	ros::ServiceClient _client;

	// Node handler
	ros::NodeHandle _n;

	// Subscriber for the image data
	ros::Subscriber _img_sub;

	// This function calls the command_robot service to drive the robot in the specified direction
	void drive_robot(float lin_x, float ang_z) {
		ball_chaser::DriveToTarget srv;
		srv.request.linear_x = lin_x;
		srv.request.angular_z = ang_z;

		// Don't bother sending the request if nothing will change from what robot is currently doing
		if (lin_x == _current_linear_velocity && ang_z == _current_angular_velocity) {
			return;
		}

		if(_client.call(srv)) {
			_current_linear_velocity = lin_x;
			_current_angular_velocity = ang_z;
		} else {
			ROS_ERROR("Failed to call service drive_to_target");
		}
	}

	// This callback function attempts to keep the robot moving towards any white balls it sees
	// If no white balls are visible, it stops until a white ball appears
	void find_and_follow_white_ball(const sensor_msgs::Image img) {
		int white_pixel_column = -1;
		int max_left_pos = img.step / 3, max_center_pos = 2 * img.step / 3; // TODO: parameterize these
		float linear_velocity, angular_velocity;

		// Technically searches for a white pixel rather than a white ball, 
		// so it can be confused by e.g. white walls
		for(int i = 0; i < img.height * img.step; i += 3) {
			if(img.data[i] == MAX_RGB_VALUE && 		// red channel
			   img.data[i+1] == MAX_RGB_VALUE && 	// green channel
			   img.data[i+2] == MAX_RGB_VALUE) {  	// white channel
				white_pixel_column = i % img.step;
				break;
			}
		}

		if(white_pixel_column == -1) {
			// Stop if white ball is not visible
			linear_velocity = STOP;
			angular_velocity = STOP;
		} else if (white_pixel_column <= max_left_pos) {
			// Turn counter-clockwise if ball is in the left third of image
			linear_velocity = STOP;
			angular_velocity = TURN_COUNTERCLOCKWISE;
		} else if (white_pixel_column <= max_center_pos) {
			// Go straight forward if ball is in center of image
			linear_velocity = FORWARD;
			angular_velocity = STOP;
		} else {
			// Turn clockwise if ball is in right third of image
			linear_velocity = STOP;
			angular_velocity = TURN_CLOCKWISE;
		}

		drive_robot(linear_velocity, angular_velocity);

		return;
	}
};

int main(int argc, char **argv) {
	// Initialize the ProcessImage node
	ros::init(argc, argv, "process_image");
	ProcessImage ProcessImageNode;

	// Handle ROS communication events
	ros::spin();

	return 0;
}
