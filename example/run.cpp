#include <ros/ros.h>
#include "rosneuro_decoder/Decoder.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "decoder");
	rosneuro::decoder::Decoder decoder;

	if(!decoder.configure()) {
		ROS_ERROR("[decoder] Configuration failed");
		return -1;
	}

	decoder.run();
	ros::shutdown();
	return 0;
}
