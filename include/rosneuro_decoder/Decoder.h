#ifndef ROSNEURO_DECODERS_DECODER_H_
#define ROSNEURO_DECODERS_DECODER_H_

#include <memory>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <pluginlib/class_loader.h>
#include <gtest/gtest_prod.h>
#include <rosneuro_msgs/NeuroOutput.h>
#include "rosneuro_decoder/GenericDecoder.h"

namespace rosneuro {
	namespace decoder {

class Decoder {
	
	public:
		Decoder(void);
		~Decoder(void);

		bool configure(void); 
		void run(void);
		Eigen::VectorXf apply(const Eigen::VectorXf& in);
		Eigen::VectorXf getFeatures(const Eigen::MatrixXf& psd); // freq x channels --> it is one window
		std::string name(void);
		std::string path(void);
		std::vector<int> classes(void);

	private:
		void on_received_data(const rosneuro_msgs::NeuroOutput& msg);

	private:
		Eigen::VectorXf vector_to_eigen(const std::vector<float>& in);
		std::vector<float> eigen_to_vector(const Eigen::VectorXf& in);
		void set_message(const Eigen::VectorXf& data);

	private:
		ros::NodeHandle nh_;
		ros::NodeHandle p_nh_;
		ros::Subscriber	sub_;
		ros::Publisher	pub_;
	
		Eigen::VectorXf input_;
		Eigen::VectorXf output_;

		rosneuro_msgs::NeuroOutput msgoutput_;

		bool has_new_data_;
		bool is_first_message_;
		
		std::string plugin_;
		std::string decodername_;

		boost::shared_ptr<GenericDecoder> 	decoder_;

		std::unique_ptr<pluginlib::ClassLoader<GenericDecoder>> loader_;
        FRIEND_TEST(TestDecoderSuite, Constructor);
        FRIEND_TEST(TestDecoderSuite, Path);
        FRIEND_TEST(TestDecoderSuite, WrongPath);
        FRIEND_TEST(TestDecoderSuite, Name);
        FRIEND_TEST(TestDecoderSuite, Classes);
        FRIEND_TEST(TestDecoderSuite, SetMessage);
        FRIEND_TEST(TestDecoderSuite, VectorToEigen);
        FRIEND_TEST(TestDecoderSuite, EigenToVector);
};

	}

}

#endif
