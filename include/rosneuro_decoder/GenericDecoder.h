#ifndef ROSNEURO_DECODER_GENERICDECODER_H_
#define ROSNEURO_DECODER_GENERICDECODER_H_

#include <Eigen/Dense>
#include <ros/ros.h>
#include <gtest/gtest_prod.h>

namespace rosneuro {
	namespace decoder {

class GenericDecoder {

	public:
		GenericDecoder(void);
		virtual ~GenericDecoder(void);

		virtual bool configure(void) = 0;
		virtual Eigen::VectorXf apply(const Eigen::VectorXf& in) = 0;
		virtual Eigen::VectorXf getFeatures(const Eigen::MatrixXf& in) = 0;
		std::string name(void);
		void setname(const std::string& name);
		virtual std::string path(void) = 0;
		virtual std::vector<int> classes(void) = 0;
		virtual bool isSet(void);

	private:
		std::string name_;

	protected:
        bool        is_configured_;
		bool load_eigen(const std::string centers_str, Eigen::Ref<Eigen::MatrixXf> out);
        bool load_vectorOfVector(const std::string current_str, std::vector<std::vector<uint32_t>>& out);
                
	public:
		bool configure(const std::string& param_name);
		bool configure(XmlRpc::XmlRpcValue& config);
		std::string type(void) const;
		std::string name(void) const;


		bool getParam(const std::string& name, std::string& value) const;
		bool getParam(const std::string& name, int& value) const;
		bool getParam(const std::string& name, double& value) const;
		bool getParam(const std::string& name, unsigned  int& value) const;
		bool getParam(const std::string& name, std::vector<uint32_t>& value) const;
		bool getParam(const std::string& name, std::vector<double>& value) const;

	protected:
		bool loadConfiguration(XmlRpc::XmlRpcValue& config);
		std::map<std::string, XmlRpc::XmlRpcValue> params_;

	private:
		bool setNameAndType(XmlRpc::XmlRpcValue& config);
		ros::NodeHandle nh_;

        FRIEND_TEST(TestDecoderSuite, Path);
        FRIEND_TEST(TestDecoderSuite, Name);
        FRIEND_TEST(TestDecoderSuite, Classes);
        FRIEND_TEST(TestGenericDecoderSuite, Constructor);
        FRIEND_TEST(TestGenericDecoderSuite, LoadVectorOfVector);
        FRIEND_TEST(TestGenericDecoderSuite, LoadEigen);
        FRIEND_TEST(TestGenericDecoderSuite, LoadEigenFail);
        };

	}

}


#endif