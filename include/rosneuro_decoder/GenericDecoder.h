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
                std::string getName(void);
                void setName(const std::string& name);
                virtual std::string getPath(void) = 0;
                virtual std::vector<int> getClasses(void) = 0;
                virtual bool isSet(void);

                bool configure(const std::string& param_name);
                bool configure(XmlRpc::XmlRpcValue& config);

                std::string getType(void) const;
                std::string getName(void) const;

                bool getParam(const std::string& name, std::string& value) const;
                bool getParam(const std::string& name, int& value) const;
                bool getParam(const std::string& name, double& value) const;
                bool getParam(const std::string& name, unsigned  int& value) const;
                bool getParam(const std::string& name, std::vector<uint32_t>& value) const;
                bool getParam(const std::string& name, std::vector<double>& value) const;

            protected:
                bool loadConfiguration(XmlRpc::XmlRpcValue& config);
                std::map<std::string, XmlRpc::XmlRpcValue> params_;
                bool loadEigen(const std::string centers_str, Eigen::Ref<Eigen::MatrixXf> out);
                bool loadVectorOfVector(const std::string current_str, std::vector<std::vector<uint32_t>>& out);
                bool is_configured_;

            private:
                template<typename T>
                bool parseStringToMatrix(const std::string current_str, std::vector<std::vector<T>>& out);
                bool validateMatrixDimensions(const std::vector<std::vector<float>>& matrix, Eigen::Ref<Eigen::MatrixXf> out);
                void populateEigenMatrix(const std::vector<std::vector<float>>& input, Eigen::Ref<Eigen::MatrixXf> output);
                bool setNameAndType(XmlRpc::XmlRpcValue& config);

                ros::NodeHandle nh_;
                std::string name_;

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