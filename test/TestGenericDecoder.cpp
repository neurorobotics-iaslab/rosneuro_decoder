#include "GenericDecoder.h"
#include <gtest/gtest.h>

namespace rosneuro {
    namespace decoder {
        class TestGenericDecoder: public GenericDecoder {
            public:
                TestGenericDecoder() : GenericDecoder() {}
                ~TestGenericDecoder() {}
                bool configure(void) { return true; }
                Eigen::VectorXf apply(const Eigen::VectorXf& in) { return Eigen::VectorXf::Zero(1); }
                Eigen::VectorXf getFeatures(const Eigen::MatrixXf& in) { return Eigen::VectorXf::Zero(1); }
                std::string getPath(void) { return ""; }
                std::vector<int> getClasses(void) { return std::vector<int>({}); }
        };

        class TestGenericDecoderSuite : public ::testing::Test {
            public:
                TestGenericDecoderSuite() { genericDecoder = new TestGenericDecoder(); }
                ~TestGenericDecoderSuite() { delete genericDecoder; }
                TestGenericDecoder* genericDecoder;
        };

        TEST_F(TestGenericDecoderSuite, Constructor) {
            ASSERT_EQ(genericDecoder->is_configured_, false);
            ASSERT_EQ(genericDecoder->getName(), "undefined");
        }

        TEST_F(TestGenericDecoderSuite, Path){
            ASSERT_EQ(genericDecoder->getPath(), "");
        }

        TEST_F(TestGenericDecoderSuite, Name){
            ASSERT_EQ(genericDecoder->getName(), "undefined");
            genericDecoder->setName("test");
            ASSERT_EQ(genericDecoder->getName(), "test");
        }

        TEST_F(TestGenericDecoderSuite, Classes){
            ASSERT_EQ(genericDecoder->getClasses(), std::vector<int>({}));
        }

        TEST_F(TestGenericDecoderSuite, IsSet){
            ASSERT_EQ(genericDecoder->isSet(), false);
        }

        TEST_F(TestGenericDecoderSuite, LoadVectorOfVector){
            std::vector<std::vector<uint32_t>> out;
            ASSERT_TRUE(genericDecoder->loadVectorOfVector("1 2 3;4 5 6;", out));
            ASSERT_EQ(out.size(), 2);
            ASSERT_EQ(out[0].size(), 3);
            ASSERT_EQ(out[1].size(), 3);
            ASSERT_EQ(out[0][0], 1);
            ASSERT_EQ(out[0][1], 2);
            ASSERT_EQ(out[0][2], 3);
            ASSERT_EQ(out[1][0], 4);
            ASSERT_EQ(out[1][1], 5);
            ASSERT_EQ(out[1][2], 6);
        }

        TEST_F(TestGenericDecoderSuite, LoadEigen){
            Eigen::MatrixXf out = Eigen::MatrixXf::Zero(3, 3);
            ASSERT_TRUE(genericDecoder->loadEigen("1 2 3;4 5 6;7 8 9;", out));
            ASSERT_EQ(out.rows(), 3);
            ASSERT_EQ(out.cols(), 3);
            ASSERT_EQ(out(0, 0), 1);
            ASSERT_EQ(out(0, 1), 2);
            ASSERT_EQ(out(0, 2), 3);
            ASSERT_EQ(out(1, 0), 4);
            ASSERT_EQ(out(1, 1), 5);
            ASSERT_EQ(out(1, 2), 6);
        }

        TEST_F(TestGenericDecoderSuite, LoadEigenFail){
            Eigen::MatrixXf out;
            ASSERT_FALSE(genericDecoder->loadEigen("1 2 3;4 5 6;7 8 9", out));
            out = Eigen::MatrixXf::Zero(3, 3);
            ASSERT_FALSE(genericDecoder->loadEigen("1 2 3;4 5 6;7 8;", out));
        }

    }
}

int main(int argc, char **argv) {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal);
    ros::init(argc, argv, "test_generic_decoder");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
