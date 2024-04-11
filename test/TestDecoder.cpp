#include "Decoder.h"
#include <gtest/gtest.h>
#include <gmock/gmock.h>

namespace rosneuro {
namespace decoder {

class TestDecoder : public Decoder {
    public:
        TestDecoder() : Decoder() {}
        ~TestDecoder() {}
        MOCK_METHOD0(configure, bool());
        MOCK_METHOD1(apply, Eigen::VectorXf(const Eigen::VectorXf& in));
        MOCK_METHOD1(getFeatures, Eigen::VectorXf(const Eigen::MatrixXf& in));
        MOCK_METHOD0(classes, std::vector<int>());
        MOCK_METHOD0(path, std::string());
};

class TestDecoderSuite : public ::testing::Test {
    public:
        TestDecoderSuite() { decoder = new TestDecoder(); }
        ~TestDecoderSuite() { delete decoder; }
        TestDecoder* decoder;
};

TEST_F(TestDecoderSuite, Constructor) {
    ASSERT_EQ(decoder->has_new_data_, false);
    ASSERT_EQ(decoder->is_first_message_, true);
    ASSERT_NE(decoder->loader_, nullptr);
}

TEST_F(TestDecoderSuite, Path){
    EXPECT_CALL(*decoder, path()).Times(1).WillOnce(testing::Return("test"));
    ASSERT_EQ(decoder->path(), "test");
}

TEST_F(TestDecoderSuite, Classes){
    EXPECT_CALL(*decoder, classes()).Times(1).WillOnce(testing::Return(std::vector<int>({})));
    ASSERT_EQ(decoder->classes(), std::vector<int>({}));
}

TEST_F(TestDecoderSuite, Configure){
    EXPECT_CALL(*decoder, configure()).Times(1).WillOnce(testing::Return(true));
    ASSERT_TRUE(decoder->configure());
}

TEST_F(TestDecoderSuite, ConfigureFail){
    EXPECT_CALL(*decoder, configure()).Times(1).WillOnce(testing::Return(false));
    ASSERT_FALSE(decoder->configure());
}

TEST_F(TestDecoderSuite, ConfigureFailPlugin){
    EXPECT_CALL(*decoder, configure()).Times(1).WillOnce(testing::Return(false));
    ASSERT_FALSE(decoder->configure());
}

TEST_F(TestDecoderSuite, Apply){
    Eigen::VectorXf in(3);
    in << 1, 2, 3;

    EXPECT_CALL(*decoder, apply(testing::_)).Times(1).WillOnce(testing::Return(in));

    ASSERT_EQ(in, decoder->apply(in));
}

TEST_F(TestDecoderSuite, GetFeatures){
    Eigen::MatrixXf in(3, 3);
    in << 1, 2, 3;

    EXPECT_CALL(*decoder, getFeatures(testing::_)).Times(1).WillOnce(testing::Return(in));


    ASSERT_EQ(in, decoder->getFeatures(in));
}

TEST_F(TestDecoderSuite, SetMessage){
    Eigen::VectorXf data(3);
    data << 1, 2, 3;

    decoder->set_message(data);

    ASSERT_EQ(decoder->msgoutput_.softpredict.data, std::vector<float>({1, 2, 3}));
}

TEST_F(TestDecoderSuite, VectorToEigen){
    std::vector<float> in = {1, 2, 3};
    Eigen::VectorXf out = decoder->vector_to_eigen(in);
    ASSERT_EQ(in.size(), out.size());
    for (int i = 0; i < in.size(); i++)
        ASSERT_EQ(in[i], out[i]);
}

TEST_F(TestDecoderSuite, EigenToVector){
    Eigen::VectorXf in(3);
    in << 1, 2, 3;
    std::vector<float> out = decoder->eigen_to_vector(in);
    ASSERT_EQ(in.size(), out.size());
    for (int i = 0; i < in.size(); i++)
        ASSERT_EQ(in[i], out[i]);
}

}
}


int main(int argc, char **argv) {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal);
    ros::init(argc, argv, "test_decoder");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
