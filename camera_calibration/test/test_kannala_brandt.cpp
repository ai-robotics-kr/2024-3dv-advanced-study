#include "gtest/gtest.h"
#include "camera/kannala_brandt.hpp"
#include <cmath>


class KannalaBrandtTest : public ::testing::Test {
protected:
    KannalaBrandt camera;
};

TEST_F(KannalaBrandtTest, TestInitParams) {
    camera.init_params(500.0, 250.0, 250.0);
    std::vector<double> expected_params = {500.0, 500.0, 250.0, 250.0, -0.2, 0.1, 0.0, 0.0};
    ASSERT_EQ(camera.get_params(), expected_params);
}

TEST_F(KannalaBrandtTest, TestConstructorWithParams) {
    std::vector<double> params = {500.0, 500.0, 250.0, 250.0, -0.2, 0.1, 0.0, 0.0};
    KannalaBrandt camera_with_params(params);
    ASSERT_EQ(camera_with_params.get_params(), params);
}

TEST_F(KannalaBrandtTest, TestProjectAndUnproject) {
    camera.init_params(500.0, 250.0, 250.0);
    cv::Point3d point(1.0, 1.0, 1.0);

    cv::Point2d projected_point = camera.project(point);
    cv::Point3d unprojected_point = camera.unproject(projected_point);

    ASSERT_NEAR(unprojected_point.x, point.x, 1e-6);
    ASSERT_NEAR(unprojected_point.y, point.y, 1e-6);
    ASSERT_NEAR(unprojected_point.z, point.z, 1e-6);
}