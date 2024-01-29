#include "gtest/gtest.h"
#include "camera/single_sphere.hpp"
#include <cmath>

class SingleSphereTest : public ::testing::Test {
protected:
    SingleSphere camera;
};

TEST_F(SingleSphereTest, TestInitParams) {
    camera.init_params(500.0, 250.0, 250.0);
    std::vector<double> expected_params = {500.0, 500.0, 250.0, 250.0, 0.5};
    ASSERT_EQ(camera.get_params(), expected_params);
}

TEST_F(SingleSphereTest, TestConstructorWithParams) {
    std::vector<double> params = {500.0, 500.0, 250.0, 250.0, 0.5};
    SingleSphere camera_with_params(params);
    ASSERT_EQ(camera_with_params.get_params(), params);
}

TEST_F(SingleSphereTest, TestProjectAndUnproject) {
    const double sqrt3 = std::sqrt(3.0);

    camera.init_params(500.0, 250.0, 250.0);
    cv::Point3d point(1.0 / sqrt3, 1.0 / sqrt3, 1.0 / sqrt3);

    cv::Point2d projected_point = camera.project(point);
    cv::Point3d unprojected_point = camera.unproject(projected_point);

    ASSERT_NEAR(unprojected_point.x, point.x, 1e-6);
    ASSERT_NEAR(unprojected_point.y, point.y, 1e-6);
    ASSERT_NEAR(unprojected_point.z, point.z, 1e-6);
}