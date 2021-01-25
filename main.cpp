/*! \file

Main file for SLAMdnk application.

*/

//! Just some includes to check availability
#include "featureExtractor/featureExtractor.hpp"

//Libraries
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>

//stdlib
#include <filesystem>
#include <iostream>
#include <vector>

void test_gtsam_functionality()
{
    const double deg2rad = M_PI / 180;
    gtsam::Rot2 prior = gtsam::Rot2::fromAngle(30 * deg2rad);
    prior.print("30 degrees in radians is");
    gtsam::noiseModel::Isotropic::shared_ptr model = gtsam::noiseModel::Isotropic::Sigma(1, 1 * deg2rad);
    gtsam::Symbol key('x',1);
    gtsam::PriorFactor<gtsam::Rot2> factor(key, prior, model);
}

void test_eigen_functionality()
{
    Eigen::Matrix3d rot_mat;
    rot_mat <<  1, 0, 0, 
                0, 1, 0, 
                0, 0, 1;

    std::cout << "Eigen matrix example:\n" << rot_mat << std::endl; 
}

void test_opencv_functionality(int argc, char* argv[])
{
    if ( argc != 2 )
    {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return ;
    }

    cv::Mat image;
    image = cv::imread( argv[1], 1 );
    if ( !image.data )
    {
        printf("No image data \n");
        return;
    }
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
    cv::imshow("Display Image", image);
    cv::waitKey(0);
}


int main(int argc, char* argv[])
{
    test_opencv_functionality(argc, argv);
    test_eigen_functionality();
    test_gtsam_functionality();

    return 0;
}