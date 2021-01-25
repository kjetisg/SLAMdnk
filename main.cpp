/*! \file

Main file for SLAMdnk application.

*/

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include <filesystem>
#include <iostream>
#include <vector>

int main(int argc, char* argv[])
{
    Eigen::Matrix3d rot_mat;
    rot_mat <<  1, 0, 0, 
                0, 1, 0, 
                0, 0, 1;

    std::cout << "Eigen matrix example:\n" << rot_mat << std::endl;

    if ( argc != 2 )
    {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }
    cv::Mat image;
    image = cv::imread( argv[1], 1 );
    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
    cv::imshow("Display Image", image);
    cv::waitKey(0);

    return 0;
}