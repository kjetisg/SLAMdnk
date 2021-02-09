/*! \file

Main file for SLAMdnk application.

*/

//! Just some includes to check availability
#include "featureExtractor/featureExtractor.hpp"

//Libraries
#include "opencv2/opencv.hpp"
#include "Eigen/Core"
#include "gtsam/geometry/Rot2.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/slam/PriorFactor.h"

#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"

#include "boost/program_options.hpp"

//stdlib
#include <filesystem>
#include <iostream>
#include <vector>

namespace po = boost::program_options;

void test_gtsam_functionality()
{
    const double deg2rad = M_PI / 180;
    gtsam::Rot2 prior = gtsam::Rot2::fromAngle(30 * deg2rad);
    prior.print("30 degrees in radians is");
    gtsam::noiseModel::Isotropic::shared_ptr model = gtsam::noiseModel::Isotropic::Sigma(1, 1 * deg2rad);
    gtsam::Symbol key('x',1);
    gtsam::PriorFactor<gtsam::Rot2> factor(key, prior, model);
    spdlog::debug("Conversion deg2rad = {}", deg2rad);
}

void test_eigen_functionality()
{
    Eigen::Matrix3d rot_mat;
    rot_mat <<  1, 0, 0, 
                0, 1, 0, 
                0, 0, 1;

    std::cout << "Eigen matrix example:\n" << rot_mat << std::endl; 
    spdlog::info("Matrix determinant = {}", rot_mat.determinant());
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
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("image_stream", po::value<std::string>(), "set image stream source");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    FeatureExtractor fex;

    if (vm.count("help")) 
    {
        std::cout << desc << "\n";
        return 0;
    }
    if (vm.count("image_stream")) 
    {
        auto const stream_src = vm["image_stream"].as<std::string>();
        std::cout << "Image stream source was set to " << stream_src << ".\n";
        if ( !std::filesystem::exists(stream_src) )
        {
            fex.setStreamUrl(stream_src);
            if (fex.startCameraStream())
            {
                fex.SFM_example();
            }
        }
        else // Assume file path-stream
        {
            if (fex.startFileStream())
                fex.SFM_example();
        }
    } 
    else 
        std::cout << "Image stream source was not set.\n";

    // create color multi threaded logger
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::debug);
    console_sink->set_pattern("[%H:%M:%S] [%n] [%^%l%$] [thread %t] %v");

    std::vector<spdlog::sink_ptr> sinks{console_sink}; //Add more sinks as necessary
    auto logger = std::make_shared<spdlog::logger>("SLAMdnk_logger", std::begin(sinks), std::end(sinks));
    spdlog::set_default_logger(logger);
    logger->set_level(spdlog::level::debug);

    test_opencv_functionality(argc, argv);
    test_eigen_functionality();
    test_gtsam_functionality();

    // spdlog::info(cv::getBuildInformation());
    /*
    Example command to start image server on Windows using ffmpeg: (use specific camera device and correct WSL2 IP address)
    ffmpeg.exe -f dshow -i video="Logitech Webcam C930e" -preset ultrafast -tune zerolatency -vcodec libx264 -r 10 -b:v 2014k -s 640x640 -ab 32k -ar 44100 -f mpegts -flush_packets 0 udp://172.19.243.9:10901?pkt_size=1316
    */

    return 0;
}