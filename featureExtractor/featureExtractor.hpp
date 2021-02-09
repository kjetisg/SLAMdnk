/*! \file
I dunno, some class associated to a feature extractor..?
*/

#pragma once


// #include "opencv4/opencv2/videoio/videoio.hpp"
// #include "opencv4/opencv2/imgcodecs/imgcodecs.hpp"



#include "opencv2/opencv.hpp"
#include "opencv4/opencv2/objdetect.hpp"
#include "opencv2/videoio/videoio.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgcodecs.hpp"


// #include "gst/gst.h"

#include "gtsam/geometry/Point2.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/slam/PriorFactor.h"
#include "gtsam/slam/ProjectionFactor.h"
#include "gtsam/slam/GeneralSFMFactor.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/DoglegOptimizer.h"
#include "gtsam/nonlinear/Values.h"

#include "spdlog/spdlog.h"

#include <cstdio>
#include <regex>
#include <memory>
#include <stdexcept>

struct SFM_Helper
{
    struct ImagePose
    {
        cv::Mat img; // down sampled image used for display
        cv::Mat desc; // feature descriptor
        std::vector<cv::KeyPoint> kp; // keypoint

        cv::Mat T; // 4x4 pose transformation matrix
        cv::Mat P; // 3x4 projection matrix

        // alias to clarify map usage below
        using kp_idx_t = size_t;
        using landmark_idx_t = size_t;
        using img_idx_t = size_t;

        std::map<kp_idx_t, std::map<img_idx_t, kp_idx_t>> kp_matches; // keypoint matches in other images
        std::map<kp_idx_t, landmark_idx_t> kp_landmark; // seypoint to 3d points

        // helper
        kp_idx_t& kp_match_idx(size_t kp_idx, size_t img_idx) { return kp_matches[kp_idx][img_idx]; };
        bool kp_match_exist(size_t kp_idx, size_t img_idx) { return kp_matches[kp_idx].count(img_idx) > 0; };

        landmark_idx_t& kp_3d(size_t kp_idx) { return kp_landmark[kp_idx]; }
        bool kp_3d_exist(size_t kp_idx) { return kp_landmark.count(kp_idx) > 0; }
    };

    // 3D point
    struct Landmark
    {
        cv::Point3f pt;
        int seen = 0; // how many cameras have seen this point
    };

    std::vector<ImagePose> img_pose;
    std::vector<Landmark> landmark;
};

class FeatureExtractor
{
public:
    FeatureExtractor();

    bool startCameraStream();
    bool startFileStream();
    void setStreamUrl(const std::string & url_str);
    void SFM_example();

protected:

private:
    std::string m_imagePath_;
    std::string m_videoUrl_;
    static constexpr std::uint16_t UdpPort { 10901 }
    ;
    std::unique_ptr<cv::VideoCapture> m_imgStream_;
    cv::Mat m_currentImage_;

    const int IMAGE_DOWNSAMPLE = 2;
};


/*
Run image server on Windows: (use specific camera device)
ffmpeg.exe -f dshow -i video="Logitech Webcam C930e" -preset ultrafast -tune zerolatency -vcodec libx264 -r 10 -b:v 2014k -s 640x640 -ab 32k -ar 44100 -f mpegts -flush_packets 0 udp://172.19.243.9:10901?pkt_size=1316
*/




