#include "featureExtractor.hpp"

std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

bool FeatureExtractor::startFileStream()
{
    m_imgStream_ = std::make_unique<cv::VideoCapture>(m_imagePath_);
    return m_imgStream_->isOpened();
}

FeatureExtractor::FeatureExtractor()
{
    // m_imagePath_ = "~/DEV/source/data/tum_vision_mono/sequence_07/images/%05d.jpg";
    m_imagePath_ = R"(/home/kjetilsg/DEV/source/data/tum_vision_mono/sequence_07/images/00000.jpg)";
    // m_videoUrl_ = "udp://@172.19.240.1:10901"; 

    //! Snippet to get WSL2 IP address automatically:
    std::string wsl2_ip;
    std::smatch match;
    std::regex reg{R"(\sinet ((?:\d+\.){3}\d+)/)"};
    auto cmd_output = exec("ip -4 addr show dev eth0");

    {//! Find IP address of WSL2 ethernet adapter:
        std::regex_search (cmd_output, match, reg);
        cmd_output = match.suffix().str();
        m_videoUrl_ = match[1];
        m_videoUrl_ = "udp://@"+m_videoUrl_+":10901";
    }
    spdlog::info("IP address: {}", m_videoUrl_);

    try
    {
        m_imgStream_ = std::make_unique<cv::VideoCapture>(cv::CAP_FFMPEG);
    }
    catch(...)
    {
        spdlog::warn("Could not instantiate video capture with current setup.");
    }
}

void FeatureExtractor::setStreamPath(const std::string & path)
{
    m_imagePath_ = path;
}

void FeatureExtractor::setStreamUrl(const std::string & url_str)
{
    m_videoUrl_ = "udp://@" + m_videoUrl_ + ":" + std::to_string(UdpPort);
}

bool FeatureExtractor::startCameraStream()
{
    if (!m_imgStream_)
        return false;

    if ( !m_videoUrl_.empty() )
        m_imgStream_->open(m_videoUrl_, cv::CAP_FFMPEG);

    return m_imgStream_->isOpened();
}
enum {GAUSSIAN, BLUR, MEDIAN};
int sigma = 3;
int smoothType = GAUSSIAN;
void FeatureExtractor::SFM_example()
{
    spdlog::info("Starting SFM example...");
    SFM_Helper SFM;
    {
        cv::Ptr<cv::AKAZE> feature = cv::AKAZE::create();
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
        
        cv::namedWindow("img", cv::WINDOW_NORMAL);
        
        //! Extract features:
        auto counter = 0;
        while (m_imgStream_->isOpened() && counter < 10)
        {
            cv::namedWindow("Laplacian", cv::WINDOW_AUTOSIZE);
            cv::createTrackbar("Sigma", "Laplacian", &sigma, 15, 0);
            cv::Mat smoothed, laplace, result;
            for(;;)
            {
                cv::Mat frame;
                m_imgStream_->read(frame);
                if( frame.empty() )
                    break;
                int ksize = (sigma*5)|1;
                if(smoothType == GAUSSIAN)
                    cv::GaussianBlur(frame, smoothed, cv::Size(ksize, ksize), sigma, sigma);
                else if(smoothType == BLUR)
                    cv::blur(frame, smoothed, cv::Size(ksize, ksize));
                else
                    cv::medianBlur(frame, smoothed, ksize);
                cv::Laplacian(smoothed, laplace, CV_16S, 5);
                cv::convertScaleAbs(laplace, result, (sigma+1)*0.25);
                cv::imshow("Laplacian", result);
                char c = (char)cv::waitKey(30);
                if( c == ' ' )
                    smoothType = smoothType == GAUSSIAN ? BLUR : smoothType == BLUR ? MEDIAN : GAUSSIAN;
                if( c == 'q' || c == 'Q' || c == 27 )
                    break;
            }

            break;



            cv::Mat frame;
            if (m_imgStream_->read(frame))
            {
                spdlog::debug("Image received, processing...");
                SFM_Helper::ImagePose pose;
                
                if (!frame.empty())
                {
                    cv::resize(frame, frame, frame.size()/IMAGE_DOWNSAMPLE);
                    pose.img = frame;
                    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);

                    feature->detect(frame, pose.kp);
                    feature->compute(frame, pose.kp, pose.desc);

                    SFM.img_pose.push_back(pose);
                    counter++;
                }
            }
            else
            {
                break;
            }
        }

        // Match features between all images
        for (size_t i=0; i < SFM.img_pose.size()-1; i++) 
        {
            auto &img_pose_i = SFM.img_pose[i];
            for (size_t j=i+1; j < SFM.img_pose.size(); j++) 
            {
                auto &img_pose_j = SFM.img_pose[j];
                std::vector<vector<cv::DMatch>> matches;
                std::vector<cv::Point2f> src, dst;
                std::vector<uchar> mask;
                std::vector<int> i_kp, j_kp;

                // 2 nearest neighbour match
                matcher->knnMatch(img_pose_i.desc, img_pose_j.desc, matches, 2);

                for (auto &m : matches) {
                    if(m[0].distance < 0.7*m[1].distance) {
                        src.push_back(img_pose_i.kp[m[0].queryIdx].pt);
                        dst.push_back(img_pose_j.kp[m[0].trainIdx].pt);

                        i_kp.push_back(m[0].queryIdx);
                        j_kp.push_back(m[0].trainIdx);
                    }
                }

                // Filter bad matches using fundamental matrix constraint
                cv::findFundamentalMat(src, dst, cv::FM_RANSAC, 3.0, 0.99, mask);

                cv::Mat canvas = img_pose_i.img.clone();
                canvas.push_back(img_pose_j.img.clone());

                for (size_t k=0; k < mask.size(); k++) {
                    if (mask[k]) {
                        img_pose_i.kp_match_idx(i_kp[k], j) = j_kp[k];
                        img_pose_j.kp_match_idx(j_kp[k], i) = i_kp[k];
                        // spdlog::info("Canvas size: ({}, {})", canvas.rows, canvas.cols);
                        // spdlog::info("src size: ({})", src.size());
                        // spdlog::info("dst size: ({})", dst.size());
                        cv::line(canvas, src[k], dst[k] + cv::Point2f(0, img_pose_i.img.rows), cv::Scalar(0, 0, 255), 2);
                    }
                }

                int good_matches = cv::sum(mask)[0];
                assert(good_matches >= 10);

                std::cout << "Feature matching " << i << " " << j << " ==> " << good_matches << "/" << matches.size() << std::endl;

                cv::resize(canvas, canvas, canvas.size()/2);

                cv::imshow("img", canvas);
                cv::waitKey(1);
            }
        }
    }
    spdlog::debug("URL video stream stopped.");
}