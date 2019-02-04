#include <array>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>
#include <memory>
#include <algorithm>

#include <vision/VisionRunner.h>
#include <vision/VisionPipeline.h>

#include <cameraserver/CameraServer.h>
#include <networktables/NetworkTableInstance.h>

#include <wpi/json.h>
#include <wpi/StringRef.h>
#include <wpi/raw_istream.h>
#include <wpi/raw_ostream.h>

#include <opencv2/opencv.hpp>

/*
   JSON format:
   {
       "team": <team number>,
       "ntmode": <"client" or "server", "client" if unspecified>
       "cameras": [
           {
               "name": <camera name>
               "path": <path, e.g. "/dev/video0">
               "pixel format": <"MJPEG", "YUYV", etc>   // optional
               "width": <video mode width>              // optional
               "height": <video mode height>            // optional
               "fps": <video mode fps>                  // optional
               "brightness": <percentage brightness>    // optional
               "white balance": <"auto", "hold", value> // optional
               "exposure": <"auto", "hold", value>      // optional
               "properties": [                          // optional
                   {
                       "name": <property name>
                       "value": <property value>
                   }
               ],
               "stream": {                              // optional
                   "properties": [
                       {
                           "name": <stream property name>
                           "value": <stream property value>
                       }
                   ]
               }
           }
       ]
   }
 */

#define VISION_TARGET_COUNT 2

namespace vision {

    static const char* k_ConfigFileName = "/boot/frc.json";

    static const wpi::json k_VisionConfig{{"Lower Hue",        1.0},
                                          {"Lower Saturation", 90.0},
                                          {"Lower Value",      110.0},
                                          {"Upper Hue",        85.0},
                                          {"Upper Saturation", 255.0},
                                          {"Upper Value",      255.0}};

    std::shared_ptr<nt::NetworkTable> networkTable;
    unsigned int teamNumber;
    bool isServer = false;

    struct CameraConfig {
        std::string name, path;
        wpi::json cameraConfig, streamConfig;
    };

    std::vector<CameraConfig> cameraConfigs;

    wpi::raw_ostream& ParseError() {
        return wpi::errs() << "config error in '" << k_ConfigFileName << "': ";
    }

    bool ReadCameraConfig(const wpi::json& config) {
        CameraConfig cameraConfig;
        try {
            cameraConfig.name = config.at("name").get<std::string>();
        } catch (const wpi::json::exception& exception) {
            ParseError() << "could not read camera name: " << exception.what() << '\n';
            return false;
        }
        try {
            cameraConfig.path = config.at("path").get<std::string>();
        } catch (const wpi::json::exception& exception) {
            ParseError() << "camera '" << cameraConfig.name << "': could not read path: " << exception.what() << '\n';
            return false;
        }
        if (config.count("stream") != 0) cameraConfig.streamConfig = config.at("stream");
        cameraConfig.cameraConfig = config;
        cameraConfigs.emplace_back(std::move(cameraConfig));
        return true;
    }

    bool ReadConfig() {
        std::error_code errorCode;
        wpi::raw_fd_istream configFile(k_ConfigFileName, errorCode);
        if (errorCode) {
            wpi::errs() << "could not open '" << k_ConfigFileName << "': " << errorCode.message() << '\n';
            return false;
        }
        wpi::json config;
        try {
            config = wpi::json::parse(configFile);
        } catch (const wpi::json::parse_error& parse_error) {
            ParseError() << "byte " << parse_error.byte << ": " << parse_error.what() << '\n';
            return false;
        }
        if (!config.is_object()) {
            ParseError() << "must be JSON object\n";
            return false;
        }
        try {
            teamNumber = config.at("team").get<unsigned int>();
        } catch (const wpi::json::exception& exception) {
            ParseError() << "could not read team number: " << exception.what() << '\n';
            return false;
        }
        if (config.count("ntmode") != 0) {
            try {
                auto networkModeConfig = config.at("ntmode").get<std::string>();
                wpi::StringRef networkMode(networkModeConfig);
                if (networkMode.equals_lower("client")) {
                    isServer = false;
                } else if (networkMode.equals_lower("server")) {
                    isServer = true;
                } else {
                    ParseError() << "could not understand network mode value '" << networkModeConfig << "'\n";
                }
            } catch (const wpi::json::exception& exception) {
                ParseError() << "could not read network mode: " << exception.what() << '\n';
            }
        }
        try {
            for (auto&& camera : config.at("cameras")) {
                if (!ReadCameraConfig(camera)) return false;
            }
        } catch (const wpi::json::exception& exception) {
            ParseError() << "could not read cameras: " << exception.what() << '\n';
            return false;
        }
        return true;
    }

    cs::UsbCamera StartCamera(const CameraConfig& config) {
        wpi::outs() << "Starting camera '" << config.name << "' on " << config.path << '\n';
        auto cameraServer = frc::CameraServer::GetInstance();
        cs::UsbCamera camera{config.name, config.path};
        auto capture = cameraServer->StartAutomaticCapture(camera);
        wpi::outs() << "Using OpenCV version: " << cv::getVersionString() << '\n';
        std::thread([&] {
            cs::CvSink sink = cameraServer->GetVideo();
            cs::CvSource outputStream = cameraServer->PutVideo("Processed rPi 0", 160, 90);
            cs::CvSource maskStream = cameraServer->PutVideo("Mask", 160, 90);
            outputStream.SetConnectionStrategy(cs::VideoSource::kConnectionKeepOpen);
            maskStream.SetConnectionStrategy(cs::VideoSource::kConnectionKeepOpen);
            cv::Mat bgr, hsv, hsvBlur, hsvErode, edged, mask, output = cv::Mat::zeros(cv::Size(160, 90), CV_8UC3);
            while (outputStream.IsEnabled()) {
                output.setTo(cv::Scalar(0));
                sink.GrabFrame(bgr);
                std::vector<std::vector<cv::Point>> contours;
                std::vector<cv::Vec4i> hierarchy;
                if (!bgr.empty()) {
                    cv::cvtColor(bgr, hsv, CV_BGR2HSV);
                    hsv.copyTo(hsvBlur);
//                    cv::medianBlur(hsv, hsvBlur, 3);
//                    hsvBlur.copyTo(hsvErode);
                    cv::morphologyEx(hsvBlur, hsvErode, cv::MORPH_CLOSE, cv::Mat::ones(cv::Size(3, 3), CV_8UC1));
//                    cv::erode(hsvBlur, hsvErode, cv::Mat::ones(cv::Size(3, 3), CV_8UC1));
                    cv::Scalar
                            lowerGreen(vision::networkTable->GetNumber("Lower Hue", k_VisionConfig.at("Lower Hue").get<double>()),
                                       vision::networkTable->GetNumber("Lower Saturation", k_VisionConfig.at("Lower Saturation").get<double>()),
                                       vision::networkTable->GetNumber("Lower Value", k_VisionConfig.at("Lower Value").get<double>())),
                            upperGreen(vision::networkTable->GetNumber("Upper Hue", k_VisionConfig.at("Upper Hue").get<double>()),
                                       vision::networkTable->GetNumber("Upper Saturation", k_VisionConfig.at("Upper Saturation").get<double>()),
                                       vision::networkTable->GetNumber("Upper Value", k_VisionConfig.at("Upper Value").get<double>()));
                    cv::inRange(hsvErode, lowerGreen, upperGreen, mask);
                    mask.copyTo(edged);
//                    cv::Canny(mask, edged, 30, 30*3);
                    cv::findContours(edged, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point());
                    const int contoursDetected = static_cast<const int>(contours.size());
                    cv::putText(output, std::to_string(contoursDetected), cv::Point(10, 160), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 255), 2,
                                CV_AA);
                    if (contoursDetected >= VISION_TARGET_COUNT) {
                        std::partial_sort(contours.begin(), contours.begin() + VISION_TARGET_COUNT, contours.end(), [](const std::vector<cv::Point>& p1, const std::vector<cv::Point>& p2) -> bool {
                            return cv::contourArea(p1) > cv::contourArea(p2);
                        });
                        std::vector<cv::Point>&
                                firstLargestContour = contours[0],
                                secondLargestContour = contours[1];
                        bool leftFirst = firstLargestContour[0].x < secondLargestContour[0].x;
                        std::vector<cv::Point>&
                                leftContour = leftFirst ? firstLargestContour : secondLargestContour,
                                rightContour = leftFirst ? secondLargestContour : firstLargestContour;
                        std::vector<std::vector<cv::Point>> tapeContours{leftContour, rightContour};
                        double leftEpsilon = cv::arcLength(leftContour, true) * 0.025, rightEpsilon = cv::arcLength(rightContour, true) * 0.025;
                        std::vector<cv::Point> leftTape, rightTape;
                        cv::approxPolyDP(leftContour, leftTape, leftEpsilon, true);
                        cv::approxPolyDP(rightContour, rightTape, rightEpsilon, true);
                        cv::bitwise_and(bgr, bgr, output, mask);
                        std::vector<std::vector<cv::Point>> tapes{leftTape, rightTape};
//                        cv::drawContours(output, std::vector<std::vector<cv::Point>>{leftContour}, -1, cv::Scalar(0, 255, 255), 2);
//                        cv::drawContours(output, std::vector<std::vector<cv::Point>>{rightContour}, -1, cv::Scalar(255, 0, 255), 1);
//                        cv::drawContours(output, contours, -1, cv::Scalar(255, 0, 255), 1);
                        cv::drawContours(output, tapes, -1, cv::Scalar(255, 0, 255), 2);
                    }
                }
                if (!mask.empty())
                    maskStream.PutFrame(mask);
                outputStream.PutFrame(output);
            }
        }).detach();
        camera.SetConfigJson(config.cameraConfig);
        camera.SetConnectionStrategy(cs::VideoSource::kConnectionKeepOpen);
        if (config.streamConfig.is_object())
            capture.SetConfigJson(config.streamConfig);
        return camera;
    }

    class MyPipeline : public frc::VisionPipeline {
    public:
        void Process(cv::Mat& image) override {
        }
    };
}

int main(int argc, char* argv[]) {
    if (argc >= 2) vision::k_ConfigFileName = argv[1];
    if (!vision::ReadConfig()) return EXIT_FAILURE;
    auto networkTableInstance = nt::NetworkTableInstance::GetDefault();
    vision::networkTable = networkTableInstance.GetTable("Garage Robotics Vision");
    for (auto item : vision::k_VisionConfig.items())
        vision::networkTable->PutNumber(item.key(), item.value());
    if (vision::isServer) {
        wpi::outs() << "Setting up NetworkTables server\n";
        networkTableInstance.StartServer();
    } else {
        wpi::outs() << "Setting up NetworkTables client for team " << vision::teamNumber << '\n';
        networkTableInstance.StartClientTeam(vision::teamNumber);
    }
    std::vector<cs::VideoSource> cameras;
    for (auto&& cameraConfig : vision::cameraConfigs)
        cameras.emplace_back(StartCamera(cameraConfig));
    if (!cameras.empty()) {
        std::thread([&] {
            frc::VisionRunner<vision::MyPipeline> runner(cameras[0], new vision::MyPipeline(),
                                                         [&](vision::MyPipeline& pipeline) {

                                                         });
            runner.RunForever();
        }).join();
    }
    return EXIT_SUCCESS;
}
