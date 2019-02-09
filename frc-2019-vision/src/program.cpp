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
#define FOCAL_LENGTH_MM 3.04
#define SENSOR_WIDTH_MM 2.07
#define SENSOR_HEIGHT_MM 3.68
#define IMAGE_WIDTH_PX 320
#define IMAGE_HEIGHT_PX 180

namespace vision {

    static const char* k_ConfigFileName = "/boot/frc.json";

    static const wpi::json k_VisionConfig{{"Lower Hue",        60.0},
                                          {"Lower Saturation", 90.0},
                                          {"Lower Value",      110.0},
                                          {"Upper Hue",        85.0},
                                          {"Upper Saturation", 255.0},
                                          {"Upper Value",      255.0},
                                          {"Arc Constant",     0.05}};

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
            cs::CvSource outputStream = cameraServer->PutVideo("Processed rPi 0", IMAGE_WIDTH_PX, IMAGE_HEIGHT_PX);
            cs::CvSource maskStream = cameraServer->PutVideo("Mask", IMAGE_WIDTH_PX, IMAGE_HEIGHT_PX);
            outputStream.SetConnectionStrategy(cs::VideoSource::kConnectionKeepOpen);
            maskStream.SetConnectionStrategy(cs::VideoSource::kConnectionKeepOpen);
            std::vector<cv::Point3f> objectPoints{
                    cv::Point3f(34.0f, 0.0f, 0.0f), cv::Point3f(0.0f, 136.0f, 0.0f), cv::Point3f(49.0f, 149.0f, 0.0f),
                    cv::Point3f(85.0f, 13.0f, 0.0f),
                    cv::Point3f(290.0f, 13.0f, 0.0f), cv::Point3f(326.0f, 149.0f, 0.0f), cv::Point3f(377.0f, 136.0f, 0.0f),
                    cv::Point3f(340.0f, 0.0f, 0.0f)
            };
            cv::Mat bgr, hsv, hsvBlur, hsvMorph, edged, mask, output = cv::Mat::zeros(cv::Size(IMAGE_WIDTH_PX, IMAGE_HEIGHT_PX), CV_8UC3);
            while (outputStream.IsEnabled()) {
                output.setTo(cv::Scalar(0));
                sink.GrabFrame(bgr);
                std::vector<std::vector<cv::Point>> allContours;
                std::vector<cv::Vec4i> hierarchy;
                if (!bgr.empty()) {
//                    cv::resize(raw_bgr, bgr, cv::Size(IMAGE_WIDTH_PX, IMAGE_HEIGHT_PX));
                    // To gray scale
                    cv::cvtColor(bgr, hsv, CV_BGR2HSV);
                    // Blur
                    hsv.copyTo(hsvBlur);
//                    cv::medianBlur(hsv, hsvBlur, 3);
                    // Morphology dilate and erode
                    cv::morphologyEx(hsvBlur, hsvMorph, cv::MORPH_CLOSE, cv::Mat::ones(cv::Size(3, 3), CV_8UC1));
                    cv::Scalar
                            lowerGreen(vision::networkTable->GetNumber("Lower Hue", k_VisionConfig.at("Lower Hue").get<double>()),
                                       vision::networkTable->GetNumber("Lower Saturation", k_VisionConfig.at("Lower Saturation").get<double>()),
                                       vision::networkTable->GetNumber("Lower Value", k_VisionConfig.at("Lower Value").get<double>())),
                            upperGreen(vision::networkTable->GetNumber("Upper Hue", k_VisionConfig.at("Upper Hue").get<double>()),
                                       vision::networkTable->GetNumber("Upper Saturation", k_VisionConfig.at("Upper Saturation").get<double>()),
                                       vision::networkTable->GetNumber("Upper Value", k_VisionConfig.at("Upper Value").get<double>()));
                    // Masking, remove all except for tape
                    cv::inRange(hsvMorph, lowerGreen, upperGreen, mask);
                    // Find edges
                    mask.copyTo(edged);
//                    cv::Canny(mask, edged, 30, 30*3);
                    // Find contours
                    cv::findContours(edged, allContours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
                    const int contoursDetected = static_cast<const int>(allContours.size());
//                    cv::putText(output, std::to_string(contoursDetected), cv::Point(10, 160), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 255), 2,
//                                CV_AA);
                    if (contoursDetected >= VISION_TARGET_COUNT) {
                        // Sort by contour area
                        std::partial_sort(allContours.begin(), allContours.begin() + VISION_TARGET_COUNT, allContours.end(),
                                          [](const std::vector<cv::Point>& p1, const std::vector<cv::Point>& p2) -> bool {
                                              return cv::contourArea(p1) > cv::contourArea(p2);
                                          });
                        // Determine which is left tape and which is right tape
                        auto& firstLargestContour = allContours[0], secondLargestContour = allContours[1];
                        const bool leftFirst = firstLargestContour[0].x < secondLargestContour[0].x;
                        auto&
                                leftContour = leftFirst ? firstLargestContour : secondLargestContour,
                                rightContour = leftFirst ? secondLargestContour : firstLargestContour;
                        std::vector<std::vector<cv::Point>> contours{leftContour, rightContour};
                        // Approximate rectangles for tape
                        const double
                                leftEpsilon = cv::arcLength(leftContour, true) * vision::networkTable->GetNumber("Arc Constant", 0.005),
                                rightEpsilon = cv::arcLength(rightContour, true) * vision::networkTable->GetNumber("Arc Constant", 0.005);
                        std::vector<cv::Point> leftTarget, rightTarget;
                        cv::approxPolyDP(leftContour, leftTarget, leftEpsilon, true);
                        cv::approxPolyDP(rightContour, rightTarget, rightEpsilon, true);
                        cv::putText(output, std::to_string(leftTarget.size()), cv::Size(150, 70), CV_FONT_HERSHEY_SIMPLEX, 0.5,
                                    cv::Scalar(0, 255, 255), 1, CV_AA);
                        cv::putText(output, std::to_string(rightTarget.size()), cv::Size(150, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5,
                                    cv::Scalar(0, 255, 255), 1, CV_AA);
                        if (leftTarget.size() == 4 && rightTarget.size() == 4) {
                            // Show only detected parts from original image
                            cv::bitwise_and(bgr, bgr, output, mask);
                            std::vector<std::vector<cv::Point>> targets{leftTarget, rightTarget};
                            cv::drawContours(output, targets, -1, cv::Scalar(255, 0, 255), 2);
                            for (int i = 0; i < VISION_TARGET_COUNT; i++) {
                                for (int j = 0; j < 4; j++) {
                                    cv::putText(output, std::to_string(i * 4 + j), targets[i][j], CV_FONT_HERSHEY_SIMPLEX, 0.5,
                                                cv::Scalar(0, 255, 255), 1, CV_AA);
                                }
                            }
                            cv::drawContours(output, contours, -1, cv::Scalar(0, 255, 255), 1);
//                            std::vector<cv::Point> allTargets{4};
//                            allTargets.insert(leftTarget.begin(), leftTarget.end(), allTargets.end());
//                            allTargets.insert(rightTarget.begin(), rightTarget.end(), allTargets.end());
                            cv::Mat translationVectors(3, 1, cv::DataType<double>::type), rotationVectors(3, 1, cv::DataType<double>::type);
//                            auto& imagePoints = allTargets;
//                            // Calculate pose
                            std::vector<cv::Point2f> imagePoints;
                            for (const auto& point : leftTarget) imagePoints.push_back(point);
                            for (const auto& point : rightTarget) imagePoints.push_back(point);
                            cv::Matx33d cameraMatrix(
                                    FOCAL_LENGTH_MM * (IMAGE_WIDTH_PX / SENSOR_WIDTH_MM), 0.0, 80.0,
                                    0.0, FOCAL_LENGTH_MM * (IMAGE_HEIGHT_PX / SENSOR_HEIGHT_MM), 45.0,
                                    0.0, 0.0, 1.0
                            );
//                            std::array<double, 5> distortionData{0.07364798, 0.7798033, -0.01179736, 0.00429556, -3.34828984};
//                            cv::Mat distortionCoefficients(5, 1, cv::DataType<double>::type, distortionData.data());
                            cv::Mat distortionCoefficients = cv::Mat::zeros(cv::Size(5, 1), cv::DataType<double>::type);
                            cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distortionCoefficients, rotationVectors, translationVectors);
                            cv::Mat rotationMatrix(3, 3, CV_64FC1);
                            cv::Rodrigues(rotationVectors, rotationMatrix);
                            cv::Vec3d axis{0.0, 0.0, -1.0};
                            cv::Mat direction = rotationMatrix * cv::Mat(axis, false);
                            double dirX = direction.at<double>(0);
                            double dirY = direction.at<double>(1);
                            double dirZ = direction.at<double>(2);
                            double len = sqrt(dirX * dirX + dirY * dirY + dirZ * dirZ);
                            dirX /= len;
                            dirY /= len;
                            dirZ /= len;
                            cv::putText(output, std::to_string(dirX * (180.0/M_PI)), cv::Size(20, 70), CV_FONT_HERSHEY_SIMPLEX, 0.5,
                                        cv::Scalar(0, 255, 255), 1, CV_AA);
                            cv::putText(output, std::to_string(-dirY * (180.0/M_PI)), cv::Size(20, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5,
                                        cv::Scalar(0, 255, 255), 1, CV_AA);
                            cv::putText(output, std::to_string(dirZ * (180.0/M_PI)), cv::Size(20, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5,
                                        cv::Scalar(0, 255, 255), 1, CV_AA);
                        }
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
    for (auto& item : vision::k_VisionConfig.items())
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
