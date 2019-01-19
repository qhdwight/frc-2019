#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include <networktables/NetworkTableInstance.h>
#include <vision/VisionPipeline.h>
#include <vision/VisionRunner.h>
#include <wpi/StringRef.h>
#include <wpi/json.h>
#include <wpi/raw_istream.h>
#include <wpi/raw_ostream.h>
#include <opencv2/opencv.hpp>

#include "cameraserver/CameraServer.h"

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


namespace vision {

    static const char* k_ConfigFile = "/boot/frc.json";

    unsigned int teamNumber;
    bool isServer = false;

    struct CameraConfig {
        std::string name, path;
        wpi::json cameraConfig, streamConfig;
    };

    std::vector<CameraConfig> cameraConfigs;

    wpi::raw_ostream& ParseError() {
        return wpi::errs() << "config error in '" << k_ConfigFile << "': ";
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
        wpi::raw_fd_istream configFile(k_ConfigFile, errorCode);
        if (errorCode) {
            wpi::errs() << "could not open '" << k_ConfigFile << "': " << errorCode.message() << '\n';
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
        std::thread([&] {
            cs::CvSink sink = cameraServer->GetVideo();
            cs::CvSource outputStream = cameraServer->PutVideo("Raspberry Pi", 160, 90);
            outputStream.SetFPS(30);
            outputStream.SetConnectionStrategy(cs::VideoSource::kConnectionKeepOpen);
            cv::Mat frame;
            while (true) {
                sink.GrabFrame(frame);
                if (frame.empty()) continue;
                cv::putText(frame, "Meme", cv::Point(10, 160), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 255), 2, CV_AA);
                outputStream.PutFrame(frame);
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
    if (argc >= 2) vision::k_ConfigFile = argv[1];
    if (!vision::ReadConfig()) return EXIT_FAILURE;
    auto networkTable = nt::NetworkTableInstance::GetDefault();
    if (vision::isServer) {
        wpi::outs() << "Setting up NetworkTables server\n";
        networkTable.StartServer();
    } else {
        wpi::outs() << "Setting up NetworkTables client for team " << vision::teamNumber << '\n';
        networkTable.StartClientTeam(vision::teamNumber);
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
