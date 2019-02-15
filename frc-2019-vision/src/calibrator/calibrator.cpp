#include <array>
#include <thread>
#include <vector>
#include <string>
#include <algorithm>

#include <cameraserver/CameraServer.h>
#include <networktables/NetworkTableInstance.h>

#include <wpi/json.h>
#include <wpi/raw_istream.h>
#include <wpi/raw_ostream.h>

#include <opencv2/opencv.hpp>

namespace vision {
    namespace calibration {

        std::shared_ptr<nt::NetworkTable> networkTable;

        int start() {
            auto networkTableInstance = nt::NetworkTableInstance::GetDefault();
            networkTableInstance.StartServer();
            networkTable = networkTableInstance.GetTable("Garage Robotics Vision");
//            networkTable->GetEntry("Take Photo").CreateRpc([](const nt::RpcAnswer& answer) {
//
//            });
            std::error_code errorCode;
            wpi::raw_fd_istream configFile("/boot/frc.json", errorCode);
            wpi::json config = wpi::json::parse(configFile);
            auto cameraConfig = config.at("cameras")[0], streamConfig = cameraConfig.at("stream");
            auto cameraName = cameraConfig.at("name").get<std::string>(), cameraPath = cameraConfig.at("path").get<std::string>();
            auto cameraServer = frc::CameraServer::GetInstance();
            cs::UsbCamera camera{cameraName , cameraPath};
            auto capture = cameraServer->StartAutomaticCapture();
            camera.SetConfigJson(cameraConfig);
            camera.SetConnectionStrategy(cs::VideoSource::kConnectionKeepOpen);
            capture.SetConfigJson(streamConfig);
            while (capture.IsEnabled()) {

            }
        }
    }
}

int main(int argc, char* argv[]) {
    vision::calibration::start();
}
