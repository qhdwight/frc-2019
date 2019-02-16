#include <array>
#include <thread>
#include <vector>
#include <string>
#include <algorithm>

#include <cameraserver/CameraServer.h>
#include <networktables/NetworkTableInstance.h>

#include <frc/shuffleboard/Shuffleboard.h>
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
            std::error_code errorCode;
            wpi::raw_fd_istream configFile("/boot/frc.json", errorCode);
            auto config = wpi::json::parse(configFile);
            auto cameraConfig = config.at("cameras").front(), streamConfig = cameraConfig.at("stream");
            auto cameraName = cameraConfig.at("name").get<std::string>(), cameraPath = cameraConfig.at("path").get<std::string>();
            auto cameraServer = frc::CameraServer::GetInstance();
            cs::UsbCamera camera{cameraName, cameraPath};
            auto capture = cameraServer->StartAutomaticCapture(camera);
            camera.SetConfigJson(cameraConfig);
            camera.SetConnectionStrategy(cs::VideoSource::kConnectionKeepOpen);
            capture.SetConfigJson(streamConfig);
            auto sink = cameraServer->GetVideo();
            auto stream = cameraServer->PutVideo("Calibration", 320, 180);
            stream.SetConnectionStrategy(cs::VideoSource::kConnectionKeepOpen);
            bool isDone = false;
            networkTable->GetEntry("Take Photo").CreateRpc([&](const nt::RpcAnswer& answer) {
                cameraServer->RemoveCamera(cameraName);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                system("raspistill -w 1600 -h 900 -q 100 -o calibration/image.png");
                isDone = true;
            });
            while (!isDone) {}
            return EXIT_SUCCESS;
        }
    }
}

int main(int argc, char* argv[]) {
    return vision::calibration::start();
}
