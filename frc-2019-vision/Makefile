CXX=arm-raspbian9-linux-gnueabihf-g++

build_streamer: setup streamer

build_calibrator: setup calibrator

install_streamer: build_streamer
	plink -pw 'raspberry' pi@frcvision.local "sudo rm -f runCamera" && pscp -pw 'raspberry' streamer pi@frcvision.local:/home/pi/runCamera && plink -pw 'raspberry' pi@frcvision.local "chmod +xw runCamera && sudo svc -t /service/camera"

install_calibrator: build_calibrator
	plink -pw 'raspberry' pi@frcvision.local "sudo rm -f runCamera" && pscp -pw 'raspberry' calibrator pi@frcvision.local:/home/pi/runCamera && plink -pw 'raspberry' pi@frcvision.local "chmod +xw runCamera && sudo svc -t /service/camera"

clean:
	rm build/*.o streamer calibrator

streamer: build/streamer.o
	${CXX} -pthread -o $@ $< \
	    -Llib \
	    -lwpilibc \
	    -lwpiHal \
	    -lcameraserver \
	    -lcscore \
	    -lntcore \
	    -lwpiutil \
	    -lopencv_ml \
	    -lopencv_objdetect \
	    -lopencv_shape \
	    -lopencv_stitching \
	    -lopencv_superres \
	    -lopencv_videostab \
	    -lopencv_calib3d \
	    -lopencv_features2d \
	    -lopencv_highgui \
	    -lopencv_videoio \
	    -lopencv_imgcodecs \
	    -lopencv_video \
	    -lopencv_photo \
	    -lopencv_imgproc \
	    -lopencv_flann \
	    -lopencv_core

calibrator: build/calibrator.o
	${CXX} -pthread -o $@ $< \
	    -Llib \
	    -lwpilibc \
	    -lwpiHal \
	    -lcameraserver \
	    -lcscore \
	    -lntcore \
	    -lwpiutil \
	    -lopencv_ml \
	    -lopencv_objdetect \
	    -lopencv_shape \
	    -lopencv_stitching \
	    -lopencv_superres \
	    -lopencv_videostab \
	    -lopencv_calib3d \
	    -lopencv_features2d \
	    -lopencv_highgui \
	    -lopencv_videoio \
	    -lopencv_imgcodecs \
	    -lopencv_video \
	    -lopencv_photo \
	    -lopencv_imgproc \
	    -lopencv_flann \
	    -lopencv_core

build/calibrator.o: src/calibrator/calibrator.cpp
	${CXX} -pthread -O -c -o $@ -Iinclude $<

build/streamer.o: src/streamer/streamer.cpp
	${CXX} -pthread -O -c -o $@ -Iinclude $<

setup:
	mkdir -p build
