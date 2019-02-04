CXX=arm-raspbian9-linux-gnueabihf-g++

.PHONY: clean build install

build: setup program

install: build
	plink -pw 'raspberry' pi@frcvision.local "sudo rm -f runCamera" && pscp -pw 'raspberry' program pi@frcvision.local:/home/pi/runCamera && plink -pw 'raspberry' pi@frcvision.local "chmod +x runCamera && sudo svc -t /service/camera"
#	cp program runCamera /home/pi/

clean:
	rm build/*.o program

program: build/program.o
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

build/%.o: src/%.cpp
	${CXX} -pthread -O -c -o $@ -Iinclude $<

setup:
	mkdir -p build
