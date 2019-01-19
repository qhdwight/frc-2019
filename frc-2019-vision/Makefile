CXX=arm-raspbian9-linux-gnueabihf-g++

.PHONY: clean build install

build: program

install: build
	cp program runCamera /home/pi/

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

build/program.o: src/program.cpp
	${CXX} -pthread -O -c -o $@ -Iinclude $<
