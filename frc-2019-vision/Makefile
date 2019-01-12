CXX=./raspbian-tools/bin/arm-raspbian9-linux-gnueabihf-g++
EXE=program
DESTDIR?=./build/

.PHONY: clean build install

build: ${EXE}

install: build
	cp ${EXE} runCamera ${DESTDIR}

clean:
	rm ${EXE} *.o

${EXE}: program.o
	${CXX} -pthread -o $@ $< \
	    -L./raspbian-frc/lib \
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

.cpp.o:
	${CXX} -pthread -O -c -o $@ -I./raspbian-frc/include $<
