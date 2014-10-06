SOURCE = src/
BUILD = build/
HEADERS = CMesh.h CPlyHelper.h
OBJECTS = CMesh.o CPlyHelper.o sliceIntersection.o
LIBS = -lpng -lopencv_core -lopencv_highgui -lopencv_imgproc
INCLUDE = -I /usr/include/pcl-1.7 -I /usr/include/eigen3
CXXFLAGS = -Wall

%.o: $(SOURCE)%.cpp
	g++ $(CXXFLAGS) $(INCLUDE) -c $^ -o $@

sliceIntersection: $(OBJECTS)
	g++ $(CXXFLAGS) $^ $(LIBS) -o sliceIntersection

pathGen: pathGen.o
	g++ $(CXXFLAGS) $^ $(LIBS) -o pathGen

clean:
	rm *.o
