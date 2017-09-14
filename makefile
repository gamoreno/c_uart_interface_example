CXXFLAGS=-std=c++11

all: mavlink_control


mavlink_control: git_submodule mavlink_control.cpp
	g++ $(CXXFLAGS) -I mavlink/include/mavlink/v1.0 mavlink_control.cpp utils.cpp Port.cpp UDPPort.cpp serial_port.cpp autopilot_interface.cpp -o mavlink_control -lpthread

git_submodule:
	git submodule update --init --recursive

clean:
	 rm -rf *o mavlink_control
