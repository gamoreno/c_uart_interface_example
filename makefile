CXXFLAGS=-std=c++11 -O3 -Wall -fmessage-length=0

OBJS=mavlink_control.o utils.o Port.o UDPPort.o serial_port.o autopilot_interface.o pipe_control.o stdio_control.o
LIBS=-lpthread
TARGET=mavlink_control
CPPFLAGS=-Imavlink/include/mavlink/v1.0

all: mavlink_control


$(TARGET): git_submodule $(OBJS)
	$(CXX) -o $(TARGET) $(OBJS) $(LIBS)

git_submodule:
	git submodule update --init --recursive

clean:
	 rm -f $(OBJS) $(TARGET)
