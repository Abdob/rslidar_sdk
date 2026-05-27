# Build libnvds_rs_lidar_dataloader.so (Ds3D dataloader wrapping rs_driver)
# Modeled on /opt/nvidia/deepstream/deepstream/sources/libs/ds3d/dataloader/lidarsource/Makefile

CUDA_VER  ?=
CUDA_HOME := /usr/local/cuda
ifneq ($(CUDA_VER),)
  CUDA_HOME := /usr/local/cuda-$(CUDA_VER)
endif

DS_PATH    := /opt/nvidia/deepstream/deepstream
NVDS_VER   ?= $(shell basename $(realpath $(DS_PATH)) | cut -f2 -d -)
DS_HOME    := /opt/nvidia/deepstream/deepstream-$(NVDS_VER)
LIB_INSTALL := $(DS_HOME)/lib

RS_DRIVER_DIR ?= /opt/rs_driver

TARGET := libnvds_rs_lidar_dataloader.so

CXX := g++

# rs_driver vendored headers reference std::shared_ptr without including <memory>
# under newer libstdc++; force-include the standard headers.
CXXFLAGS := -fPIC -std=c++17 -Wall -fvisibility=hidden \
            -include memory -include functional \
            -I $(DS_HOME)/sources/includes \
            -I $(DS_HOME)/sources/libs \
            -I $(CUDA_HOME)/include \
            -I $(RS_DRIVER_DIR)/src

CXXFLAGS += $(shell pkg-config --cflags yaml-cpp)

LIBS := $(shell pkg-config --libs yaml-cpp)
LIBS += -L$(CUDA_HOME)/lib64 -lcudart -lcuda
LIBS += -L$(LIB_INSTALL) -lnvds_3d_common -Wl,-rpath,$(LIB_INSTALL)
LIBS += -lpthread -lpcap -lm

SRCS := $(wildcard *.cpp)
OBJS := $(SRCS:.cpp=.o)
HDRS := $(wildcard *.h)

LFLAGS := -shared -Wl,--start-group $(LIBS) -Wl,--end-group

all: $(TARGET)

$(TARGET): $(OBJS) Makefile
	$(CXX) -o $@ $(OBJS) $(LFLAGS)

%.o: %.cpp $(HDRS) Makefile
	$(CXX) -c -o $@ $(CXXFLAGS) $<

install: $(TARGET)
	cp -v $(TARGET) $(LIB_INSTALL)/

clean:
	rm -f $(OBJS) $(TARGET)
