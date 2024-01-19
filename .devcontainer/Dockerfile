# Specify the parent image from which we build
FROM ubuntu:22.04

# OpenCV Version 
ARG OpenCV_VERSION
ARG Open3D_VERSION
ARG Ceres_VERSION

# Install dependencies for OpenCV, Open3D, Ceres
RUN apt-get update || true && apt-get upgrade -y &&\
    # Install build tools, build dependencies and python
    apt-get install --no-install-recommends -y \
	build-essential gcc g++ wget \
	cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev \
	libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev \
    yasm libatlas-base-dev gfortran libpq-dev \
    libxine2-dev libglew-dev libtiff5-dev zlib1g-dev libavutil-dev libpostproc-dev \ 
    libeigen3-dev python3-dev python3-pip python3-numpy libx11-dev tzdata \
	libgoogle-glog-dev libgflags-dev libbenchmark-dev libgtest-dev \
	libatlas-base-dev libsuitesparse-dev \
&& rm -rf /var/lib/apt/lists/*

# Set Working directory
WORKDIR /opt


# Install OpenCV from Source
RUN git clone --depth 1 --branch ${OpenCV_VERSION} https://github.com/opencv/opencv.git && \
    git clone --depth 1 --branch ${OpenCV_VERSION} https://github.com/opencv/opencv_contrib.git && \
    cd opencv && \
    mkdir build && \
    cd build && \
    cmake \
	-D CMAKE_BUILD_TYPE=RELEASE \
	-D CMAKE_INSTALL_PREFIX=/usr/ \
	-D PYTHON3_PACKAGES_PATH=/usr/lib/python3/dist-packages \
	-D WITH_V4L=ON \
	-D WITH_QT=OFF \
	-D WITH_OPENGL=ON \
	-D WITH_GSTREAMER=ON \
	-D OPENCV_GENERATE_PKGCONFIG=ON \
	-D OPENCV_ENABLE_NONFREE=ON \
	-D OPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib/modules \
	-D INSTALL_PYTHON_EXAMPLES=OFF \
	-D INSTALL_C_EXAMPLES=OFF \
	-D BUILD_EXAMPLES=OFF .. && \
    make -j"$(nproc)" && \
    make install && cd ../../ \
    rm -rf opencv opencv_contrib # remove folders

# mkdir workspace and start workspace
RUN mkdir workspace

WORKDIR /workspace

CMD ["bash"]