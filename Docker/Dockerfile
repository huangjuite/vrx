FROM osrf/vrx:nvidia_current

USER root
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get -o Acquire::ForceIPv4=true update && apt-get -yq dist-upgrade \
    && apt-get -o Acquire::ForceIPv4=true install -yq --no-install-recommends \
    locales \
    cmake \
    git \
    build-essential \
    python-pip \
    python3-pip \
    python3-setuptools \
    wget \
    sudo \
    curl \
    lsb-release \
    vim \
    net-tools \
    dirmngr \
    gnupg2 \
    mercurial \
    libignition-math4-dev \
    libglib2.0-dev \
    libcwiid-dev \
    libusb-dev \
    libpcap-dev \
    python-gobject \
    python-gtk2 \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    python3-scipy \
    libmove-base-msgs-dev \
    libpcap-dev \    
    gedit \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*


##################################### PIP ######################################

RUN pip2 install --upgrade pip setuptools

RUN pip2 install  \
    tensorflow-gpu==1.14.0 \
    numpy==1.16 \
    gast==0.2.2 \
    matplotlib \
    pandas \
    pypozyx \
    requests \
    torch \
    torchvision \
    scikit-image \
    scikit-learn


# install packages
RUN apt-get -o Acquire::ForceIPv4=true update && apt-get -o Acquire::ForceIPv4=true install -q -y \
    ros-melodic-perception \
    ros-melodic-rgbd-launch \
    ros-melodic-joystick-drivers \
    ros-melodic-pointcloud-to-laserscan \
    ros-melodic-robot-localization \
    ros-melodic-cartographer-ros \
    ros-melodic-cartographer-rviz \
    ros-melodic-spacenav-node \
    ros-melodic-tf2-sensor-msgs \
    ros-melodic-geodesy \
    ros-melodic-serial \
    ros-melodic-controller-manager \
    ros-melodic-openslam-gmapping \
    ros-melodic-twist-mux \
    ros-melodic-velodyne-simulator \
    ros-melodic-gazebo-ros \
    ros-melodic-lms1xx \
    ros-melodic-rviz-imu-plugin \
    ros-melodic-interactive-marker-twist-server \
    ros-melodic-teleop-twist-joy \
    ros-melodic-ddynamic-reconfigure \
    ros-melodic-rosserial-arduino \
    ros-melodic-rosserial \
    && rm -rf /var/lib/apt/lists/*

####################################### procman ###########################################

RUN cd ${HOME} && git clone https://github.com/lcm-proj/lcm \
    && cd lcm \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make install

RUN cd ${HOME} && git clone http://github.com/huangjuite/procman \
    && cd procman \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make install


###################################### CUDA ####################################

RUN apt-get update && apt-get install -y --no-install-recommends gnupg2 curl ca-certificates && \
    curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub | apt-key add - && \
    echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/cuda.list && \
    echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list && \
    rm -rf /var/lib/apt/lists/*

ENV CUDA_VERSION 10.0.130

ENV CUDA_PKG_VERSION 10-0=$CUDA_VERSION-1
# For libraries in the cuda-compat-* package: https://docs.nvidia.com/cuda/eula/index.html#attachment-a
RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-cudart-$CUDA_PKG_VERSION \
    cuda-compat-10-0=410.48-1 && \
    ln -s cuda-10.0 /usr/local/cuda && \
    rm -rf /var/lib/apt/lists/*

ENV PATH /usr/local/cuda/bin:${PATH}

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility,graphics
ENV NVIDIA_REQUIRE_CUDA "cuda>=10.0 brand=tesla,driver>=384,driver<385"

ENV NCCL_VERSION 2.4.2

RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-libraries-$CUDA_PKG_VERSION \
    cuda-nvtx-$CUDA_PKG_VERSION \
    libnccl2=$NCCL_VERSION-1+cuda10.0 && \
    apt-mark hold libnccl2 && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-libraries-dev-$CUDA_PKG_VERSION \
    cuda-nvml-dev-$CUDA_PKG_VERSION \
    cuda-minimal-build-$CUDA_PKG_VERSION \
    cuda-command-line-tools-$CUDA_PKG_VERSION \
    libnccl-dev=$NCCL_VERSION-1+cuda10.0 && \
    rm -rf /var/lib/apt/lists/*

ENV LIBRARY_PATH=$LIBRARY_PATH:/usr/local/cuda/lib64/stubs

###################################### CUDNN ###################################

ENV CUDNN_VERSION 7.4.2.24

LABEL com.nvidia.cudnn.version="${CUDNN_VERSION}"

RUN apt-get update && apt-get install -y --no-install-recommends \
    libcudnn7=$CUDNN_VERSION-1+cuda10.0 \
    libcudnn7-dev=$CUDNN_VERSION-1+cuda10.0 && \
    apt-mark hold libcudnn7 && \
    rm -rf /var/lib/apt/lists/*


USER developer