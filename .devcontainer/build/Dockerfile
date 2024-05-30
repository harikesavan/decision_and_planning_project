ARG UBUNTU_VERSION=20.04

ARG ARCH=
ARG CUDA=11.6
FROM nvidia/cuda${ARCH:+-$ARCH}:${CUDA}.1-base-ubuntu${UBUNTU_VERSION} as base
# ARCH and CUDA are specified again because the FROM directive resets ARGs
# (but their default value is retained if set previously)
ARG ARCH
ARG CUDA
ARG CUDNN=8.4.0.27-1
ARG CUDNN_MAJOR_VERSION=8
ARG LIB_DIR_PREFIX=x86_64
ARG LIBNVINFER=8.0.0-1
ARG LIBNVINFER_MAJOR_VERSION=8

#User config
ARG USERNAME=docker
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

# Let us install tzdata painlessly
ENV DEBIAN_FRONTEND=noninteractive

# # Needed for string substitution
SHELL ["/bin/bash", "-c"]

# Pick up some TF dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        ffmpeg \
        git \
        git-core \
        g++ \
        vim \
        zip \
        zlib1g-dev \
        cuda-command-line-tools-${CUDA/./-} \
        libcublas-${CUDA/./-} \
        cuda-nvrtc-${CUDA/./-} \
        libcufft-${CUDA/./-} \
        libcurand-${CUDA/./-} \
        libcusolver-${CUDA/./-} \
        libcusparse-${CUDA/./-} \
        curl \
        libcudnn8=${CUDNN}+cuda${CUDA} \
        libfreetype6-dev \
        libhdf5-serial-dev \
        libzmq3-dev \
        libcairo2-dev \
        pkg-config \
        libssl-dev \
        software-properties-common \
        unzip \
        wget \ 
        libuv1-dev \
        libssl-dev \
        libz-dev \
        libboost-all-dev \
        libblas-dev\
        liblapack-dev\
        libxerces-c3.2


# See http://bugs.python.org/issue19846
ENV LANG C.UTF-8

RUN apt update -y && \
    add-apt-repository ppa:deadsnakes/ppa -y && \
    apt install -y python3.7-full

RUN update-alternatives --install /usr/bin/python python /usr/bin/python3.7 1

RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && \
    python3.7 get-pip.py && \
    rm get-pip.py

# Install python dependencies

COPY requirements.txt .
RUN python3.7 -m pip install --upgrade pip \ 
    && python3.7 -m pip install -r requirements.txt

# Create the user
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME}\
    && apt-get update \
    && apt-get install -y sudo \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
RUN mkdir -p /home/docker/app
WORKDIR /home/docker/app
ENV PYTHONPATH "$PYTHONPATH:/app/Project/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg"