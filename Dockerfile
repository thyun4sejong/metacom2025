ARG DEBIAN_FRONTEND=noninteractive
ARG ISAACSIM_VERSION=4.2.0

# NVIDIA Isaac Sim 기반 이미지 사용
# https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim
FROM nvcr.io/nvidia/isaac-sim:${ISAACSIM_VERSION} AS isaac-sim

# OMNI_SERVER 
# ENV OMNI_SERVER=http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.2.0
# ENV OMNI_SERVER=omniverse://localhost/NVIDIA/Assets/Isaac/4.2.0
# ENV OMNI_USER=admin
# ENV OMNI_PASS=admin
# ENV MIN_DRIVER_VERSION=525.60.11

# 작업 디렉터리 설정
WORKDIR /isaac-sim

# 필요한 패키지 설치
RUN apt-get update && apt-get install -y \
    python3-pip \
    x11-xserver-utils \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# pip 최신 버전으로 업그레이드
RUN /isaac-sim/kit/python/bin/python3 -m pip install --upgrade pip

# PYTHONPATH 설정
ENV PYTHONPATH=/isaac-sim/kit/python/lib/python3.10/site-packages:/isaac-sim/exts

# USD Plugin Path 설정
ENV USD_PLUGIN_PATH=/isaac-sim/exts/omni.usd.schema.isaac/plugins

# Add symlink
RUN ln -s exts/omni.isaac.examples/omni/isaac/examples extension_examples

# Default entrypoint to launch headless with streaming
ENTRYPOINT ["/isaac-sim/runapp.sh"]