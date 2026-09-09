FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-colcon-common-extensions \
        ros-humble-cv-bridge \
        ros-humble-sensor-msgs \
        ros-humble-rosidl-default-generators \
        git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/lang_sam_ws/src
COPY lang_sam_msgs ./lang_sam_msgs
COPY lang_sam_detector ./lang_sam_detector

WORKDIR /opt/lang_sam_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --packages-select lang_sam_msgs lang_sam_detector

RUN pip3 install --no-cache-dir \
        torch \
        torchvision \
        transformers \
        hydra-core \
        omegaconf \
        opencv-python-headless \
        pillow \
        "numpy<2" \
        supervision \
        huggingface_hub \
        git+https://github.com/facebookresearch/sam2.git

COPY docker/server_profile.xml /opt/lang_sam_ws/docker/server_profile.xml
COPY docker/entrypoint.sh /opt/lang_sam_ws/docker/entrypoint.sh
RUN chmod +x /opt/lang_sam_ws/docker/entrypoint.sh
