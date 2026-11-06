FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-colcon-common-extensions \
        ros-humble-cv-bridge \
        ros-humble-sensor-msgs \
        ros-humble-rosidl-default-generators \
        git \
        wget \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/lang_sam_ws/src
COPY lang_sam_msgs ./lang_sam_msgs
COPY lang_sam_detector ./lang_sam_detector
COPY lang_sam_tracker ./lang_sam_tracker
COPY lang_sam_executor ./lang_sam_executor

WORKDIR /opt/lang_sam_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install \
        --packages-select lang_sam_msgs lang_sam_detector lang_sam_tracker lang_sam_executor

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
        tqdm \
        git+https://github.com/facebookresearch/sam2.git

# Cutie: 推論に使う部分はtorch/omegaconf/hydra/tqdmのみに依存する。
# 依存一式(PySide6, gradio等のGUI・学習用)は不要かつ巨大なので--no-depsで入れる
RUN pip3 install --no-cache-dir --no-deps \
        git+https://github.com/hkchengrex/Cutie.git@ec5cdd4cf16f75c73ad785a2f96fb97dbad4125a

# Cutieの重みはイメージに同梱する(計算ノードからの初回ダウンロードを避ける)
RUN mkdir -p /opt/cutie && \
    wget -q -O /opt/cutie/cutie-base-mega.pth \
        https://github.com/hkchengrex/Cutie/releases/download/v1.0/cutie-base-mega.pth

COPY docker/server_profile.xml /opt/lang_sam_ws/docker/server_profile.xml
COPY docker/entrypoint.sh /opt/lang_sam_ws/docker/entrypoint.sh
RUN chmod +x /opt/lang_sam_ws/docker/entrypoint.sh
