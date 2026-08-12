FROM nvidia/cuda:12.4.1-cudnn-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /app

# PX4 SITL build tools and runtime libraries for the PyQt/GStreamer application.
RUN apt-get update && apt-get install -y --no-install-recommends \
    astyle \
    bc \
    build-essential \
    ca-certificates \
    ccache \
    cmake \
    cppcheck \
    curl \
    file \
    g++ \
    gcc \
    gdb \
    git \
    gnupg \
    lcov \
    libasound2 \
    libdbus-1-3 \
    libfontconfig1 \
    libegl1 \
    libgl1 \
    libgles2 \
    libglib2.0-0 \
    libglvnd0 \
    libgstreamer-plugins-base1.0-dev \
    libglx0 \
    libnss3 \
    libssl-dev \
    libx11-xcb1 \
    libxcb-cursor0 \
    libxcb-glx0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-shape0 \
    libxcb-xinerama0 \
    libxcb-xkb1 \
    libxcomposite1 \
    libxdamage1 \
    libxkbcommon-x11-0 \
    libxrandr2 \
    libxml2-dev \
    libxml2-utils \
    libxtst6 \
    lsb-release \
    make \
    mesa-utils \
    ninja-build \
    pkg-config \
    python3 \
    python3-dev \
    python3-pip \
    python3-venv \
    libqt5svg5 \
    rsync \
    shellcheck \
    unzip \
    wget \
    zip \
    gstreamer1.0-libav \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    && rm -rf /var/lib/apt/lists/*

# PX4 1.16+ uses Gazebo Harmonic (gz-sim8), distributed by OSRF for Jammy.
RUN wget -q https://packages.osrfoundation.org/gazebo.gpg \
        -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends gz-harmonic libunwind-dev \
    && rm -rf /var/lib/apt/lists/*

# PX4 Gazebo plugins use the OpenCV C++ SDK at configure and link time.
RUN apt-get update \
    && apt-get install -y --no-install-recommends libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

ARG PYTORCH_INDEX_URL=https://download.pytorch.org/whl/cu124

ENV VIRTUAL_ENV=/opt/venv \
    PATH=/opt/venv/bin:${PATH} \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,video,display \
    QT_XCB_GL_INTEGRATION=xcb_glx \
    QTWEBENGINE_DISABLE_SANDBOX=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_NO_CACHE_DIR=1

RUN python3 -m venv "${VIRTUAL_ENV}" \
    && python -m pip install --upgrade pip wheel

# Copy requirement manifests first so source edits do not invalidate this layer.
COPY docker/requirements.txt /tmp/app-requirements.txt
COPY docker/requirements-px4.txt /tmp/px4-requirements.txt

RUN python -m pip install -r /tmp/px4-requirements.txt \
    && python -m pip install \
        --index-url "${PYTORCH_INDEX_URL}" \
        torch torchvision torchaudio \
    && python -m pip install -r /tmp/app-requirements.txt

COPY . .

CMD ["sleep", "infinity"]
