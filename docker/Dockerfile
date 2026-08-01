FROM px4io/px4-dev-simulation-jammy:2024-05-18

ENV DEBIAN_FRONTEND=noninteractive
ENV QT_XCB_GL_INTEGRATION=none
ENV PATH=/opt/conda/bin:${PATH}

WORKDIR /app

RUN apt-get update && apt-get install -y \
    wget \
    git \
    python3 \
    python3-pip \
    libgl1 \
    libglib2.0-0 \
    libxkbcommon-x11-0 \
    libxcb-xinerama0 \
    libxcb-cursor0 \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh -O /tmp/miniforge.sh \
    && bash /tmp/miniforge.sh -b -p /opt/conda \
    && rm /tmp/miniforge.sh \
    && (conda config --remove channels defaults || true) \
    && conda config --add channels conda-forge \
    && conda config --set channel_priority strict

COPY environment_basic.yml ./

RUN conda env create -f environment_basic.yml \
    && conda clean -afy

COPY . .

RUN echo 'if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; fi' >> /root/.bashrc \
    && echo 'source /opt/conda/etc/profile.d/conda.sh' >> /root/.bashrc \
    && echo 'conda activate uav' >> /root/.bashrc

CMD ["conda", "run", "--no-capture-output", "-n", "uav", "python", "src/app.py"]
