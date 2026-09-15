FROM kalibr-ros1-noetic:latest
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
USER root
RUN sed -i 's|http://archive.ubuntu.com|https://archive.ubuntu.com|g; s|http://security.ubuntu.com|https://security.ubuntu.com|g' /etc/apt/sources.list \
    && apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends libceres-dev python3-matplotlib python3-numpy python3-yaml git \
    && rm -rf /var/lib/apt/lists/*
ARG IMU_UTILS_REV=f8a770cc81f8c86e93ee3c94518d2456dc722ad3
RUN mkdir -p /imu_ws/src && git clone https://github.com/gaowenliang/imu_utils.git /imu_ws/src/imu_utils \
    && cd /imu_ws/src/imu_utils && git checkout "$IMU_UTILS_REV"
COPY patch_sample_limit.py /tmp/patch_sample_limit.py
RUN python3 /tmp/patch_sample_limit.py \
    && source /opt/ros/noetic/setup.bash && cd /imu_ws \
    && catkin_make --pkg imu_utils -DCMAKE_BUILD_TYPE=Release -j2 \
    && git -C src/imu_utils rev-parse HEAD > /imu_ws/upstream_revision.txt \
    && git -C src/imu_utils diff > /imu_ws/local.patch
COPY install_allan_variance_ros.sh /tmp/install_allan_variance_ros.sh
RUN bash /tmp/install_allan_variance_ros.sh && rm -rf /var/lib/apt/lists/*
WORKDIR /imu_ws
CMD ["sleep", "infinity"]
