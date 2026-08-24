FROM ros:humble-ros-base

# Cyclone DDS RMW 및 유틸리티 설치
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cyclonedds \
    iproute2 \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# ROS2가 기본 RMW 대신 Cyclone DDS를 쓰도록 지정
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml

COPY cyclonedds.xml /etc/cyclonedds/cyclonedds.xml
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
