# F1TENTH ROS 2 Humble image for NVIDIA Jetson (JetPack 6 / L4T r36.3, arm64)
#
# Reconstructed from `docker history` of the original `misys:f1tenth` image.
# Build ONLY on an arm64 Jetson board -- the base image is Tegra-specific and
# several steps compile against CUDA (range_libc, librealsense).
#
#   docker build -t misys:f1tenth .
#
FROM nvcr.io/nvidia/l4t-jetpack:r36.3.0

SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive

# --- timezone -----------------------------------------------------------------
RUN echo 'Asia/Seoul' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Asia/Seoul /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# --- locale -------------------------------------------------------------------
RUN apt update && apt install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

# --- ROS 2 apt repository -----------------------------------------------------
RUN apt install -y software-properties-common && \
    add-apt-repository -y universe && \
    apt update && apt install -y curl && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
         -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    tee /etc/apt/sources.list.d/ros2.list > /dev/null

# --- ROS 2 Humble desktop -----------------------------------------------------
RUN apt update && apt upgrade -y && \
    apt install -y \
      ros-humble-desktop \
      ros-dev-tools \
      sudo vim git \
      python3-pip

RUN rosdep init

# --- Python 3.8 (needed by the Raceline-Optimization venv) --------------------
RUN apt update && \
    apt install -y software-properties-common && \
    add-apt-repository -y ppa:deadsnakes/ppa && \
    apt update && \
    apt install -y python3.8 python3.8-venv python3.8-dev && \
    rm -rf /var/lib/apt/lists/*

# --- non-root user ------------------------------------------------------------
ARG UID=1000
ARG UNAME=misys
ARG PW=root

RUN useradd -u $UID -m -s /bin/bash $UNAME && \
    echo $UNAME:$PW | chpasswd && \
    usermod -aG sudo $UNAME && \
    echo "$UNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER $UNAME
WORKDIR /home/$UNAME

RUN sudo apt-get update && rosdep update && mkdir -p f1tenth_ws/src

# Cache buster: bump this ARG to force everything below to rebuild.
ARG CACHEBUST=__update__
RUN echo "$CACHEBUST"

# --- workspace sources --------------------------------------------------------
RUN cd f1tenth_ws/src && \
    git clone https://github.com/ros-drivers/transport_drivers.git && \
    git clone https://github.com/flynneva/udp_msgs.git && \
    git clone https://github.com/f1tenth/ackermann_mux.git && \
    git clone https://github.com/ros-teleop/teleop_tools.git && \
    git clone https://github.com/f1tenth/vesc.git -b ros2

RUN cd f1tenth_ws/src && \
    git clone --recursive https://github.com/Hokuyo-aut/urg_node2.git && \
    cd urg_node2 && git submodule update --remote

# --- joystick tooling ---------------------------------------------------------
RUN sudo apt update && sudo apt install -y jstest-gtk
RUN mkdir -p .config/jstest-gtk

# --- librealsense (built with CUDA, installed into the ROS prefix) ------------
RUN sudo apt install -y libssl-dev xorg-dev libusb-1.0-0-dev usbutils \
      libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

RUN mkdir temp_ws && cd temp_ws && \
    git clone https://github.com/IntelRealSense/librealsense.git && \
    cd librealsense && mkdir build && cd build && \
    cmake ../ -DFORCE_RSUSB_BACKEND=ON -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=/usr/bin/python3 \
      -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true -DBUILD_WITH_CUDA:bool=true \
      -DCMAKE_INSTALL_PREFIX=/opt/ros/humble && \
    make -j6 && sudo make install && \
    sudo cp ../config/99-realsense-libusb.rules /etc/udev/rules.d/ && \
    cd ~/ && rm -rf temp_ws

# --- f1tenth_system -----------------------------------------------------------
RUN cd f1tenth_ws/src && \
    git clone https://github.com/goodash2one/f1tenth_system.git -b humble-devel && \
    cd f1tenth_system && rmdir ackermann_mux teleop_tools vesc

RUN sed -i "102s/-state/state/" f1tenth_ws/src/vesc/vesc_ackermann/src/vesc_to_odom.cpp
RUN sed -i "184s/RELIABLE/BEST_EFFORT/" f1tenth_ws/src/teleop_tools/joy_teleop/joy_teleop/joy_teleop.py

# --- gym, raceline optimization, sim, slam, particle filter -------------------
RUN git clone https://github.com/f1tenth/f1tenth_gym.git && \
    git clone https://github.com/goodash2one/Raceline-Optimization.git && \
    cd f1tenth_ws/src && \
    git clone https://github.com/goodash2one/f1tenth_gym_ros.git && \
    git clone https://github.com/goodash2one/slam_toolbox.git && \
    git clone https://github.com/goodash2one/particle_filter.git && \
    git clone https://github.com/goodash2one/bringup.git && \
    git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master

# Sparse checkout of pure_pursuit + safety_node from CL2-UWaterloo
RUN cd f1tenth_ws/src && git init && \
    git config core.sparseCheckout true && \
    git remote add origin https://github.com/goodash2one/CL2-UWaterloo.git && \
    echo "src/pure_pursuit" >> .git/info/sparse-checkout && \
    echo "src/safety_node" >> .git/info/sparse-checkout && \
    git pull origin main && rm -rf .git/ && mv src CL2-UWaterloo && \
    cd CL2-UWaterloo && git init && \
    git config user.name $UNAME && git config user.email "you@example.com" && \
    git add . && git commit -m "clone packages from CL2-UWaterloo/f1tenth_ws/src"

# Sparse checkout of the mpc package from derekhanbaliq/f1tenth-software-stack
RUN cd f1tenth_ws/src && git init && \
    git config core.sparseCheckout true && \
    git remote add origin https://github.com/derekhanbaliq/f1tenth-software-stack.git && \
    echo "mpc" >> .git/info/sparse-checkout && \
    git pull origin main && rm -rf .git/ && \
    cd mpc && rm -f .DS_Store && mkdir racelines && git init && \
    git config user.name $UNAME && git config user.email "you@example.com" && \
    git add . && git commit -m "clone packages from derekhanbaliq/f1tenth-software-stack"

RUN cd f1tenth_gym && pip3 install -e .

# --- simulator / SLAM config patches -----------------------------------------
RUN sed -i "45s/user/$UNAME/" f1tenth_ws/src/f1tenth_gym_ros/config/sim.yaml
RUN sed -i "30s/\/drive/drive/; \
            35s/\/opp_drive/opp_drive/" \
            f1tenth_ws/src/f1tenth_gym_ros/config/sim.yaml

RUN cd f1tenth_ws/src/slam_toolbox/config/ && \
    cp mapper_params_offline.yaml mapper_params_offline_sim.yaml && \
    sed -i "13s/odom_frame: odom/odom_frame: sim/; 14s/map_frame: map/map_frame: slam_map/; 15s/base_link/ego_racecar\/base_link/; 16s/\/scan/\/sim\/scan/" mapper_params_offline_sim.yaml

RUN cp f1tenth_ws/src/slam_toolbox/launch/offline_launch.py f1tenth_ws/src/slam_toolbox/launch/offline_launch_sim.py && \
    sed -i "11s/mapper_params_offline/mapper_params_offline_sim/" \
           f1tenth_ws/src/slam_toolbox/launch/offline_launch_sim.py

# --- particle filter with CUDA-accelerated range_libc -------------------------
RUN sudo pip install cython && \
    cd f1tenth_ws/src/particle_filter && \
    sed -i "14s/cddt/rmgpu/" config/localize.yaml && \
    cd range_libc/pywrapper && ./compile_with_cuda.sh

# --- Raceline-Optimization (python3.8 venv) -----------------------------------
RUN sed -i "1s/1.18.1/1.19.5/; \
            5s/3.5.1/3.5.5/; \
            6s/1.3.3/1.5.4/; \
            7s/0.23.1/0.24.2/" \
        Raceline-Optimization/requirements.txt

RUN sed -i "17s/0.1/0.15/" Raceline-Optimization/params/f110.ini

RUN cd Raceline-Optimization && \
    python3.8 -m venv raceline && \
    source raceline/bin/activate && \
    pip install -r requirements.txt && \
    pip install ipykernel notebook pyqt6 && \
    python -m ipykernel install --user --name raceline && \
    deactivate

# --- pure_pursuit / mpc tuning ------------------------------------------------
RUN sed -i "s/user/$UNAME/g" f1tenth_ws/src/CL2-UWaterloo/pure_pursuit/config/sim_config.yaml
RUN sed -i "s/user/$UNAME/g" f1tenth_ws/src/CL2-UWaterloo/pure_pursuit/config/config.yaml

RUN sed -i "83s/levine_2nd/levine_full/; \
            86s/if/# if/; \
            87s/\"\/drive\"/\"\/auto\"/; \
            92s/if/if not/; \
            105s/csv_data/mpc/; \
            106s/'\/'/'\/racelines\/'/; \
            166s/if/if not/; \
            176s/if/if not/; \
            177s/if/if not/; \
            180s/if/if not/; \
            20s/.*/from rclpy.qos import QoSProfile/; \
            95s/1/QoSProfile(depth=1, reliability=2)/; \
            39s/\[13.5, 13.5, 5.5, 13.0\]/\[50.0, 50.0, 10.0, 0.0\]/; \
            43s/\[13.5, 13.5, 5.5, 13.0\]/\[50.0, 50.0, 10.0, 0.0\]/" \
            f1tenth_ws/src/mpc/scripts/mpc_node.py

RUN sed -i "26s/8/6/; \
            39s/\[50.0, 50.0, 10.0, 0.0\]/\[100.0, 100.0, 0.0, 0.0\]/; \
            43s/\[50.0, 50.0, 10.0, 0.0\]/\[100.0, 100.0, 0.0, 0.0\]/; \
            87s/\/auto/\/sim\/drive/; \
            95s/reliability=2/reliability=1/" \
        f1tenth_ws/src/mpc/scripts/mpc_node.py

RUN sed -i "156s/ld/# ld/" \
        f1tenth_ws/src/f1tenth_system/f1tenth_stack/launch/bringup_launch.py

RUN sed -i "56s/True/False/; \
            57s/True/False/; \
            59s/True/False/; \
            60s/True/False/" \
           Raceline-Optimization/main_globaltraj_f110.py

# --- resolve dependencies and build the workspace -----------------------------
RUN sudo apt update && cd f1tenth_ws && \
    sudo pip install meson ninja && \
    rosdep install -i --from-path src --ignore-src --rosdistro humble --skip-keys=librealsense2 -r -y

RUN source /opt/ros/humble/setup.bash && \
    cd f1tenth_ws && colcon build

# --- shell conveniences -------------------------------------------------------
RUN echo "PATH=\$PATH:/home/$UNAME/.local/bin" >> .bashrc
RUN echo "alias underlay=\"source /opt/ros/humble/setup.bash\"" >> .bashrc && \
    echo "alias overlay=\"source install/setup.bash\"" >> .bashrc
RUN echo "export SDL_JOYSTICK_DEVICE=\"/dev/input/js0\"" >> .bashrc

WORKDIR /home/$UNAME
CMD ["bash"]
