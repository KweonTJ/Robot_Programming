FROM ubuntu:24.04
# change to your name and email
LABEL maintainer="kmk <kmk6061602@cbnu.ac.kr>"
LABEL maintainer="ktj <kweontj0701@naver.com>"

###FILL HERE### (INSTALL ROS2, and something else you needed)
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y locales curl gnupg2 lsb-release software-properties-common sudo && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

# ROS2 Jazzy GPG key 등록
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# ROS2 Jazzy apt repository 추가
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu noble main" \
    > /etc/apt/sources.list.d/ros2.list

# ROS2 Jazzy 설치
RUN apt-get update && \
    apt-get install -y ros-jazzy-desktop python3-argcomplete ros-dev-tools && \
    rm -rf /var/lib/apt/lists/*

###---------###

# Create user as uid 1001
RUN useradd --shell /bin/bash -u 1001 -c "" -m user && usermod -a -G dialout user

# Add user to sudoers
RUN echo user ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/user \
    && chmod 0440 /etc/sudoers.d/user

# Remove ubuntu user (from 24.04 ubuntu image has default user ubuntu, to use entrypoint.sh need to delete it)
RUN userdel -r ubuntu

###FILL HERE### (make ros2 command available by default for user)
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/user/.bashrc

# gosu 설치
RUN apt-get update && apt-get install -y wget gnupg2 && \
    wget -O /usr/local/bin/gosu "https://github.com/tianon/gosu/releases/download/1.16/gosu-amd64" && \
    chmod +x /usr/local/bin/gosu && \
    gosu nobody true

###---------###

# Entrypoint
COPY docker/scripts/entrypoint.sh /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

CMD ["/bin/bash"]
