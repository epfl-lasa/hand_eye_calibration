ARG BASE_TAG=noetic
FROM ros:${BASE_TAG} as environment-variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_NO_CACHE_DIR 1
ENV USER ros
ENV HOME /home/${USER}
ENV ROS_WORKSPACE /home/${USER}/ros_ws
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib


FROM environment-variables as base-dependencies

# disable suggested and recommended install
RUN apt-config dump | grep -we Recommends -e Suggests | sed s/1/0/ \
  | sudo tee /etc/apt/apt.conf.d/999norecommend

# install base dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
  curl \
  featherpad \
  gdb \
  git \
  iputils-ping \
  libeigen3-dev \
  libtool \
  mesa-utils \
  nano \
  python3-pip \
  python3-rosinstall-generator \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-rviz \
  ros-${ROS_DISTRO}-realsense2-camera \
  ros-${ROS_DISTRO}-rqt-graph \
  rsync \
  software-properties-common \
  ssh \
  unzip \
  wget \
  && rm -rf /var/lib/apt/lists/*

RUN echo "Set disable_coredump false" >> /etc/sudo.conf

# configure sshd server settings
RUN ( \
    echo 'LogLevel DEBUG2'; \
    echo 'PubkeyAuthentication yes'; \
    echo 'Subsystem sftp /usr/lib/openssh/sftp-server'; \
  ) > /etc/ssh/sshd_config_development \
  && mkdir /run/sshd


FROM base-dependencies as base-workspace
ENV USER ros
ENV HOME /home/${USER}
ENV ROS_WORKSPACE /home/${USER}/ros_ws

# create and configure a new user
ARG UID=1000
ARG GID=1000
RUN addgroup --gid ${GID} ${USER}
RUN adduser --gecos "ROS User" --uid ${UID} --gid ${GID} ${USER} && yes | passwd ${USER}
RUN usermod -a -G dialout ${USER}
RUN echo "${USER} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/99_aptget
RUN chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget

# configure sshd entrypoint to authorise the new user for ssh access and
# optionally update UID and GID when invoking the container with the entrypoint script
COPY ./config/sshd_entrypoint.sh /sshd_entrypoint.sh
RUN chmod 744 /sshd_entrypoint.sh

# build ROS workspace
USER ${USER}
RUN mkdir -p ${ROS_WORKSPACE}/src
WORKDIR ${ROS_WORKSPACE}
RUN rosdep update
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; catkin_make"

# create the credentials to be able to pull private repos using ssh
USER root
RUN mkdir /root/.ssh/ && ssh-keyscan github.com | tee -a /root/.ssh/known_hosts

# prepend the environment sourcing to bashrc (appending will fail for non-interactive sessions)
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash; \
source ${ROS_WORKSPACE}/devel/setup.bash" | cat - ${HOME}/.bashrc > tmp && mv tmp ${HOME}/.bashrc
RUN echo "session required pam_limits.so" | tee --append /etc/pam.d/common-session > /dev/null


FROM environment-variables as final
COPY --from=base-workspace / /

RUN pip install transforms3d
RUN pip install statsmodels
RUN pip install opencv-python

# start as ROS user
USER ${USER}
WORKDIR ${ROS_WORKSPACE}


