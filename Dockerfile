FROM ros:humble-ros-base

RUN set -x \
    && sed -i 's_/archive.ubuntu.com_/us.archive.ubuntu.com_g' /etc/apt/sources.list

COPY apt /tmp/dtcbuild/

RUN set -x \
    && apt-get update \
    && apt-get upgrade -y \
    && DEBIAN_FRONTEND=noninteractive xargs -a /tmp/dtcbuild/apt-dependencies.txt \ 
        apt-get install --no-install-recommends -y \
    && apt-get remove modemmanager -y \
    && rm -rf /var/lib/apt/lists/*

RUN set -x \
    && groupadd -g 1000 dtcbuild \
    && useradd -m -g dtcbuild -u 1000 dtcbuild \
    && echo "dtcbuild:dtcbuild" | chpasswd \
    && adduser dtcbuild sudo \
    && echo "dtcbuild ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers \
    && chown -R dtcbuild:dtcbuild /home/dtcbuild/

ENV USER=dtcbuild
RUN usermod -a -G dialout $USER

USER dtcbuild
COPY ./ardupilot.zip /home/dtcbuild/ardupilot.zip
RUN cd /home/dtcbuild && unzip ardupilot.zip
RUN sudo chown -R dtcbuild /home/dtcbuild/ardupilot
# RUN git clone https://github.com/ArduPilot/ardupilot.git /home/dtcbuild/ardupilot
RUN sudo chmod +x /home/dtcbuild/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh
WORKDIR /home/dtcbuild/ardupilot

RUN git config --global --add safe.directory /home/dtcbuild/ardupilot
RUN Tools/environment_install/install-prereqs-ubuntu.sh -y

RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
RUN chmod a+x install_geographiclib_datasets.sh
RUN sudo ./install_geographiclib_datasets.sh

RUN . ~/.profile && ./modules/waf/waf-light configure --board sitl
RUN . ~/.profile && ./modules/waf/waf-light build --target bin/arducopter

### QGroundControl
RUN wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage --directory-prefix=/home/dtcbuild/
RUN chmod +x /home/dtcbuild/QGroundControl.AppImage

USER dtcbuild
WORKDIR /home/dtcbuild/project

COPY --chown=dtcbuild:dtcbuild /project /home/dtcbuild/project
RUN . /opt/ros/humble/setup.sh \
    && colcon build