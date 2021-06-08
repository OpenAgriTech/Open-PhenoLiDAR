FROM ros:melodic-ros-base
LABEL maintainer="Jose Jimenez-Berni<berni@ias.csic.es>"

WORKDIR /workspace

RUN . /opt/ros/$ROS_DISTRO/setup.sh && mkdir -p /workspace/src && \
    cd /workspace && catkin_make

COPY agrolaser_node/ /workspace/src/agrolaser_node
COPY lms400_driver/ /workspace/src/lms400_driver

SHELL ["/bin/bash", "-c"]
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -y \
      --from-paths \
        /workspace/src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

RUN . /workspace/devel/setup.bash && \
    cd /workspace && catkin_make

# Dependency from https://github.com/mavlink/mavros/blob/master/mavros/README.md
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
RUN chmod +x install_geographiclib_datasets.sh
RUN ./install_geographiclib_datasets.sh



RUN apt-get update && apt-get install --no-install-recommends -y python-tables \
    && rm -rf /var/lib/apt/lists/*


# setup entrypoint
COPY agrolaser_node/Docker/ros_entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 9090

RUN echo "source /workspace/devel/setup.bash" >> ~/.bashrc && \
    source ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["roslaunch", "agrolaser_node", "agrolaser_mavros.launch"]
