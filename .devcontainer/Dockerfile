ARG ROS_DISTRO=kilted
FROM ros:${ROS_DISTRO}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ccache \
    geographiclib-tools \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    vim \
  && rm -rf /var/lib/apt/lists/*

COPY mavros/scripts/install_geographiclib_datasets.sh /tmp/install_geographiclib_datasets.sh
RUN bash /tmp/install_geographiclib_datasets.sh && rm -f /tmp/install_geographiclib_datasets.sh
