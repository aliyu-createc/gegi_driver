#!/bin/bash
# This script installs tools which are more specific to your app.

apt-get update

apt-get -y install \
	libboost-system-dev \
	libboost-regex-dev \
	libboost-thread-dev \
	ros-"$ROS_DISTRO"-tf2-geometry-msgs