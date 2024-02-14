#!/bin/bash
set -e 

# Get key
add-apt-repository -y ppa:git-core/ppa 

apt-get update
apt-get upgrade -y

apt-get install -y --allow-unauthenticated \
	python3-pip \
	terminator \
	gedit \
	cmake \
	git \
	vim \
	tree \
	software-properties-common \
	apt-transport-https \
	wget

apt-get update
apt-get upgrade -y