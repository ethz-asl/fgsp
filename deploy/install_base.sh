#!/usr/bin/env zsh

# Base install
apt update
apt -y upgrade
apt -y install software-properties-common \
  python3 \
  python3-pip \
  python3-termcolor \
  git \
  curl \
  iproute2 \
  gnupg \
  lsb-release
