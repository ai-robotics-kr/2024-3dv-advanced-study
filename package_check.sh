#!/bin/bash

# OpenCV 버전 확인
echo "Checking OpenCV version..."
opencv_version=$(pkg-config --modversion opencv4 2>&1)
echo "OpenCV version: $opencv_version"

# Ceres-solver 버전 확인
echo "Checking Ceres-solver version..."
ceres_version=$(ceres-solver --version 2>&1) # Ceres-solver에 해당하는 버전 확인 명령어를 입력하세요.
echo "Ceres-solver version: $ceres_version"