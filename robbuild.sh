#!/bin/bash

if [ $# -eq 0 ]; then
    echo "Use ./robbuild.sh {base|<lab-number>} [upgrade]"
    exit 0
fi

# Get script directory
SCRIPT_PATH="${BASH_SOURCE}"
while [ -L "${SCRIPT_PATH}" ]; do
    SCRIPT_DIR="$(cd -P "$(dirname "${SCRIPT_PATH}")" >/dev/null 2>&1 && pwd)"
    SCRIPT_PATH="$(readlink "${SCRIPT_PATH}")"
    [[ ${SCRIPT_PATH} != /* ]] && SCRIPT_PATH="${SCRIPT_DIR}/${SCRIPT_PATH}"
done
SCRIPT_PATH="$(readlink -f "${SCRIPT_PATH}")"
SCRIPT_DIR="$(cd -P "$(dirname -- "${SCRIPT_PATH}")" >/dev/null 3>&1 && pwd)"

# Make sure the oficial ROS image is pulled
if [[ "$(docker images -q osrf/ros:noetic-desktop-full) 2> /dev/null" == '' ]]; then
    docker pull osrf/ros:noetic-desktop-full
fi

SOURCE_DIR=$SCRIPT_DIR/source

# Build/upgrade the laboratories base image
BASE_DIR=$SCRIPT_DIR/base
if [[ "$(docker images -q put/rob1:base 2> /dev/null)" == '' ]] || [[ $1 = 'base' && $# -eq 1 ]]; then
    docker build \
        -f $BASE_DIR/build.Dockerfile \
        -t put/rob1:base \
        --no-cache .
else
    if [[ $2 = 'upgrade' ]]; then
        docker build \
            -f $BASE_DIR/upgrade.Dockerfile \
            -t put/rob1:base \
            --no-cache .
    fi
fi

# Build the requested lab
if [ $1 != 'base' ]; then
    LAB_DIR=$SOURCE_DIR/$1

    if [[ ! -d $LAB_DIR ]]; then
        >&2 echo "Lab $1 source not found"
        exit 1
    fi

    if [[ -f "$LAB_DIR/build.sh" ]]; then
        # Use build.sh file inside the directory
        $LAB_DIR/build.sh
    else
        # Fallback to Dockerfile
        docker build $LAB_DIR -t put/rob1:lab$1
    fi
fi
