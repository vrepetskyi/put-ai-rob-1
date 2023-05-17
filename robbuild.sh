#!/bin/bash

# Handle help
if [[ $# -eq 0 || $1 = 'help' || $1 = '-h' || $1 = '--help' ]]; then
    echo './robbuild.sh [{<lab-id> | base | help | -h | --help}] [{upgrade | full-upgrade}]'
    echo
    echo 'Used for building Docker images from the source directory'
    echo
    echo 'upgrade will run the identically named apt command on the existing base image'
    echo 'full-upgrade will look for a new ROS image and rebuild the base image from scratch'
    exit 0
fi

# Get the script directory
SCRIPT_PATH="${BASH_SOURCE}"
while [ -L "${SCRIPT_PATH}" ]; do
    SCRIPT_DIR="$(cd -P "$(dirname "${SCRIPT_PATH}")" >/dev/null 2>&1 && pwd)"
    SCRIPT_PATH="$(readlink "${SCRIPT_PATH}")"
    [[ ${SCRIPT_PATH} != /* ]] && SCRIPT_PATH="${SCRIPT_DIR}/${SCRIPT_PATH}"
done
SCRIPT_PATH="$(readlink -f "${SCRIPT_PATH}")"
SCRIPT_DIR="$(cd -P "$(dirname -- "${SCRIPT_PATH}")" >/dev/null 3>&1 && pwd)"

SOURCE_DIR=$SCRIPT_DIR/source

# Handle base image
if [[ $1 = 'base' ]]; then
    # Pull the oficial ROS image
    if [[ $(docker images -q osrf/ros:noetic-desktop-full 2> /dev/null) == '' || $2 = 'full-upgrade' ]]; then
        docker pull osrf/ros:noetic-desktop-full
    fi

    # Build/upgrade the laboratories base image
    BASE_DIR=$SOURCE_DIR/base
    if [[ $2 = 'full-upgrade' ]]; then
        docker build -f $BASE_DIR/build.Dockerfile -t put/ai-rob-1:base --no-cache .
    elif [[ $2 = 'upgrade' ]]; then
        docker build -f $BASE_DIR/upgrade.Dockerfile -t put/ai-rob-1:base --no-cache .
    elif [[ $(docker images -q put/ai-rob-1:base 2> /dev/null) == '' ]]; then
        docker build -f $BASE_DIR/build.Dockerfile -t put/ai-rob-1:base .
    fi

    exit 0
fi

# Handle laboratory image
LAB_DIR=$SOURCE_DIR/$1

# Check if source directory exists
if [[ ! -d $LAB_DIR ]]; then
    >&2 echo "Source for laboratory $1 is not found"
    exit 1
fi

# Try to use custom build script
if [[ -f $LAB_DIR/build.sh ]]; then
    # Pass upgrade requests as a first argument
    $LAB_DIR/build.sh $2
    exit 0
fi

# Otherwise handle base image
$SCRIPT_PATH base $2

# And build laboratory's Dockerfile
docker build $LAB_DIR -t put/ai-rob-1:$1
