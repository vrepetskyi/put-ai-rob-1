#!/bin/bash

# Get script directory
SCRIPT_PATH="${BASH_SOURCE}"
while [ -L "${SCRIPT_PATH}" ]; do
    SCRIPT_DIR="$(cd -P "$(dirname "${SCRIPT_PATH}")" >/dev/null 2>&1 && pwd)"
    SCRIPT_PATH="$(readlink "${SCRIPT_PATH}")"
    [[ ${SCRIPT_PATH} != /* ]] && SCRIPT_PATH="${SCRIPT_DIR}/${SCRIPT_PATH}"
done
SCRIPT_PATH="$(readlink -f "${SCRIPT_PATH}")"
SCRIPT_DIR="$(cd -P "$(dirname -- "${SCRIPT_PATH}")" >/dev/null 3>&1 && pwd)"

# We don't use cache in this block, because we want to fetch packages every time
if [[ "$(docker images -q put/rob1:base 2> /dev/null)" == '' ]] || [ $1 = 'base' ]; then
    # Build base docker image if it hasn't been done yet (or it is explicitly requested)

    # Pull the oficial image
    docker pull osrf/ros:noetic-desktop-full

    # Extend it for our needs
    docker build \
        $SCRIPT_DIR/source/base \
        -t put/rob1:base \
        --no-cache
# else
    # Upgrade the base image if it already exists:
    # docker build \
    #     -f $SCRIPT_DIR/source/base/upgrade.Dockerfile \
    #     -t put/rob1:base \
    #     --no-cache \
fi

# Build the requested image
# (if it isn't base which would have been built earlier)
if [ $1 != 'base' ]; then
    docker build $SCRIPT_DIR/source/$1 -t put/rob1:lab$1
fi
