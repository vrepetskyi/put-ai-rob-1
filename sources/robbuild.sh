#!/bin/bash

if [[ $# -eq 0 ]]; then
    echo './robbuild.sh base|<lab-id> [upgrade|full-upgrade]'
    echo
    echo 'Used to build the <lab-id> Docker image from ./source/<lab-id>/'
    echo
    echo 'Usually called via robcreate - see its help'
    echo
    echo 'Use this script if you want to update the cached images'
    echo
    echo 'The inner workings:'
    echo '- makes sure the ROS image is pulled'
    echo '- by default tries to run the corresponding build.sh script'
    echo '- if build.sh doesn'\''t exist fallbacks to the direct usage of Dockerfiles:'
    echo '  - first builds the base image'
    echo '  - then builds the <lab-id> image'
    echo
    echo 'Build arguments:'
    echo '- upgrade runs the identically named apt command on the existing base image'
    echo '- full-upgrade looks for a new ROS image and rebuilds the base image from scratch'
    echo
    echo 'Inside the build.sh you may customize the build proccess beyond Dockerfile capabilities'
    echo 'Parts of this script'\''s functionality can still be reused there'
    echo 'All the arguments are passed unchanged'
    exit 0
fi

script_path="${BASH_SOURCE[0]}"
while [ -L "${script_path}" ]; do
    script_dir="$(cd -P "$(dirname "${script_path}")" >/dev/null 2>&1 && pwd)"
    script_path="$(readlink "${script_path}")"
    [[ ${script_path} != /* ]] && script_path="${script_dir}/${script_path}"
done
script_path="$(readlink -f "${script_path}")"
script_dir="$(cd -P "$(dirname -- "${script_path}")" >/dev/null 3>&1 && pwd)"

# Build the base image on request
if [[ $1 = 'base' ]]; then
    # Pull the oficial ROS image
    if [[ $(docker images -q osrf/ros:noetic-desktop-full 2>/dev/null) == '' || $2 = 'full-upgrade' ]]; then
        docker pull osrf/ros:noetic-desktop-full
    fi

    # Pick an appropriate cache policy
    base_dir=$script_dir/base
    if [[ $2 = 'full-upgrade' ]]; then
        docker build -f "$base_dir/build.Dockerfile" -t put/ai-rob-1:base --no-cache .
    elif [[ $2 = 'upgrade' ]]; then
        docker build -f "$base_dir/upgrade.Dockerfile" -t put/ai-rob-1:base --no-cache .
    elif [[ $(docker images -q put/ai-rob-1:base 2>/dev/null) == '' ]]; then
        docker build -f "$base_dir/build.Dockerfile" -t put/ai-rob-1:base .
    fi

    exit 0
fi

# Build an image for a particular laboratory
lab_dir=$script_dir/$1
if [[ -d "$lab_dir" && -f "$lab_dir/build.sh" ]]; then
    # Try to use custom build script
    # shellcheck disable="SC2068"
    "$lab_dir/build.sh" $@
elif [[ -f "$lab_dir/Dockerfile" ]]; then
    # Otherwise fallback to base + extended image
    $script_path base "$2"
    docker build "$lab_dir" -t "put/ai-rob-1:$1"
else
    echo >&2 "Source for laboratory $1 is not found"
    exit 1
fi
