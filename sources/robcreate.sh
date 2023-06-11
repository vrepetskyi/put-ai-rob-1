#!/bin/bash

if [[ $# -eq 0 ]]; then
    echo './robcreate.sh <lab-id> (<docker-run-options>)'
    echo
    echo 'Used to create new containers'
    echo
    echo 'Usually called via robgo - see its help'
    echo
    echo 'Use this script if you want to simultaneously have'
    echo 'two separate containers based on the same image'
    echo
    echo 'The inner workings:'
    echo '- builds the <lab-id> image if not yet built'
    echo '- creates a container from the <lab-id> image'
    echo '- configures GUI inside the container'
    echo '- mounts ./shared/ to the container'\''s root (/)'
    echo '- applies the provided docker run options'
    exit 0
fi

# GUI support
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ -n "$xauth_list" ]; then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

script_path="${BASH_SOURCE[0]}"
while [ -L "${script_path}" ]; do
    script_dir="$(cd -P "$(dirname "${script_path}")" >/dev/null 2>&1 && pwd)"
    script_path="$(readlink "${script_path}")"
    [[ ${script_path} != /* ]] && script_path="${script_dir}/${script_path}"
done
script_path="$(readlink -f "${script_path}")"
script_dir="$(cd -P "$(dirname -- "${script_path}")" >/dev/null 3>&1 && pwd)"

parent_dir=$(dirname "$script_dir")

# Explanation:
# docker run \
#     -it \                                             Run shell in interactive mode (you'll be able to enter the commands eternally)
#     --privileged \                                    Makes possible to access devices on your PC from the container
#     --network host \                                  Exposes your network to the container
#     --volume "$script_dir/shared:/shared:rw"\         Mounts ./shared/ to the container's root (/) so you can transfer files to/from it
#     --env "DISPLAY=$DISPLAY" \                        Specifies the target display. Most of the time you want it's value to be :0
#     --env "QT_X11_NO_MITSHM=1" \                      GUI support
#     --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \         ...
#     --env "XAUTHORITY=$XAUTH" \                           ...
#     --volume "$XAUTH:$XAUTH" \                            ...
#     --env "NVIDIA_VISIBLE_DEVICES=all" \              NVIDIA support (the GPUs are not necessary for this course)
#     --env "NVIDIA_DRIVER_CAPABILITIES=all" \              ...
#     --env "LAB_ID=$1" \                               Used in the robshare and robget scripts
#     ${@:2} \                                          Unwrap the arguments you've passed starting from the second one
#     "$image" \                                        Run the <lab-id> image; you've passed it as the first argument
#     /bin/bash                                         Run bash (you don't have to type it after the first argument)

docker run \
    -it \
    --privileged \
    --network host \
    --volume "$parent_dir/materials:/materials:rw" \
    --volume "$parent_dir/solutions:/solutions:rw" \
    --env "DISPLAY=$DISPLAY" \
    --env "QT_X11_NO_MITSHM=1" \
    --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env "XAUTHORITY=$XAUTH" \
    --volume "$XAUTH:$XAUTH" \
    --env "NVIDIA_VISIBLE_DEVICES=all" \
    --env "NVIDIA_DRIVER_CAPABILITIES=all" \
    --env "LAB_ID=$1" \
    --label "lab-id=$1" \
    "${@:2}" \
    "put/ai-rob-1:$1" \
    /bin/bash
