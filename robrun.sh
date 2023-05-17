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

# Something for GUI
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
     chmod a+r $XAUTH
fi

# Explanation:
# docker run \
#     -it \                                             Run shell in interactive mode (you'll be able to enter commands eternally)
#     --privileged \                                    Makes possible to access devices on your PC from container
#     --network host \                                  Exposes container to your network
#     --volume "$SCRIPT_DIR/shared:/shared:rw"\         Mounts ./shared/ to the container root so you can transfer files to/from it
#     --env "DISPLAY=$DISPLAY" \                        Most of the time you want it's value to be :0
#     --env "QT_X11_NO_MITSHM=1" \                      Something for GUI (again)
#     --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \     ...
#     --env "XAUTHORITY=$XAUTH" \                       ...
#     --volume "$XAUTH:$XAUTH" \                        ...
#     --env "NVIDIA_VISIBLE_DEVICES=all" \              Something for NVIDIA GPU (you won't really have to use it)
#     --env "NVIDIA_DRIVER_CAPABILITIES=all" \          ...
#     ${@:2} \                                          Unwrap the arguments you've passed starting from the second; some that you may want to add:
#                                                           --rm
#                                                               (delete container after you've closed all the shells)
#                                                           --gpus all
#                                                               (let your external GPU do the work; additional configuration will probably be required)
#                                                           --device *device*
#                                                               (expose a device to the container; configuration (A LOT) is required on WSL)
#     put/rob1:lab$1 \                                  Run an image under a number you've passed as a first argument;
#                                                       you should've built it using ./robbuild.sh *lab number*
#     /bin/bash                                         Run bash (you don't have to type it at the end)

docker run \
    -it \
    --privileged \
    --network host \
    --volume "$SCRIPT_DIR/shared:/shared:rw" \
    --env "DISPLAY=$DISPLAY" \
    --env "QT_X11_NO_MITSHM=1" \
    --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env "XAUTHORITY=$XAUTH" \
    --volume "$XAUTH:$XAUTH" \
    --env "NVIDIA_VISIBLE_DEVICES=all" \
    --env "NVIDIA_DRIVER_CAPABILITIES=all" \
    ${@:2} \
    "put/rob1:lab$1" \
    /bin/bash
