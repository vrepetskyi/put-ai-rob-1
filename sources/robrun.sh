#!/bin/bash

if [[ $# -eq 0 ]]; then
    echo './robrun.sh <lab-id> (<docker-run-options>)'
    echo
    echo 'Used to create new containers'
    echo
    echo 'Usually called via robgo - see its help'
    echo
    echo 'Use this script if you want to simultaneously have'
    echo 'two separate containers based on the same image'
    echo
    echo 'The inner workings:'
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

# Lookup the target image id using aliases.txt
# The format is: "<alias-name> <image-id>"
lab_id="$1"
image_id="$(rg "^$1" <"$script_dir/aliases.txt" | cut -d ' ' -f 2)"
image_id="${image_id:=$1}"

# Check if the target image exists
if [[ ! "$(docker images --quiet "put/ai-rob-1:$image_id")" ]]; then
    "$script_dir/robbuild.sh" "$image_id" || exit 1
fi

echo "Creating a new container"
docker run \
    -it \
    --privileged \
    --network host \
    --env "DISPLAY=$DISPLAY" \
    --env QT_X11_NO_MITSHM=1 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --env "XAUTHORITY=$XAUTH" \
    --volume "$XAUTH:$XAUTH" \
    --env NVIDIA_VISIBLE_DEVICES=all \
    --env NVIDIA_DRIVER_CAPABILITIES=all \
    --volume "$parent_dir/materials:/materials:rw" \
    --volume "$parent_dir/solutions:/solutions:rw" \
    --label subject-id=put-ai-rob-1 \
    --label "lab-id=$lab_id" \
    --env "LAB_ID=$lab_id" \
    "${@:2}" \
    "put/ai-rob-1:$image_id" \
    /bin/bash

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
