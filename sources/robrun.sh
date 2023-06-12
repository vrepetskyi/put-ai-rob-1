#!/bin/bash

if [[ $# -eq 0 ]]; then
    echo './robrun.sh <lab-id> (<docker-run-options>)'
    echo
    echo 'Creates a new container'
    echo
    echo 'The steps:'
    echo '- resolves the image ID using aliases'
    echo '- builds the image'
    echo '- creates a container'
    echo '- configures the network, GUI, and GPU inside the container'
    echo '- mounts ../materials/ and ../solutions/ to the container'\''s root (/)'
    echo '- labels the container with subject-id and lab-id'
    echo '- applies the provided options'
    echo '- runs an interactive Bash from the container'
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
