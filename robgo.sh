#!/bin/bash

if [[ $# -eq 0 ]]; then
    echo './robgo.sh <lab-id> [--rm] [--gpus] [--device] (<other-options>)'
    echo
    echo 'The primary way of running a container. Handles the setup for you'
    echo
    echo 'The inner workings:'
    echo '- builds the <lab-id> image if not yet built'
    echo '- creates a <lab-id> container if none exist'
    echo '- starts the latest created <lab-id> container if none is running'
    echo '- execs an interactive bash on the latest started <lab-id> container'
    echo
    echo 'The provided options are only used if a new container has to be created'
    echo
    echo 'They are the same as you would use for docker run:'
    echo '--rm              completely removes the container after you close the parent shell'
    echo '--gpus all      * provides the external GPU to the container'\''s disposal'
    echo '--device        * shares a device with the container'
    echo
    echo 'The ones marked with * require configuration for WSL (see README):'
    echo
    echo 'See robbuild'\''s and robcreate'\''s helps for more use cases'
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

sources="$script_dir/sources"

# TODO conditional build

# Try to build the image
"$sources/robbuild.sh" "$1" || exit 1

# Look for an already started container
target="$(docker ps --quiet --latest --filter "ancestor=put/ai-rob-1:$1" --filter "label=lab-id=$1")"

if [[ ! "$target" ]]; then
    # Look for an already created, but not yet started conatiner
    target="$(docker ps --all --quiet --latest --filter "ancestor=put/ai-rob-1:$1" --filter "label=lab-id=$1")"
fi

if [[ $target ]]; then
    name="$(docker ps -af "id=$target" --format "{{.Names}}")"
    echo "Connecting to an existing container: $target ($name)"
    docker start "$target" &>/dev/null && docker exec -it "$target" /bin/bash
else
    echo "Creating a new container"
    # shellcheck disable="SC2068"
    "$sources/robcreate.sh" $@
fi
