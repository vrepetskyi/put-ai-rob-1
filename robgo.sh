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
    echo 'See robbuild'\''s and robrun'\''s helps for more use cases'
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

# Look for an already existing container
existing="$(docker container list --filter label=subject-id=put-ai-rob-1 --filter label=lab-id="$1" --latest --quiet)"
if [[ "$existing" ]]; then
    docker start "$existing" &>/dev/null
    name="$(docker container list -all --filter "id=$existing" --format "{{.Names}}")"
    echo "Connecting to an existing container: $existing ($name)"
    docker exec -it "$existing" /bin/bash
    exit 0
fi

# Run a new container from the target image
# shellcheck disable="SC2068"
"$sources/robrun.sh" $@
