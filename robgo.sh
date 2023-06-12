#!/bin/bash

if [[ $# -eq 0 ]]; then
    echo './robgo.sh <lab-id> (<docker-run-options>)'
    echo
    echo 'The primary way of connecting to the environment of each laboratory'
    echo
    echo 'Automatically performs the setup steps (only if needed):'
    echo '- image build'
    echo '- container creation'
    echo '- container start'
    echo '- connection to the latest container'
    echo
    echo 'The options are passed to "docker run" on container creation. The most useful are:'
    echo '--rm                completely removes the container after you close the parent shell'
    echo '--gpus all        * shares your external GPU with the container'
    echo '--device          * shares a device with the container'
    echo '* - requires configuration if you use WSL (see README)'
    echo
    echo 'Direct use of child scripts:'
    echo '- robbuild to permorm a build without some caching'
    echo '- robrun to run multiple containers independently'
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
