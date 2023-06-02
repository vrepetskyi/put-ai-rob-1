#!/bin/bash

if [[ $# -ne 1 ]]; then
    echo 'robshare <target>'
    echo
    echo 'Helps not to lose the progress on container removal'
    echo
    echo 'Run this script from a container created with robstart or robrun'
    echo 'Alias "robshare" is available there'
    echo
    echo 'Moves/restores a file or directory to/from /shared/$LAB_ID/'
    echo
    echo 'e.g. "robshare src/some-catkin-package" will be resolved the following way:'
    echo 'src/some-catkin-package will point to /shared/$LAB_ID/some-catkin-package;'
    echo 'some-catkin-package will be moved there first in case it is not yet shared'
    exit
fi

script_path="${BASH_SOURCE[0]}"
while [ -L "${script_path}" ]; do
    script_dir="$(cd -P "$(dirname "${script_path}")" >/dev/null 2>&1 && pwd)"
    script_path="$(readlink "${script_path}")"
    [[ ${script_path} != /* ]] && script_path="${script_dir}/${script_path}"
done
script_path="$(readlink -f "${script_path}")"
script_dir="$(cd -P "$(dirname -- "${script_path}")" >/dev/null 3>&1 && pwd)"

lab_folder="$script_dir/$LAB_ID"
target="${1##*/}"
shared="$lab_folder/$target"

if [[ -f "$shared" ]]; then
    ln -sf "$shared" "$1"
    echo "Created a symlink from $1 to $shared"
elif [[ -d "$shared" ]]; then
    ln -sf "$shared/" "$1"
    echo "Created a symlink from $1/ to $shared/"
elif [[ -f "$1" || -d "$1" ]]; then
    mv "$1" "$lab_folder/"
    if [[ -f "$1" ]]; then
        echo "Moved $1 to $lab_folder/"
    else
        echo "Moved $1/ to $lab_folder/"
    fi
    "$script_path" "$1"
fi
