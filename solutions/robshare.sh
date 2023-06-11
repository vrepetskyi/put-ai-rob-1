#!/bin/bash

if [[ $# -ne 1 ]]; then
    echo 'robshare <target>'
    echo
    echo 'Helps not to lose the progress on container removal'
    echo
    echo 'Run this script from a container created with robgo or robcreate'
    echo 'Alias "robshare" is available there'
    echo
    echo 'Moves/restores a file or directory to/from /shared/$LAB_ID/'
    echo
    echo 'e.g. "robshare src/some-catkin-package" will be resolved the following way:'
    echo 'src/some-catkin-package will point to /shared/$LAB_ID/some-catkin-package;'
    echo 'some-catkin-package will be moved there first in case it is not yet shared'
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

# The original path of the target
source="${1%/}"
# The target name
target="${source##*/}"
# The solution directory
solution="$script_dir/$LAB_ID"
# The new path of the target
shared="$solution/$target"

# TODO: fix symlink is subdir

# Symlink right away if the target is already shared
if [[ -f "$shared" ]]; then
    ln -sf "$shared" "$source"
    echo "Created a symlink from $source to $shared"
elif [[ -d "$shared" ]]; then
    ln -sf "$shared/" "$source"
    echo "Created a symlink from $source/ to $shared/"
# Otherwise, share it first
elif [[ -f "$source" || -d "$source" ]]; then
    mkdir -p "$solution"
    mv "$source" "$solution/"
    if [[ -f "$shared" ]]; then
        echo "Moved $source to $solution/"
    else
        echo "Moved $source/ to $solution/"
    fi
    # TODO empty dir chmod error
    chmod +777 -R "$solution"
    "$script_path" "$source"
fi
