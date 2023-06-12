#!/bin/bash

if [[ $# -ne 1 ]]; then
    echo 'robshare <target>'
    echo
    echo 'Helps to keep your progress after container removal'
    echo
    echo 'Moves/restores a file/directory to/from /shared/$LAB_ID/'
    echo
    echo 'e.g. "robshare src/some-catkin-package" will be resolved the following way:'
    echo 'src/some-catkin-package will point to /shared/$LAB_ID/some-catkin-package;'
    echo 'some-catkin-package will be moved there first in case it is not yet shared'
    echo
    echo 'Run this script from a container created with robgo or robrun'
    echo 'Alias robshare is available there'
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

# Exit if a place is alredy occupied by another file/directory/symlink
if [[ -f "$source" || -s "$source" ]]; then
    echo >&2 "Cannot create a symlink as path \"$source\" is already occupied"
    exit 1
fi

if [[ -f "$shared" || -d "$shared" ]]; then
    # Create the symlink right away if the target is already shared
    ln -s "$shared" "$source"
    echo "Created a symlink from $source to $shared"
elif [[ -f "$source" || -d "$source" ]]; then
    # Otherwise, share the target first and then create a symlink
    mkdir -p "$solution"
    mv "$source" "$solution/"
    if [[ -f "$shared" ]]; then
        echo "Moved $source to $solution/"
    else
        echo "Moved $source/ to $solution/"
    fi
    chmod +777 -R "$solution"
    "$script_path" "$source"
else
    # Invalid argument. Nothing was found
    echo >&2 "Neither \"$source\" nor \"$shared\" has been found"
    exit 1
fi
