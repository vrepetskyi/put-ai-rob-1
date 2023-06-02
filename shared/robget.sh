#!/bin/bash

if [[ $# -ne 1 || $1 != 'pick' ]]; then
    echo 'robget pick'
    echo
    echo 'Meant as a source of truth for getting the materials'
    echo
    echo 'Run this script from a container created with robstart or robrun'
    echo 'Alias "robget" is available there'
    echo
    echo 'Promts you with the materials available for download for a particular lab'
    echo 'The selected entry will be downloaded to /shared/$LAB_ID/materials/'
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

materials="$script_dir/$LAB_ID/materials"
sources="$materials/sources.txt"

if [[ ! -f "$sources" ]]; then
    echo 'No materials available for the lab'
    exit 1
fi

selected="$(fzf <"$materials/sources.txt")"

if [[ $selected ]]; then
    # shellcheck disable="SC2086"
    wget -O $materials/$selected
else
    echo "Nothing was selected" >&2
    exit 1
fi
