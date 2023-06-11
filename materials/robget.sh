#!/bin/bash

if [[ $# -ne 1 || $1 != 'pick' ]]; then
    echo 'robget pick'
    echo
    echo 'Meant as a source of truth for getting the materials'
    echo
    echo 'Run this script from a container created with robgo or robcreate'
    echo 'Alias "robget" is available there'
    echo
    echo 'Promts you with the materials available for download for a particular lab'
    echo 'The selected entry will be downloaded to /shared/$LAB_ID/materials/'
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

materials="$script_dir/$LAB_ID"
sources="$materials/lookup.txt"

if [[ ! -f "$sources" ]]; then
    echo 'No materials available for the laboratory'
    exit 1
fi

# fzf is an extremely useful tool for fuzzy search
# I recommend you to Google it
selected="$(fzf <"$sources")"

if [[ $selected ]]; then
    # Make use of word spllitting by not putting double quotes
    # (the space in $selected will divide it into 2 arguments)
    # shellcheck disable="SC2086"
    wget -O $materials/$selected
else
    echo 'Nothing was selected' >&2
    exit 1
fi
