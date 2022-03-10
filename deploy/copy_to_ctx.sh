#!/usr/bin/env zsh

# Get the current directory.
script_source=$(readlink -f "$0")
script_dir=$(dirname "$script_source")
docker_ctx=${script_dir}/docker_ctx
bin_dir="${script_dir}/../bin"

# Copy the python files to the context.
cp "${bin_dir}/*" "${docker_ctx}"

# Copy the installation script to the context.
cp "${script_dir}/install_simple.sh" "${docker_ctx}"
