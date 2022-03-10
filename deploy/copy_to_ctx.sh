#!/usr/bin/env zsh

# Get the current directory.
script_source=$(readlink -f "$0")
script_dir=$(dirname "$script_source")
docker_ctx=${script_dir}/docker_ctx
bin_dir=${script_dir}/../bin/
config_dir=${script_dir}/../config/

# Copy the python files to the context.
rsync -a \
  --exclude '*.pyc' \
  --exclude '__pycache__' \
  --exclude '.ipynb_checkpoints' \
  "${bin_dir}" "${docker_ctx}"

  # Copty the configuration files to the context.
mkdir -p "${docker_ctx}/config"
rsync -a \
  "${config_dir}" "${docker_ctx}/config/"


# Copy the installation script to the context.
cp "${script_dir}/install_simple.sh" "${docker_ctx}"
