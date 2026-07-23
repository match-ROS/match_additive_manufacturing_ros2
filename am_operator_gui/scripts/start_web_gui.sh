#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
venv_dir="${package_dir}/.web-venv"

if [[ ! -x "${venv_dir}/bin/python" ]]; then
  # ROS Python packages are provided by the sourced system/workspace install.
  # Keep them visible while isolating the small web-only dependency set.
  python3 -m venv --system-site-packages "${venv_dir}"
  "${venv_dir}/bin/python" -m pip install -r "${package_dir}/requirements-web.txt"
fi

# The source package is used directly, which also makes the script work before a
# colcon build. Source the ROS workspace first when process control is required.
export PYTHONPATH="${package_dir}${PYTHONPATH:+:${PYTHONPATH}}"
exec "${venv_dir}/bin/python" -m am_operator_gui.web_main
