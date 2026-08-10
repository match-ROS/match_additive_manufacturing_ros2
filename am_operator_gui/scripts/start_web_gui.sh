#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
venv_dir="${package_dir}/.web-venv"
workspace_dir="$(cd "${package_dir}/../../.." && pwd)"
workspace_setup="${workspace_dir}/install/setup.bash"

# Prefer the caller's overlay: it is the only reliable way to run this source
# checkout against a deliberately selected workspace.  A bare ROS install is
# not an overlay, so in that case use the workspace that contains this script.
overlay_sourced=false
IFS=':' read -r -a ament_prefixes <<< "${AMENT_PREFIX_PATH:-}"
for prefix in "${ament_prefixes[@]}"; do
  if [[ -n "${prefix}" && "${prefix}" != /opt/ros/* ]]; then
    overlay_sourced=true
    break
  fi
done

if [[ "${overlay_sourced}" != true ]]; then
  if [[ ! -f "${workspace_setup}" ]]; then
    echo "AM Operator Web GUI: no sourced workspace overlay and ${workspace_setup} is missing." >&2
    echo "Build the workspace first (for example: colcon build --packages-up-to mur_launch_sim am_operator_gui)." >&2
    exit 1
  fi
  # shellcheck disable=SC1090
  # Generated colcon setup scripts may inspect optional variables such as
  # COLCON_TRACE without defaulting them, which is incompatible with nounset.
  set +u
  source "${workspace_setup}"
  set -u
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "AM Operator Web GUI: ros2 is unavailable; source /opt/ros/jazzy/setup.bash first." >&2
  exit 1
fi

mur_launch_prefix="$(ros2 pkg prefix mur_launch_sim 2>/dev/null || true)"
if [[ -z "${mur_launch_prefix}" ]]; then
  echo "AM Operator Web GUI: mur_launch_sim is not available in the selected workspace overlay." >&2
  echo "Build/source a workspace containing match_mobile_robotics_jazzy before starting the GUI." >&2
  exit 1
fi
if [[ "${overlay_sourced}" != true && "${mur_launch_prefix}" != "${workspace_dir}/install"* ]]; then
  echo "AM Operator Web GUI: mur_launch_sim resolved outside this workspace: ${mur_launch_prefix}" >&2
  echo "Build ${workspace_dir} so it provides mur_launch_sim instead of using an underlay." >&2
  exit 1
fi

am_operator_prefix="$(ros2 pkg prefix am_operator_gui 2>/dev/null || true)"
if [[ "${overlay_sourced}" == true ]]; then
  overlay_description="caller-sourced"
else
  overlay_description="script workspace (${workspace_dir})"
fi
echo "AM Operator Web GUI: using ${overlay_description} overlay"
echo "AM Operator Web GUI: am_operator_gui=${am_operator_prefix:-source checkout}"
echo "AM Operator Web GUI: mur_launch_sim=${mur_launch_prefix}"

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
