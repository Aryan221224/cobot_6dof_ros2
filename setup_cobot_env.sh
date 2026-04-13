#!/usr/bin/env bash
set -e
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'
banner() { echo ""; echo -e "${BLUE}${BOLD}============================================================${NC}"; echo -e "${BLUE}${BOLD}  $1${NC}"; echo -e "${BLUE}${BOLD}============================================================${NC}"; echo ""; }
ok()   { echo -e "  ${GREEN}✓${NC} $1"; }
info() { echo -e "  ${CYAN}→${NC} $1"; }
warn() { echo -e "  ${YELLOW}!${NC} $1"; }
fail() { echo -e "  ${RED}ERROR:${NC} $1"; exit 1; }
banner "6-DOF Cobot — Ubuntu 24.04 + ROS2 Jazzy Setup"
UBUNTU_VERSION=$(lsb_release -rs 2>/dev/null || echo "unknown")
IS_WSL=false
grep -qi microsoft /proc/version 2>/dev/null && IS_WSL=true
$IS_WSL && warn "WSL2 detected. Windows 11=works out of box. Windows 10=need VcXsrv."
info "Ubuntu: $UBUNTU_VERSION | ROS: Jazzy Jalisco"
banner "Step 1 — Removing stale ROS repository entries"
sudo rm -f /etc/apt/sources.list.d/ros2.list && ok "Cleared ros2.list"
sudo apt-get update -qq && ok "apt updated"
banner "Step 2 — System prerequisites"
sudo apt-get install -y curl gnupg2 lsb-release software-properties-common git python3-pip python3-colcon-common-extensions python3-rosdep python3-vcstool build-essential unzip
ok "Prerequisites installed"
banner "Step 3 — ROS2 Jazzy repository"
if dpkg -l ros-jazzy-desktop &>/dev/null; then
    ok "ROS2 Jazzy already installed"
else
    info "Adding GPG key..."
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    info "Adding Jazzy apt source (Ubuntu 24.04 noble)..."
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt-get update -qq
    banner "Installing ROS2 Jazzy Desktop (5-15 min)"
    sudo apt-get install -y ros-jazzy-desktop
    ok "ROS2 Jazzy Desktop installed"
fi
banner "Step 4 — Required ROS2 packages"
sudo apt-get install -y ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui ros-jazzy-robot-state-publisher ros-jazzy-xacro ros-jazzy-rviz2 ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-joint-trajectory-controller ros-jazzy-joint-state-broadcaster ros-jazzy-controller-manager ros-jazzy-urdf ros-jazzy-tf2-ros ros-jazzy-tf2-tools python3-colcon-common-extensions
ok "All packages installed"
banner "Step 5 — rosdep"
[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ] && sudo rosdep init
rosdep update --rosdistro jazzy
ok "rosdep ready"
banner "Step 6 — Building workspace"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$SCRIPT_DIR"
for i in 1 2 3 4; do
    [ -d "$WS_ROOT/src" ] && break
    WS_ROOT="$(dirname "$WS_ROOT")"
done
[ ! -d "$WS_ROOT/src" ] && fail "Cannot find workspace root"
info "Workspace: $WS_ROOT"
cd "$WS_ROOT"
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy 2>/dev/null || true
colcon build --packages-select cobot_description --symlink-install
ok "cobot_description built"
banner "Step 7 — Shell configuration"
BASHRC="$HOME/.bashrc"
WS_SETUP="$WS_ROOT/install/setup.bash"
if ! grep -q "source /opt/ros/jazzy/setup.bash" "$BASHRC"; then
    printf '\n# ROS2 Jazzy\nsource /opt/ros/jazzy/setup.bash\n' >> "$BASHRC"
    ok "Added ROS2 Jazzy to ~/.bashrc"
else
    ok "ROS2 Jazzy already in ~/.bashrc"
fi
if ! grep -q "$WS_SETUP" "$BASHRC"; then
    printf "\n# Cobot workspace\nsource $WS_SETUP\n" >> "$BASHRC"
    ok "Added workspace overlay to ~/.bashrc"
else
    ok "Workspace overlay already in ~/.bashrc"
fi
if $IS_WSL && ! grep -q 'DISPLAY=${DISPLAY' "$BASHRC"; then
    printf '\n# WSL2 display\nexport DISPLAY=${DISPLAY:-:0}\n' >> "$BASHRC"
    ok "Added DISPLAY for WSL2"
fi
banner "Setup Complete!"
echo -e "${GREEN}${BOLD}  Built for Articulus Surgical + Integra Robotics | ROS2 Jazzy / Ubuntu 24.04${NC}"
echo ""
