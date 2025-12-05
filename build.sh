#!/usr/bin/env bash
set -e

set +e
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y 2>&1 | tee /tmp/rosdep_output.log
rosdep_exit_code=${PIPESTATUS[0]}
set -e

if [ $rosdep_exit_code -ne 0 ]; then
    if grep -q "your rosdep installation has not been initialized yet" /tmp/rosdep_output.log; then
        echo "rosdep install failed because it is not initialized."
        if grep -q "sudo rosdep init" /tmp/rosdep_output.log; then
            read -p "Do you want to run 'sudo rosdep init'? [Y/n] " -r
            if [[ $REPLY =~ ^[Yy]$ ]] || [[ -z $REPLY ]]; then
                sudo rosdep init
            else
                exit 1
            fi
        fi
        if grep -q "rosdep update" /tmp/rosdep_output.log; then
            read -p "Do you want to run 'rosdep update'? [Y/n] " -r
            if [[ $REPLY =~ ^[Yy]$ ]] || [[ -z $REPLY ]]; then
                rosdep update
            else
                exit 1
            fi
        fi
        rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
    else
        exit $rosdep_exit_code
    fi
fi
rm -f /tmp/rosdep_output.log

if [ $# -eq 0 ]; then
    # if argc == 0, build all packages
    colcon build --symlink-install \
        --event-handlers console_direct+ \
        --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
                    -DCMAKE_BUILD_TYPE=Release \
                    -GNinja \
        --parallel-workers "$(( $(nproc) / 2 ))"
else
    # if argc > 0, build only specified packages
    colcon build --symlink-install \
        --event-handlers console_direct+ \
        --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
                    -DCMAKE_BUILD_TYPE=Release \
                    -GNinja \
        --parallel-workers "$(( $(nproc) / 2 ))" \
        --packages-select "$@"
fi
