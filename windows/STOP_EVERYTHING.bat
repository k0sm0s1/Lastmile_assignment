@echo off
wsl -e bash -lc "cd ~/lastmile_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && bash scripts/run_demo.sh stop"
echo Stopped.
pause
