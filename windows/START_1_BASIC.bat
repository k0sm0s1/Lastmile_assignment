@echo off
title Lastmile - basic assignment (no obstacles, no delivery page) (keep this window open)
echo Starting: basic assignment (no obstacles, no delivery page)
echo Anything already running is stopped first. This takes 1-2 minutes. KEEP THIS WINDOW OPEN.
wsl -e bash -lc "cd ~/lastmile_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && GUI=1  bash scripts/run_demo.sh && cmd.exe /c start http://localhost:8080; echo; echo '=== Running: basic assignment (no obstacles, no delivery page)==='; echo '=== To stop: double-click STOP_EVERYTHING.bat, then close this window ==='; sleep infinity"
