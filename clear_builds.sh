#! /bin/bash
# This file clears all build files.

find ./* -depth -name "build" -type d -exec rm -rf "{}" \; 
find ./* -depth -name ".egg-info" -type d -exec rm -rf "{}" \; 
find human_robot_gym/* -depth -name "__pycache__" -type d -exec rm -rf "{}" \; 