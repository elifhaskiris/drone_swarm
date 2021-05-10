#!/bin/bash
cd ~/ardupilot/ArduCopter/
gnome-terminal \
 --tab -e "sim_vehicle.py --instance 0" \
 --tab -e "sim_vehicle.py --instance 1" \
 --tab -e "sim_vehicle.py --instance 2" \
 --tab -e "sim_vehicle.py --instance 3" \
 --tab -e "sim_vehicle.py --instance 4" \
 --tab -e "sim_vehicle.py --instance 5" \
 --tab -e "sim_vehicle.py --instance 6" \
 --tab -e "sim_vehicle.py --instance 7" \
 --tab -e "sim_vehicle.py --instance 8" \
 --tab -e "sim_vehicle.py --instance 9" \


