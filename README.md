# drone_swarm
Installation

https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/Installing_Ardupilot.md
https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_mission_on_Linux.md
https://dronekit-python.readthedocs.io/en/latest/develop/installation.html

1-Start Mission Planner

cd Missionplanner directory

sudo mono MissionPlanner.exe

2- Start Drones

Another terminal tab
multi_h.sh # starts 10 drone 
cd drone_swarm
./multi_h.sh

3-Connect drones to Mission planner

Right clik on top bar and select connection options
UDP
57600 
Connect
14550 # this will increase by 10 for each drone 14560, 14570 ... 14640

4-Bring drones to starting  posiition
start.py # brings drone to the starting position

cd drone_swarm
python start.py

5-Shape fromation

cd drone_swarm
python test_square.py


