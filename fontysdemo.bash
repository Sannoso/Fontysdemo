echo you have started the fontys pick and place demo
echo first enable robot
rosrun baxter_tools enable_robot.py -e
echo now tuck the arms
rosrun baxter_tools tuck_arms.py -u
echo now it is finally time to start the real demo
rosrun fontysdemo vanafscratch.py
