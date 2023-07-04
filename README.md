# Autonomus-Navigation-System-
The following document shows how to execute the whole project of AxoloTec
Challenge, with the purpose that everyone that is allowed to access the files, is able
to execute the following instructions to run the code, this instructions assume that the
neural network code and model are loaded in a laptop or computer and the
remaining codes are on the Jeson Nano device:
1. First we need to open three terminals in order to run all the codes to make the
robot move.
2. Two of the terminals need to be a SSH connection and the remaining one
must be connected by the “export ROS_MASTER_URI” and “export ROS_IP”
methods.
3. It is important that the terminal that does not run SSH connections is the one
that runs the neural network since, as it is too heavy it needs a more powerful
device to run smoothly.
4. In one of the terminals with the SSH protocol you need to run the “roslaunch
ros_deep_learning video_source_ros1.launch” with a resize on the image to
320x320.
5. Now we need to move to the terminal that does not have SSH to run the
neural network, for the following commands we assume that you named the
ROS packages the same as us. Run “rosrun begginer_tutorials Yolo_v5.py”
as this will run the neural network.
6. And for the final SSH terminal you need to run “rosrun axolobot
Line_Follower.py” to make all the codes connect between them and make the
logic controller at the same time.
With all these instructions you should be able to run the Axolotec final project with no
problem
