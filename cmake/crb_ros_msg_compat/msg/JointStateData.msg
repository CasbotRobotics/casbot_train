# JointStateData.msg - joint state/command feedback including active kp/kd

std_msgs/Header header

string[] name
float64[] position
float64[] velocity
float64[] effort
float64[] kp
float64[] kd
