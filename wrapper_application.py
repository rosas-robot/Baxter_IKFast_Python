import ikModule
import numpy as np

ee_gst = [-0.748117, 0.654960, 0.106527, 0.555013, 0.568134, 0.549273, 0.612799, 0.554330, 0.342846, 0.518968, -0.783026, -0.445436]

js = [0.5, 0.5, 0.5, 0.4, 0.5, 0.4, 0.5]

# Sol will be None if ee_gst contains less than 12 elements
# first ip: free_angle, second ip: desired ee_pose
sol = ikModule.compIKs(0.5, ee_gst)
print("Inverse Kinematics Solutions in Python-side\n");
print(sol)

# eePose will be None if js does not contain 7 elements
# eePose is a 3by4 array where rotmat = eePose(1:3, 1:3)
# and posmat = eePose(1:3, 4)
eePose = ikModule.compFK(js)
print("Forward Kinematics Solutions in Python-side\n");
print(eePose)

