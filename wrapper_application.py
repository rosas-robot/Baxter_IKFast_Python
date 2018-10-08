import ikModule
import numpy as np

# End effector pose to solve IK
# eePose = [r11, r12, r13, p1, r21, r22, r23, p2, r31, r32, r33, p3]
ee_gst = [-0.748117, 0.654960, 0.106527, 0.555013, 0.568134, 0.549273, 0.612799, 0.554330, 0.342846, 0.518968, -0.783026, -0.445436]

# Joint angle vector to solve forward kinematics
# js = [j1, j2, j3, j4, j5, j6, j7]
js = [0.5, 0.5, 0.5, 0.4, 0.5, 0.4, 0.5]

# Sol will be None if ee_gst contains less than 12 elements
# first ip: free_angle, second ip: desired ee_pose
sol = ikModule.compIKs(0.5, ee_gst)
if sol is not None:
	print("Inverse Kinematics Solutions in Python-side\n");
	print(sol)

# eePose will be None if js does not contain 7 elements
# eePose is a 3by4 array where rotmat = eePose(1:3, 1:3)
# and posmat = eePose(1:3, 4)
eePose = ikModule.compFK(js)
if eePose is not None:
	print("Forward Kinematics Solutions in Python-side\n");
	print(eePose)

