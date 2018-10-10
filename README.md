# Baxter_IKFast_Python
C Extension for Python of IKFast Module for baxter Robot

This repository wraps Forward-Kinematics and Inverse-Kinematics functions of custom-IKFast solver for Baxter robot. The .cpp functions are **computeIk** and **computeFk** respectively and can be found in the file named *baxter_left_arm_solver.cpp* file. The python interface of these two functions are written in core **C** language and can be found in file named *wrapper.cpp*.
