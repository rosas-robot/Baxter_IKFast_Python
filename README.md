# Baxter_IKFast_Python
C Extension for Python of IKFast Module for baxter Robot

This repository wraps Forward-Kinematics and Inverse-Kinematics functions of custom-IKFast solver for Baxter robot. The .cpp functions are **computeIk** and **computeFk** respectively and can be found in the file named **baxter_left_arm_ikfast_solver.cpp** file. The python interface of these two functions are written in core **C** language and can be found in file named **myIKFastwrap.cpp**. The following shows the commands used to generate the shared object (.so) file named as **ikModule.so**.

```$ g++ -DNDEBUG -Wall -Wstrict-prototypes -fPIC -I/home/nobug-ros/anaconda3/include/python3.6m -c baxter_left_ik_solver_wrap.cpp -o ikModule.o -llapack```

```$ g++ -shared ikModule.o -o ikModule.so -llapack```

Once you are done with generating the $.so$ file, copy and paste it to site-package folder of you python maintainer. 
