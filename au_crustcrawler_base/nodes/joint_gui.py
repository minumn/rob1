#!/usr/bin/env python
from quickui.QuickUi import *

m = gui("AU Crustcrawler joint command","vertical",
        group("Joints","vertical",
              *iterate( ros_slider, ("Joint{0}","/joint{1}/command","std_msgs/Float64",".data",(-1.8,1.8),0.0), [["1","1"], ["2",2],["3",3],["4",4]] )
              ),
		ros_slider("Gripper","/gripper/command","std_msgs/Float64",".data",(-0.5,1.1),0.0)
        )

run(m,debug=True)
