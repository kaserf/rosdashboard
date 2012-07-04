#!/usr/bin/env python
import roslib; roslib.load_manifest('rosdashboard')
import rospy

import sys
from PyQt4 import QtGui

from modules.dashboard import Dashboard
from modules.toolbox import Toolbox        
        
def main():
    
    rospy.init_node('dashboard', anonymous=True)
    
    app = QtGui.QApplication(sys.argv)
    
    w = QtGui.QWidget()
    w.resize(800, 600)
    w.move(100, 100)
    w.setWindowTitle('Dashboard')
    
    layout = QtGui.QHBoxLayout()
    
    toolbox = Toolbox(w)
    layout.addWidget(toolbox)
    
    dashboard = Dashboard(w)
    layout.addWidget(dashboard)
    
    layout.setStretch(0, 20)
    layout.setStretch(1, 80)
    
    w.setLayout(layout)
    w.show()
    
    sys.exit(app.exec_())
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass