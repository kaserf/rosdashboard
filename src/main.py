#!/usr/bin/env python
import roslib; roslib.load_manifest('rosdashboard')
import rospy

import sys
from PyQt4 import QtGui

from modules.dashboard import Dashboard
from modules.toolbox import Toolbox

"""
def callback(data, l):
    l.setText(data.data)
    l.resize(l.sizeHint())
    rospy.loginfo(rospy.get_name()+"I heard %s",data.data)
"""
        
        
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
    
    #dashboard.selectionChanged.connect(properties.widgetSelected)
    
    w.setLayout(layout)
    w.show()

    #rospy.Subscriber("chatter", String, callback, l)
    #rospy.spin()
    
    sys.exit(app.exec_())
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
