#!/usr/bin/env python
import roslib; roslib.load_manifest('rosdashboard')
import rospy

import sys

from python_qt_binding.QtBindingHelper import QT_BINDING, QT_BINDING_VERSION #@UnresolvedImport @UnusedImport
import QtGui #@UnresolvedImport

from modules.dashboard import Dashboard
from modules.toolbox import Toolbox        
from modules.persistance import Persistance
        
class ROSDashboardMain(QtGui.QMainWindow):
    def __init__(self):
        super(ROSDashboardMain, self).__init__()

        self.toolbox = Toolbox(self)
        self.dashboard = Dashboard(self)
        
        self.persistance = Persistance(self.dashboard)
        
        self.initUi()
        
    def initUi(self):
        saveAction = QtGui.QAction('Save dashboard', self)
        saveAction.setStatusTip('Save the current dashboard to file')
        saveAction.triggered.connect(self.saveActionTriggered)
        
        loadAction = QtGui.QAction('Load dashboard', self)
        loadAction.setStatusTip('Load a dashboard from file')
        loadAction.triggered.connect(self.loadActionTriggered)
        
        exitAction = QtGui.QAction('Exit', self)
        exitAction.setStatusTip('Exit rosdashboard')
        exitAction.triggered.connect(QtGui.qApp.quit)
        
        fileMenu = self.menuBar().addMenu('File')
        fileMenu.addAction(saveAction)
        fileMenu.addAction(loadAction)
        fileMenu.addSeparator()
        fileMenu.addAction(exitAction)
        
        self.resize(800, 600)
        self.move(100, 100)
        self.setWindowTitle('Dashboard')
        
        centralWidget = QtGui.QWidget()
        layout = QtGui.QHBoxLayout()
        
        layout.addWidget(self.toolbox)
        layout.addWidget(self.dashboard)
        
        layout.setStretch(0, 20)
        layout.setStretch(1, 80)
        
        centralWidget.setLayout(layout)
        
        self.setCentralWidget(centralWidget)
        self.show()
        
    def saveActionTriggered(self):
        print "save dashboard"
        
    def loadActionTriggered(self):
        print "load dashboard"
        self.persistance.loadDashboard("/home/felix/projects/ros-workspace/rosdashboard/res/test_config.json")
        
def main():
    
    rospy.init_node('rosdashboard', anonymous=True)
    
    app = QtGui.QApplication(sys.argv)
    dashboardMain = ROSDashboardMain()
    sys.exit(app.exec_())
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
