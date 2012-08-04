#!/usr/bin/env python
import roslib; roslib.load_manifest('rosdashboard')
import rospy

import sys

from python_qt_binding.QtBindingHelper import QT_BINDING, QT_BINDING_VERSION #@UnresolvedImport @UnusedImport
import QtGui #@UnresolvedImport

from modules.dashboard import Dashboard
from modules.toolbox import Toolbox        
        
class ROSDashboardMain(QtGui.QMainWindow):
    def __init__(self):
        super(ROSDashboardMain, self).__init__()

        self.initUi()
        
    def initUi(self):
        saveAction = QtGui.QAction('Save dashboard', self)
        saveAction.setStatusTip('Save the current dashboard to file')
        #self.saveAction.triggered.connect(self.showRenameDialog)
        #self.ctxMenu.addAction(self.renameAction)
        
        loadAction = QtGui.QAction('Load dashboard', self)
        loadAction.setStatusTip('Load a dashboard from file')
        #self.loadAction.triggered.connect(self.showSubscriptionDialog)
        #self.ctxMenu.addAction(self.subscriptionAction)
        
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
        
        toolbox = Toolbox(self)
        layout.addWidget(toolbox)
        
        dashboard = Dashboard(self)
        layout.addWidget(dashboard)
        
        layout.setStretch(0, 20)
        layout.setStretch(1, 80)
        
        centralWidget.setLayout(layout)
        
        self.setCentralWidget(centralWidget)
        self.show()
        
    def saveActionTriggered(self):
        print "save dashboard"
        
    def loadActionTriggered(self):
        print "load dashboard"
        
def main():
    
    rospy.init_node('rosdashboard', anonymous=True)
    
    app = QtGui.QApplication(sys.argv)
    dashboardMain = ROSDashboardMain()
    sys.exit(app.exec_())
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
