from PyQt4 import QtGui, QtCore
from rosdashboard.modules.dashboardWidgets import DashboardWidget

class Dashboard(QtGui.QWidget):
    """ canvas where widgets can be positioned """ 
      
    def __init__(self, parent):
        super(Dashboard, self).__init__(parent)

        #The dashboard keeps track of the widgets that are positioned in the widget container
        self.widgets = list()
        self.initUI()
        
    def initUI(self):

        self.title = QtGui.QLabel('Dashboard', self)
        self.title.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        
        self.widgetContainer = WidgetContainer(self)
        self.widgetContainer.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        
        # set up a bar to drop widgets to remove them
        self.removeWidgetBar = RemoveWidgetBar(self)
        self.removeWidgetBar.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        #hide by default
        self.removeWidgetBar.hide()
        
        self.layout = QtGui.QVBoxLayout()
        self.layout.addWidget(self.title)
        self.layout.addWidget(self.widgetContainer)
        self.layout.addWidget(self.removeWidgetBar)
        self.setLayout(self.layout)
        
        
    def addWidget(self, widget, position = None):
        """ add widgets to the dashboard """
        if not isinstance(widget, DashboardWidget):
            raise TypeError("The widget you want to add is not a DashboardWidget: " + str(widget))
        
        if position == None:
            #TODO: find a free spot and move widget to the free spot
            position = QtCore.QPoint(100, 100)
        
        self.widgets.append(widget)
        widget.move(position)
        widget.show()
        
    def removeWidget(self, widget):
        """ remove a widget from the dashboard """
        if not isinstance(widget, DashboardWidget):
            raise TypeError("The widget you want to remove is not a DashboardWidget: " + str(widget))
        
        widget.hide()
        widget.teardownSubscription()
        self.widgets.remove(widget)
        
    def showRemoveWidgetBar(self):
        self.removeWidgetBar.show()
    
    def hideRemoveWidgetBar(self):
        self.removeWidgetBar.hide()
    
class RemoveWidgetBar(QtGui.QStatusBar):
    def __init__(self, parent):
        super(RemoveWidgetBar, self).__init__(parent)
        
        self.parent = parent
        self.initUI()
    
    def initUI(self):
        self.setSizeGripEnabled(False)
        self.dropLabel = QtGui.QLabel()
        self.dropLabel.setText("<font size= '6'>Drag here to delete</font>")
        self.dropLabel.setStyleSheet("QLabel { background-color: orangered;}")
        self.dropLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.insertWidget(0, self.dropLabel, 1)
        self.setAcceptDrops(True)
        
    def dragEnterEvent(self, e):
        
        #check if the drag comes from a dashboard item
        if isinstance(e.source(), DashboardWidget):
            #TODO: show hover effect
            e.accept()
        else:
            e.ignore()

    def dropEvent(self, e):

        if isinstance(e.source(), DashboardWidget):
            self.parent.removeWidget(e.source())
            e.accept()
        else:
            e.ignore()
            raise TypeError('The source for this kind of drag element is not allowed: ' + str(e.source()))
        
        self.parent.hideRemoveWidgetBar()

class WidgetContainer(QtGui.QFrame):
    def __init__(self, parent):
        super(WidgetContainer, self).__init__(parent)
        
        self.parent = parent
        self.initUI()
        
    def initUI(self):
        self.setFrameStyle(QtGui.QFrame.Raised | QtGui.QFrame.StyledPanel)
        self.setAcceptDrops(True)
        
    def dragEnterEvent(self, e):
        
        #check if the drag comes from a valid item
        if isinstance(e.source(), DashboardWidget):
            self.parent.showRemoveWidgetBar()
            e.accept()
        elif isinstance(e.source(), QtGui.QListWidget):
            e.accept()
        else:
            e.ignore()

    def dropEvent(self, e):

        if isinstance(e.source(), DashboardWidget):
            #dashboard widgets are moved on the dashboard
            e.source().move(e.pos())

            e.setDropAction(QtCore.Qt.MoveAction)
            
            self.parent.hideRemoveWidgetBar()
            e.accept()
        elif isinstance(e.source(), QtGui.QListWidget):
            #this happens if a element from the widget list gets dropped
            itemDataInstance = e.source().currentItem().data(QtCore.Qt.UserRole).toPyObject()(self)
            self.parent.addWidget(itemDataInstance, e.pos())
            
            e.accept()
        else:
            e.ignore()
            raise TypeError('The source for this kind of drag element is not allowed: ' + str(e.source()))
