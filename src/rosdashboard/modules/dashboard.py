from python_qt_binding.QtBindingHelper import QT_BINDING, QT_BINDING_VERSION #@UnresolvedImport @UnusedImport
import QtGui #@UnresolvedImport
import QtCore #@UnresolvedImport

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
        
        
    def addWidget(self, widget, position):
        """ add widgets to the dashboard """
        if not isinstance(widget, DashboardWidget):
            raise TypeError("The widget you want to add is not a DashboardWidget: " + str(widget))
        
        self.widgets.append(widget)
        widget.move(position)
        widget.show()
        
        #show subscription dialog when added to set up subscriptions initally
        widget.showSubscriptionDialog()
        
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
        
    def customEvent(self, event):
        self.parent.addWidget(event.widget, event.pos)
        
    def dragEnterEvent(self, e):
        
        #check if the drag comes from a valid item
        if isinstance(e.source(), DashboardWidget):
            self.parent.showRemoveWidgetBar()
            e.acceptProposedAction()
        elif isinstance(e.source(), QtGui.QListWidget):
            e.acceptProposedAction()
        else:
            e.ignore()

    def dropEvent(self, e):

        if isinstance(e.source(), DashboardWidget):
            #dashboard widgets are moved on the dashboard
            e.source().move(e.pos())

            e.setDropAction(QtCore.Qt.MoveAction)
            
            self.parent.hideRemoveWidgetBar()
            e.acceptProposedAction()
        elif isinstance(e.source(), QtGui.QListWidget):
            e.setDropAction(QtCore.Qt.CopyAction)
            e.acceptProposedAction()
            
            #this happens if a element from the widget list gets dropped
            
            #convert data payload
            itemData = e.source().currentItem().data(QtCore.Qt.UserRole)
            if isinstance(itemData, QtCore.QVariant):
                itemData = itemData.toPyObject()
            
            itemDataInstance = itemData(self)
            
            #post event. this should allow the drop to be completed before we add the widget.
            event = QtCore.QEvent(QtCore.QEvent.User)
            event.widget = itemDataInstance
            event.pos = e.pos()
            QtGui.QApplication.postEvent(self, event)
        else:
            e.ignore()
            raise TypeError('The source for this kind of drag element is not allowed: ' + str(e.source()))