from PyQt4 import QtGui, QtCore
from modules.widgets import DragButton, DragDial, DashboardWidget

class Dashboard(QtGui.QGroupBox):
    """ canvas where widgets can be positioned """ 
      
    def __init__(self, parent):
        super(Dashboard, self).__init__(parent)

        self.widgets = list()
        self.initUI()
        
    def initUI(self):

        self.setTitle('Dashboard')
        self.setAcceptDrops(True)

        #TODO: remove test widgets
        self.addWidget(DragButton(self), QtCore.QPoint(100, 65))
        self.addWidget(DragDial(self), QtCore.QPoint(100, 150))
        
    def addWidget(self, widget, position = None):
        """ add widgets to the dashboard """
        if not isinstance(widget, DashboardWidget):
            raise TypeError("The widget you want to add is not a DashboardWidget")
        
        if position == None:
            #TODO: find a free spot and move widget to the free spot
            position = QtCore.QPoint(100, 100)
        
        self.widgets.append(widget)
        widget.move(position)
        widget.show()
        
    def dragEnterEvent(self, e):
        
        e.accept()

    def dropEvent(self, e):

        if isinstance(e.source(), DashboardWidget):
            #dashboard widgets are moved on the dashboard
            e.source().move(e.pos())

            e.setDropAction(QtCore.Qt.MoveAction)
            e.accept()
        else:
            #this happens if a element from the widget list gets dropped
            #TODO: typecheck for elements coming from the toolbox??
            itemDataInstance = e.source().currentItem().data(QtCore.Qt.UserRole).toPyObject()(self)
            self.addWidget(itemDataInstance, e.pos())
        
        