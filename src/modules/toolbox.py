from PyQt4 import QtGui, QtCore

class Toolbox(QtGui.QGroupBox):
    """ widget toolbox to choose widgets from """
    
    def __init__(self, parent):
        super(Toolbox, self).__init__(parent)

        self.initUI()
        
    def initUI(self):

        self.setTitle('Widgets')
        
        self.listView = QtGui.QListView(self)
        
        list_data = [1,2,3,4]
        lm = WidgetListModel(list_data, self)
        self.listView.setModel(lm)
        
        self.layout = QtGui.QVBoxLayout()
        self.layout.addWidget(self.listView)
        self.setLayout(self.layout)
        
        self.setAcceptDrops(True)
        
    def dragEnterEvent(self, e):
        
        e.accept()

    def dropEvent(self, e):

        e.source().move(e.pos())

        e.setDropAction(QtCore.Qt.MoveAction)
        e.accept()
        
class WidgetListModel(QtCore.QAbstractListModel): 
    def __init__(self, data, parent=None): 
        """ data should be a list of DashboardWidgets """
        QtCore.QAbstractListModel.__init__(self, parent) 
        self.listdata = data
 
    def rowCount(self, parent=QtCore.QModelIndex()): 
        return len(self.listdata) 
 
    def data(self, index, role): 
        if index.isValid() and role == QtCore.Qt.DisplayRole:
            return QtCore.QVariant(self.listdata[index.row()])
        else: 
            return QtCore.QVariant()