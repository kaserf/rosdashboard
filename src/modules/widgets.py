
from turtlesim.msg import Pose
from PyQt4 import QtGui, QtCore
import rospy
from modules.props import WidgetProperty, WidgetPropertiesDialog,\
    WidgetRenameDialog

class DashboardWidget(QtGui.QGroupBox):
    """ base class for draggable widgets """
    
    def __init__(self, parent):
        super(DashboardWidget, self).__init__(parent)
        
        self.setTitle('noname')
        
        self.initContextMenu()
        
        self.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.customContextMenuRequested.connect(self.ctxMenuRequested)
        
        self.props = dict()
        self.initProps()
        
    def initContextMenu(self):
        """ this method sets up the context menu of the widget. per default it allows
            you to rename a widget and open the default configuration dialog. """
        # Actions
        self.renameAction = QtGui.QAction('Rename', self)
        self.renameAction.triggered.connect(self.showRenameDialog)
        
        self.propertiesAction = QtGui.QAction('Properties', self)
        self.propertiesAction.triggered.connect(self.showConfigDialog)
        
        # Context Menu
        self.ctxMenu = QtGui.QMenu(self)
        self.ctxMenu.addAction(self.renameAction)
        self.ctxMenu.addAction(self.propertiesAction)
        
    def ctxMenuRequested(self, point):
        self.ctxMenu.exec_(self.mapToGlobal(point))
    
    def mouseMoveEvent(self, e):

        mimeData = QtCore.QMimeData()

        drag = QtGui.QDrag(self)
        drag.setMimeData(mimeData)
        drag.setHotSpot(e.pos() - self.rect().topLeft())

        dropAction = drag.start(QtCore.Qt.MoveAction)
    
    def initProps(self):
        """ this method should be overwritten in the subclass
            if properties are needed. Examplecode to add properties: 
            self.props[name] = WidgetProperty(name, type, value) """
        raise NotImplementedError('initProps must be implemented in a subclass!')


    def showRenameDialog(self):
        """ shows the default renaming dialog. """
        dialog = WidgetRenameDialog(self, self.title())
        dialog.exec_()
            
    def showConfigDialog(self):
        """ shows the default properties dialog generated from self.props.
            If custom properties are needed it should be overwritten in the subclass. """
        dialog = WidgetPropertiesDialog(self, self.props)
        dialog.accepted.connect(self.propertiesDialogAccepted)
        dialog.exec_()
        
    def propertiesDialogAccepted(self):
        """ this method will be hooked to the properties dialog accept signal.
            It should be overwritten in a subclass if special functionality
            (e.g. updating the widget) is needed. """
        pass
    
    def getProperties(self):
        """ returns a dictionary of the properties for this widget """
        return self.props
       
   
class DragDial(DashboardWidget):
    """ draggable dial and lcd display """
    MIN = 'minimum'
    MAX = 'maximum'
    DATASOURCE = 'datasource'
    DATAFIELD = 'datafield'
    
    def __init__(self, parent):
        super(DragDial, self).__init__(parent)
        self.setTitle('DragDial')
        self.initUI()
        self.initSubscriptions()
        
    def initUI(self):
        self.layout = QtGui.QVBoxLayout()
        self.dial = QtGui.QDial(self)
        self.dial.setSliderPosition(0)
        self.dial.setDisabled(True)
        
        self.lcd = QtGui.QLCDNumber(self)
        self.lcd.setSegmentStyle(QtGui.QLCDNumber.Flat)
        self.dial.valueChanged.connect(self.lcd.display)
        
        self.layout.addWidget(self.dial)
        self.layout.addWidget(self.lcd)
        
        #update widget according to properties
        self.updateWidget()
        
        self.setLayout(self.layout)
        
    def initProps(self):
        self.props[self.MIN] = WidgetProperty('numeric', -2)
        self.props[self.MAX] = WidgetProperty('numeric', 2)
        self.props[self.DATASOURCE] = WidgetProperty('text', '/turtle1/pose')
        self.props[self.DATAFIELD] = WidgetProperty('text', 'linear_velocity')
    
    def propertiesDialogAccepted(self):
        self.updateWidget()
        
        #re-setup the subscription
        self.subscriber.unregister()
        self.initSubscriptions()
        
    def updateWidget(self):
        #update the widget properties
        self.dial.setMinimum(self.props[self.MIN].value)
        self.dial.setMaximum(self.props[self.MAX].value)
        
    def initSubscriptions(self):
        #FIXME: the cast to string is a workaround because subscriber only accepts python strings and not QStrings
        #TODO: inspect type of message? go down to low level messaging?
        self.subscriber = rospy.Subscriber(str(self.props[self.DATASOURCE].value), Pose, self.subscriptionCallback)
        #self.subscriber = rospy.Subscriber(str(self.props[self.DATASOURCE].value), self.subscriptionCallback)
        
    def subscriptionCallback(self, data):
        #FIXME: remove cast to string
        #print data.__slots__
        #print data._slot_types
        datafield = getattr(data, str(self.props[self.DATAFIELD].value))
        self.dial.setValue(int(datafield))
        
class DragButton(DashboardWidget):
    """ drag button wrapper """
    def __init__(self, parent):
        super(DragButton, self).__init__(parent)
        self.setTitle('DragButton')
        
        self.initUI("Button")
        
    def initProps(self):
        
        self.props['datasource'] = WidgetProperty('text', '/turtle1/pose')
        self.props['datafield'] = WidgetProperty('text', 'linear_velocity')
        self.props['numeric'] = WidgetProperty('numeric', 17)
        self.props['float'] = WidgetProperty('float', 17.9)
        self.props['buttonText'] = WidgetProperty('text', "Push Me!")
        #FIXME: default should be set from the constructor param title
        
    def initUI(self, title):
        
        self.button = QtGui.QPushButton(title, self)
        self.button.setDisabled(True)
        #self.button.clicked.connect(self.buttonClicked)
        
        self.layout = QtGui.QVBoxLayout()
        self.layout.addWidget(self.button)
        
        self.setLayout(self.layout)
        
    def buttonClicked(self):
        #if event.button() == QtCore.Qt.LeftButton:
        print 'button clicked'
    
    def propertiesDialogAccepted(self):
        self.button.setText(self.props['buttonText'].value)