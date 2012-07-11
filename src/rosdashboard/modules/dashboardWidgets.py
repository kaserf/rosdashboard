from PyQt4 import QtGui, QtCore
from rosdashboard.modules.props import WidgetPropertiesDialog, WidgetRenameDialog,\
    WidgetSubscriptionDialog
import rospy
import rostopic

class DashboardWidget(QtGui.QGroupBox):
    """ base class for draggable widgets """
    
    def __init__(self, parent):
        super(DashboardWidget, self).__init__(parent)
        
        self.setTitle('noname')
        
        self.topic = "/your/topic/here"
        self.datafield = "datafield"
        self.subscriber = None
        
        self.props = dict()
        self.initProps()
        
        self.initContextMenu()
        
        self.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.customContextMenuRequested.connect(self.ctxMenuRequested)
        
    def initContextMenu(self):
        """ this method sets up the context menu of the widget. per default it allows
            you to rename a widget and setup the subscription for the widget. if there
            are properties set in props (see initProps()) then it also shows the
            default configuration dialog for properties. """
            
        self.ctxMenu = QtGui.QMenu(self)
        
        self.renameAction = QtGui.QAction('Rename', self)
        self.renameAction.triggered.connect(self.showRenameDialog)
        self.ctxMenu.addAction(self.renameAction)
        
        self.subscriptionAction = QtGui.QAction('Subscription', self)
        self.subscriptionAction.triggered.connect(self.showSubscriptionDialog)
        self.ctxMenu.addAction(self.subscriptionAction)
        
        # only add properties dialog if there are any props to set.
        if len(self.props) != 0:
            self.propertiesAction = QtGui.QAction('Properties', self)
            self.propertiesAction.triggered.connect(self.showConfigDialog)        
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
        dialog = WidgetRenameDialog(self, self._renameDialogCallback, self.title())
        dialog.exec_()
        
    def _renameDialogCallback(self, value):
        """ this method will be hooked to the rename dialog accept signal.
            It should be overwritten in a subclass if special functionality
            is needed. """
        if value != '':
            self.setTitle(value)
            
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
        
    def showSubscriptionDialog(self):
        """ shows the default subscription dialog.
            If custom properties are needed it should be overwritten in the subclass. """
        dialog = WidgetSubscriptionDialog(self, self._subscriptionDialogCallback, self.topic, self.datafield)
        dialog.exec_()
        
    def _subscriptionDialogCallback(self, newTopic, newDatafield):
        if (newTopic != ""):
            self.topic = newTopic
        if (newDatafield != ""):
            self.datafield = newDatafield
            
        self.setupSubscription()
        
    def teardownSubscription(self):
        #tear down previous subscriber
        if (self.subscriber != None):
            print "unregister subscriber: " + self.subscriber.name
            self.subscriber.unregister()
            
    def setupSubscription(self):
        self.teardownSubscription()
        
        # TODO: wrap blocking call to make it async
        dataClass = rostopic.get_topic_class(self.topic, blocking=False)[0]
        if dataClass:
            self.subscriber = rospy.Subscriber(self.topic, dataClass, self.subscriptionCallback)
        else:
            print "ERROR: could not find data class for this topic: " + self.topic
        
    def subscriptionCallback(self, data):
        value = getattr(data, self.datafield)
        self.updateValue(value)
        
    def updateValue(self, value):
        """
        needs to be overwritten in the subclass to update the
        value field of whatever widget is displayed.
        """
        print "update value: " + str(value)
    
    def getProperties(self):
        """ returns a dictionary of the properties for this widget """
        return self.props
