""" module for all property classes """

class WidgetProperties():
    """ object to store the key value pairs of a widget """
    def __init__(self):        
        #key value store for the properties.
        self.props = dict()
        
    def getAllProperties(self):
        return self.props
    
    def getKeys(self):
        return self.props.keys()
    
    def getProperty(self, key):
        print 'requested value for ' + str(key)
        return self.props[key]
    
    def setProperty(self, key, value):
        print 'setting ' + str(key) + ' to ' + str(value)
        self.props[key] = value
        
class DashboardProperties():
    """ object to store the properties of all the widgets on the dashboard """
    def __init__(self):
        #props is a dictionary where the key is the widget ID
        self.props = dict()
    
    def getProperties(self, widgetID):
        print 'requested props for widget ' + str(widgetID)
        return self.props[widgetID]
    
    def setProperties(self, widgetID, props):
        print 'setting props for widget ' + str(widgetID) + ' with props ' + str(props)
        self.props[widgetID] = props