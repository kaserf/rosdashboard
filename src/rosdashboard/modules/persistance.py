from python_qt_binding.QtBindingHelper import QT_BINDING, QT_BINDING_VERSION #@UnresolvedImport @UnusedImport
import QtCore #@UnresolvedImport

#import xml.dom.minidom as minidom
import os
import json

#TODO: the widgets should be importet automatically from a plugin folder
from rosdashboard.widgets.dragDial import DragDial
from rosdashboard.widgets.dragKnob import DragKnob
from rosdashboard.widgets.dragCompass import DragCompass
from rosdashboard.widgets.dragThermo import DragThermo
from rosdashboard.widgets.dragString import DragString
from rosdashboard.widgets.dragLed import DragLed

from rosdashboard.modules.props import WidgetProperty

class Persistance(object):
    def __init__(self, dashboard):
        self.dashboard = dashboard
        
    def loadDashboard(self, filename):
        # clear old dashboard
        self.dashboard.clearDashboard()
        
        # get file data
        _file = open(filename,'r')
        filedata = _file.read()
        _file.close()
        
        # parse file data
        name, extension = os.path.splitext(filename)
        if (extension == ".xml"):
            print "load from xml: " + filename
            self.loadFromXML(self, filedata)
        elif (extension == ".json"):
            print "load from json: " + filename
            self.loadFromJSON(filedata)
        else:
            print "I don't know how to handle file extension " + extension + "; I only know .json and .xml"
            
    def saveDashboard(self, filename):
        # route to save xml or save json
        name, extension = os.path.splitext(filename)
        if (extension == ".xml"):
            print "save to xml: " + filename
            data = self.saveToJSON()
        elif (extension == ".json"):
            print "save to json: " + filename
            data = self.saveToJSON()
        else:
            print "I don't know how to handle file extension " + extension + "; I only know .json and .xml"
            return
            
        #write to file
        _file = open(filename,'w')
        _file.write(data)
        _file.close()
                    
    def loadFromXML(self, filedata):
        print "Not implemented yet"
        
        """

        doc = minidom.parseString(filedata)
        #node = doc.documentElement
        widgets = doc.getElementsByTagName("widget")
        
        for widget in widgets:
            nodes = widget.getElementsByTagName("name")[0].childNodes
            for node in nodes:
                if node.nodeType == node.TEXT_NODE:
                    print "reading widget: " + node.data
        """
        
    def loadFromJSON(self, filedata):
        data = json.loads(filedata)
        for widget in data["widgets"]:
            
            # load properties
            props = dict()
            for prop in widget["properties"]:
                propName = str(prop["name"])
                propType = str(prop["type"])
                propValue = prop["value"]
                props[propName] = WidgetProperty(propType, propValue)
            
            # load subscription
            topic = widget["subscription"]["topic"]
            datafield = widget["subscription"]["datafield"]
            regex = widget["subscription"]["regex"]
            
            # load geometry
            height = widget["height"]
            width = widget["width"]
            
            # load widget class
            typename = str(widget["type"])
            constructor = globals()[typename]
            instance = constructor()
            
            instance.setProperties(props)
            instance.setSubscription(topic, datafield, regex)
            instance.setWidgetName(widget["name"])
            instance.resize(width, height)
            
            x = widget["posX"]
            y = widget["posY"]
            position = QtCore.QPoint(x, y)

            self.dashboard.addWidget(instance, position, False)
            
    def saveToJSON(self):
        widgets = list()
        for widget in self.dashboard.widgets:
            widgetType = widget.__class__.__name__
            widgetName = widget.getWidgetName()
            widgetX = widget.x()
            widgetY = widget.y()
            widgetHeight = widget.height()
            widgetWidth = widget.width()
            subscription = {"topic": widget.topic, "datafield": widget.datafield, "regex": widget.regex}
            
            #fill properties
            props = list()
            for propKey, prop  in widget.getProperties().iteritems():
                props.append({"name": propKey, "type": prop.propertyType, "value": prop.value})
                
            widgets.append({"type": widgetType, "name": widgetName, "height": widgetHeight,
                            "width": widgetWidth, "posX": widgetX, "posY":widgetY,
                            "subscription": subscription, "properties": props})
            
        return json.dumps({"widgets": widgets})
