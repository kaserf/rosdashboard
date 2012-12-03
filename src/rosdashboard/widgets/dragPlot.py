from python_qt_binding.QtBindingHelper import QT_BINDING, QT_BINDING_VERSION #@UnresolvedImport @UnusedImport
import QtGui #@UnresolvedImport
import QtCore #@UnresolvedImport

from rosdashboard.modules.props import WidgetProperty
from rosdashboard.modules.dashboardWidgets import DashboardWidget
from PyQt4.Qwt5 import Qwt
from PyQt4.Qwt5.anynumpy import *

# data plot class from http://pyqwt.sourceforge.net/examples/DataDemo.py.html
class DataPlot(Qwt.QwtPlot):

    def __init__(self, *args):
        Qwt.QwtPlot.__init__(self, *args)

        self.setCanvasBackground(QtCore.Qt.white)
        self.alignScales()

        # Initialize data
        self.x = arange(0.0, 100.1, 0.5)
        self.y = zeros(len(self.x), Float)

        self.curveL = Qwt.QwtPlotCurve("Data Moving Left")
        self.curveL.attach(self)

        mY = Qwt.QwtPlotMarker()
        mY.setLabelAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignTop)
        mY.setLineStyle(Qwt.QwtPlotMarker.HLine)
        mY.setYValue(0.0)
        mY.attach(self)

        self.setAxisTitle(Qwt.QwtPlot.xBottom, "Time (seconds)")
        self.setAxisTitle(Qwt.QwtPlot.yLeft, "Values")

    # __init__()

    def alignScales(self):
        self.canvas().setFrameStyle(QtGui.QFrame.Box | QtGui.QFrame.Plain)
        self.canvas().setLineWidth(1)
        for i in range(Qwt.QwtPlot.axisCnt):
            scaleWidget = self.axisWidget(i)
            if scaleWidget:
                scaleWidget.setMargin(0)
            scaleDraw = self.axisScaleDraw(i)
            if scaleDraw:
                scaleDraw.enableComponent(
                    Qwt.QwtAbstractScaleDraw.Backbone, False)

    # alignScales()
    
    def timerEvent(self, e):
        #repeat the current value
        self.addValue(self.y[-1])

    # timerEvent()

    def addValue(self, value):
      #shift array left and add new value
      self.y = concatenate((self.y[1:], self.y[:1]), 1)
      self.y[-1] = value

      self.curveL.setData(self.x, self.y)

      self.replot()
    # addValue()

# class DataPlot


class DragPlot(DashboardWidget):
    """ draggable plot """
    TITLE = 'Plot title'
    RATE = 'Refresh rate (ms)'
    
    def __init__(self, parent = None):
        super(DragPlot, self).__init__(parent)
        self.setTitle('DragPlot')
        
        self.currentTimerId = 0
        
        self.initUI()
        
    def initUI(self):
        self.layout = QtGui.QVBoxLayout()
        self.qwtPlot = DataPlot()
        
        self.layout.addWidget(self.qwtPlot)
        
        #initial size
        self.resize(400,300)
        
        #update widget according to properties
        self.updateWidget()
        
        self.setLayout(self.layout)
        
    def initProps(self):
        self.props[self.TITLE] = WidgetProperty('text', 'Plot title')
        self.props[self.RATE] = WidgetProperty('numeric', 500)
    
    def propertiesDialogAccepted(self):
        self.updateWidget()

    def updateWidget(self):
        #update the widget properties
        self.qwtPlot.setTitle(self.props[self.TITLE].value)

        if (self.currentTimerId != 0):
          self.qwtPlot.killTimer(self.currentTimerId)
          self.currentTimerId = 0

        self.currentTimerId = self.qwtPlot.startTimer(self.props[self.RATE].value)
        
    def updateValue(self, value):
        self.qwtPlot.addValue(float(value))
