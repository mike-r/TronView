#!/usr/bin/env python

#######################################################################################################################################
#######################################################################################################################################
# Shared globals objects
#

from lib.common.dataship import dataship
from lib import smartdisplay
from lib.common.graphic.growl_manager import GrowlManager
from lib.common.event_manager import EventManager
from lib.common.graphic.edit_TronViewScreenObject import TronViewScreenObject

####################################
## DataShip object
## All input data is stuffed into this dataship object in a "standard format"
## Then dataship object is passed on to different screens for displaying data.
Dataship = dataship.Dataship()


####################################
## Input objects
## Input objects take in external data, process it if needed then
## stuff the data into the dataship object for the screens to use.
## Inputs can be from Serial, Files, wifi, etc...

Inputs = {}

####################################
## SmartDisplay Obect
## This is a helper object that knows the screen size and ratio.
## Makes it easier for screens to write data to the screen without having to 
## know all the details of the screen.
smartdisplay = smartdisplay.SmartDisplay()


####################################
## Screen Obect
## This is the current graphical screen object that is being displayed.
class Screen2d(object):
    def __init__(self):
        self.ScreenObjects: list[TronViewScreenObject] = []
        self.show_FPS = False
        self.name = None
        self.filename = None
        self.loaded_from_template = None

    def clear(self):
        self.ScreenObjects.clear()
        self.filename = None
        self.loaded_from_template = None
        self.name = None

CurrentScreen = Screen2d()

pygamescreen = None

####################################
## Default flight log dir.
## default location where flight logs are saved. Can be overwritten in config file.
DefaultFlightLogDir = "./flightlog/"


####################################
## Default data dir.
## default location where data is saved. Can be overwritten in config file.
DataDir = "./data/"


####################################
## Change History
## This is a global object that is used to store the history of changes to the screen objects while in edit mode.
## This is used for the undo functionality.
Change_history = None

####################################
## Growl Manager
## This is a global object that is used to manage the growl messages.
GrowlManager = GrowlManager()

####################################
## Event Manager
## This is a global object that is used to manage events.
EventManager = EventManager()

####################################
## Active Dropdown
## This is a global reference to the currently active dropdown menu in edit mode
active_dropdown = None

# vi: modeline tabstop=8 expandtab shiftwidth=4 softtabstop=4 syntax=python

