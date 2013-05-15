# -*- coding: utf-8 -*-
import sys
import os
import xbmc
import xbmcgui
#import threading

_ = sys.modules[ "__main__" ].__language__
__scriptname__ = sys.modules[ "__main__" ].__scriptname__
__version__ = sys.modules[ "__main__" ].__version__
__settings__ = sys.modules[ "__main__" ].__settings__

# add librarys and resource files into sys path
BASE_RESOURCE_PATH = xbmc.translatePath( os.path.join( os.getcwd(), 'resources' ) )
BASE_LIB_PATH = xbmc.translatePath( os.path.join( os.getcwd(), 'resources', 'lib' ) )
sys.path.append (BASE_RESOURCE_PATH)
sys.path.append (BASE_LIB_PATH)


import openbm

# ACTION definitions
ACTION_PARENT_DIR = 9 
ACTION_PREVIOUS_MENU = 10
ACTION_CLOSE_DIALOG = 51
ACTION_SELECT_ITEM = 7 
ACTION_MOVE_LEFT = 1
ACTION_MOVE_RIGHT = 2
ACTION_MOVE_UP = 3
ACTION_MOVE_DOWN = 4

# MID definitions
MID_BUTTON_REW = 12
MID_BUTTON_FF = 13

# SKIN definitions
MAIN_LABEL = 90
MAIN_IMG = 21
BUTTON_ID_BASE = 9000
ICON_CD = 91
TONE_BUTTON_BASE = 1000

# setup default variables
BASE_PATH = xbmc.translatePath( os.getcwd() )
BASE_RESOURCE_PATH = xbmc.translatePath( os.path.join( os.getcwd(), 'resources' ) )
bmwLogoSmallImg = os.path.join ( BASE_RESOURCE_PATH, "bmw_logo_small.png")

# --------------------------------------------------
# TONE to control tone settings, just a dialog
# --------------------------------------------------
class TONE( xbmcgui.WindowXMLDialog ):

	def __init__( self, *args, **kwargs ):
		pass

	def onInit( self ):
		self.setup_all()
	
	def setup_all( self ):
		pass
		
	# ---------------------
	def setToneState(self, button):
		if  (button >= 0 and button <= 2):
			self.getControl(self.getFocusId()).setPosition(-10, 90 - openbm.midGetRadioToneState(button) * 10)
		elif (button == 3):
			self.getControl(self.getFocusId()).setPosition(90 - openbm.midGetRadioToneState(button) * 10, -10)
	
	# ---------------------
	def onAction(self, action):
		act = action.getId()
		button = self.getFocusId() - TONE_BUTTON_BASE

		if (act == ACTION_MOVE_UP and button >= 0 and button <= 3):
			openbm.midSetRadioToneState(button, +1)	
		elif (act == ACTION_MOVE_DOWN and button >= 0 and button <= 3):
			openbm.midSetRadioToneState(button, -1)
		elif (not act == ACTION_SELECT_ITEM):
			self.onAction1(act)
		
		self.setToneState(button)
		
		pass
	      
	# ---------------------
	# React on action sent from the window system
	# ---------------------    	
	def onAction1(self, action):
		if (action == ACTION_PREVIOUS_MENU) or (action == ACTION_CLOSE_DIALOG) or (action == ACTION_PARENT_DIR):
			self.close()



# --------------------------------------------------
# MID to control the Radio
# --------------------------------------------------
class MID( xbmcgui.WindowXML ):
	
	def __init__( self, *args, **kwargs ):
		pass

	def onInit( self ):
		self.setup_all()

	# ---------------------
	# Disconnect from D-Bus and close this window
	# ---------------------	
	def onStop( self):
		openbm.stop()
		self.close()
	
	# ---------------------
	# Connect to D-Bus and setup all callbacks
	# ---------------------	
	def setup_all( self ):
		self.getControl( MAIN_LABEL ).setLabel( "Radio RDS")
		
		# try to connect to dbus, if fails, then error message
		try:	
			
			openbm.setOnTitleChange(self.onTitleChange)
			openbm.setOnDisconnect(self.onStop)
			openbm.setOnButtonFieldChange(self.onButtonFieldChange)
			openbm.setOnRadioStateChange(self.onRadioState)
			openbm.setOnMessage(self.onIBusMessage)
			openbm.midEnableEmulation()
			openbm.start()
			openbm.midReqUpdateFields()
			
		except openbm.error,e:
			dialog = xbmcgui.Dialog()
			dialog.ok("connection to gateway failed", str(e))	
			xbmc.output(str(e), xbmc.LOGDEBUG)
	
	def onFocus( self, controlId ):
		pass

	# ---------------------
	def onAction(self, action):
		if (not action.getId() == ACTION_SELECT_ITEM):
			self.onAction1(action.getId())
		pass
	      
	# ---------------------
	# React on action sent from the window system
	# ---------------------    	
	def onAction1(self, action):	
		if (action == ACTION_PREVIOUS_MENU) or (action == ACTION_CLOSE_DIALOG):
			self.onStop()
			
		# emulate button press event, when clicking on buttons 1..8
		if (action == ACTION_SELECT_ITEM) and (self.getFocusId() >= BUTTON_ID_BASE):
			openbm.midSendButtonPress(self.getFocusId() - BUTTON_ID_BASE)
		
		# Emulate FF and REW buttons when we showing radio screen
		if (action == ACTION_MOVE_LEFT):
			openbm.midSendButtonPress(MID_BUTTON_REW)
		elif (action == ACTION_MOVE_RIGHT):
			openbm.midSendButtonPress(MID_BUTTON_FF)
					
		pass
	      
	# ---------------------
	def onClick( self, controlId ):
		self.onAction1(ACTION_SELECT_ITEM)   
		pass

	# ---------------------
	def onControl(self, control):
		#self.setFocus(control)
		pass

	# ---------------------
	# Title field changed, react on this
	# ---------------------    	
	def onTitleChange(self, title):
		self.getControl( MAIN_LABEL ).setLabel(title)
		
	# ---------------------
	# Button field changed, update it
	# ---------------------    	
	def onButtonFieldChange(self, buttonLabel, activeButton):
	  
		# set button text
		for _index, _item in enumerate(buttonLabel):
			if (_index >= 8):
				break
				
			# trim whitespaces and replace text with either PROG or CD depending on mode
			item = _item.strip()
			if (item.isdigit() and len(item) == 1):
				if (openbm.midGetRadioState() == 0):
					item = 'PROG %s' % (item)
				elif (openbm.midGetRadioState() == 1):
					item = 'CD %s' % (item)
			
			if (len(item) == 0):
				self.getControl(_index + BUTTON_ID_BASE).setVisible(False)
			else:
				self.getControl(_index + BUTTON_ID_BASE).setVisible(True)
				self.getControl(_index + BUTTON_ID_BASE).setLabel( item )
		
		# set focus to active button
		if (activeButton >= 0 and activeButton < 8):
			self.setFocus( self.getControl(activeButton + BUTTON_ID_BASE) )
			
	# ---------------------
	# Radio State changed, so react on this
	# --------------------- 
	def onRadioState(self, state):
		if (state == 0):
			self.getControl(MAIN_IMG).setImage("radio.jpg")
		if (state == 1):
			self.getControl(MAIN_IMG).setImage("cd.jpg")
		if (state == 2):
			self.onStop()
			#self.getControl(MAIN_IMG).setImage("tape.jpg")
		pass
		    
		
	# ---------------------
	# React on IBus messages
	# ---------------------
	def onIBusMessage(self, src, dst, data):
		if (src == 0xF0 and dst == 0xFF and data[0] == 0x48):
			if (data[1] == 0x04):
				uiTone = TONE( "tone-skin.xml", BASE_PATH )
				uiTone.doModal()
				del uiTone
		pass

