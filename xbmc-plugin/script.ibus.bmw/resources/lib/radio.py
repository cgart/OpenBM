# -*- coding: utf-8 -*-
import sys
import os
import xbmc
import xbmcgui

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


# --------------------------------------------------
# Radio control window
# --------------------------------------------------
class GUI( xbmcgui.WindowXML ):
			
	# ---------------------	
	def __init__( self, *args, **kwargs ):
		self.radioState = 0
		pass
		
	# ---------------------	
	def setRadioState(self, state):
		self.radioState = state
		pass

	# ---------------------	
	def onInit( self ):
		self.setup_all()

	# ---------------------	
	def onStop( self):
		self.close()

	# ---------------------	
	def onFocus( self, controlId ):
		pass
	
	# ---------------------	
	def setup_all( self ):
		self.getControl( MAIN_LABEL ).setLabel( "Radio RDS")
		
		openbm.setOnTitleChange(self.onTitleChange)
		openbm.setOnButtonFieldChange(self.onButtonFieldChange)
		openbm.setOnRadioStateChange(self.onRadioState)
		
		self.onRadioState(self.radioState)
		openbm.midReqUpdateFields()
		
		
	# ---------------------
	def onAction(self, action):
		if (not action.getId() == ACTION_SELECT_ITEM):
			self.onAction1(action.getId())
		pass
	      
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
	def onTitleChange(self, title):
		self.getControl( MAIN_LABEL ).setLabel(title)
		
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
	def onRadioState(self, state):
		if (state == 0):
			self.getControl(MAIN_IMG).setImage("radio.jpg")
		if (state == 1):
			self.getControl(MAIN_IMG).setImage("cd.jpg")
		if (state == 2):
			self.onStop()
		pass
		    

