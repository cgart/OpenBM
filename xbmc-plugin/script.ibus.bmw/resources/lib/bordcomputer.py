# -*- coding: utf-8 -*-
import sys
import os
import xbmc
import xbmcgui
import threading

# add librarys and resource files into sys path
BASE_RESOURCE_PATH = xbmc.translatePath( os.path.join( os.getcwd(), 'resources' ) )
BASE_LIB_PATH = xbmc.translatePath( os.path.join( os.getcwd(), 'resources', 'lib' ) )
sys.path.append (BASE_RESOURCE_PATH)
sys.path.append (BASE_LIB_PATH)

import openbm
import repeattimer
import tts

# ACTION definitions
ACTION_PARENT_DIR = 9 
ACTION_PREVIOUS_MENU = 10
ACTION_CLOSE_DIALOG = 51
ACTION_SELECT_ITEM = 7 
ACTION_MOVE_LEFT = 1
ACTION_MOVE_RIGHT = 2
ACTION_MOVE_UP = 3
ACTION_MOVE_DOWN = 4

# GUI definitions
OBC_BUTTON_BASE = 100
OBC_LIMIT_LOGO_ID = 1060

MODE_BC, MODE_INFO, MODE_CTRL = range(3)
MODE = [200, 201, 202]

# Label-Text Definitions
LBL_NONE = ''
LBL_RESET = 'Reset'
LBL_ACTIVATE = 'Activate'
LBL_DEACTIVATE = 'Deactivate'
LBL_SET = 'Set'

# --------------------------------------------------
# Bordcomputer set value
# --------------------------------------------------
class SetValueDialog( xbmcgui.WindowXMLDialog ):

	def __init__( self, *args, **kwargs ):
		self.canceled = False
		pass
		
	def setLabel(self, label):
		self.Label = label
		pass
		
	def setMaxLen(self, length):
		self.Maxlen = length
		pass
		
	# ---------------------	
	def onInit( self ):
		self.getControl(110).setEnabled(False)
		pass
	def onFocus( self, controlId ):
		pass
	def onClick( self, controlId ):
		pass

	# ---------------------
	def onAction(self, action):
		act = action.getId()
		
		# close window	
		if (act == ACTION_PREVIOUS_MENU) or (act == ACTION_CLOSE_DIALOG):
			self.close()
			return
			
		# item selected
		if (act == ACTION_SELECT_ITEM):
			if (self.getFocusId() == 11):
				self.canceled = False
				self.close()
			elif (self.getFocusId() == 12):
				self.canceled = True
				self.close()
			else:
				btn = (self.getFocusId() - 100) % 10
				stl = self.Label.getLabel()
				
				if (len(stl) < self.Maxlen):
					self.getControl(110).setEnabled(True)
					self.Label.setLabel(stl + str(btn))
					
				if (len(self.Label.getLabel()) >= self.Maxlen):
					self.setFocus(self.getControl(11))
					for i in range(101,111):
						self.getControl(i).setEnabled(False)
		
			
			
# --------------------------------------------------
# Bordcomputer control window
# --------------------------------------------------
class GUI( xbmcgui.WindowXML ):
			
	# ---------------------	
	def __init__( self, *args, **kwargs ):
		self.obcMode = MODE_BC
		self.selectedButton = -1
		self.selectedSubButton = 0
		self.nextUpdateLabel = 0
		self.updateTimer = repeattimer.RepeatTimer(0.1, self.update)
		
		# OBC Data Label Data
		LABEL_Consumption1 = [10, 0x04, self.onConsumption1Value, [self.resetData], 0, False, "consumption one"]
		LABEL_Consumption2 = [11, 0x05, self.onConsumption2Value, [self.resetData], 0, False, "consumption two"]
		LABEL_Range = [12, 0x06, self.onRangeValue, [], 0, False, "range"]
		LABEL_Distance = [13, 0x07, self.onDistanceValue, [self.resetData, self.setData], 4, False, "distance"]
		LABEL_Arrival = [14, 0x08, self.onArrivalValue, [], 0, False, "arrival time"]
		LABEL_AverageSpeed = [15, 0x0A, self.onAverageSpeedValue, [self.resetData], 0, False, "aaverage speed"]
		LABEL_SpeedLimit = [16, 0x09, self.onSpeedLimitValue, [self.activateData, self.resetData, self.setData], 3, False, "speed limit"]
		LABEL_OutsideTemperature = [17, 0x03, self.onOutsideTemperatureValue, [], 0, False, "outside temperature"]

		self.LabelList = [LABEL_Consumption1, \
				  LABEL_Consumption2, \
				  LABEL_Range, \
				  LABEL_Distance, \
				  LABEL_Arrival, \
				  LABEL_AverageSpeed, \
				  LABEL_SpeedLimit, \
				  LABEL_OutsideTemperature]

		pass

	# --------------------
	# reset data of the obc
	def resetData(self, value, push = False):
		if (push == False):
			return LBL_RESET
			
		tts.sayAsync(("%s reset")%self.LabelList[value][6])
		
		openbm.bcResetOBCValue(self.LabelList[value][1])
		pass
				
	# --------------------
	# activate data of the obc (i.e. limit))
	def activateData(self, value, push = False):
		if (push == False):
			if (self.LabelList[value][5] == True):
				return LBL_DEACTIVATE
			else:
				return LBL_ACTIVATE

		if (self.LabelList[value][5] == True):
			tts.sayAsync(("%s disabled")%self.LabelList[value][6])
		
			openbm.bcDisableOBCValue(self.LabelList[value][1])
			openbm.bcReqOBCState()		
		else:
			tts.sayAsync(("%s activated")%self.LabelList[value][6])
		
			openbm.bcEnableOBCValue(self.LabelList[value][1])
			openbm.bcReqOBCState()
		pass
		
	# --------------------
	# set data of the obc (i.e. limit, distance))
	def setData(self, value, push = False):
		if (push == False):
			return LBL_SET
			
		# what is the current value
		old = self.getControl(self.LabelList[value][0]).getLabel()
		
		# show dialog to set a value
		dialog = SetValueDialog("bc-setvalue.xml", os.getcwd())
		dialog.setLabel(self.getControl(self.LabelList[value][0]))
		dialog.setMaxLen(self.LabelList[value][4])
		dialog.doModal()
		
		# check if user liked to reset the value, then just restore old value
		if (dialog.canceled == True):
			self.getControl(self.LabelList[value][0]).setLabel(old)
		else:
			tts.sayAsync( ("%s set to %s")%(self.LabelList[value][6], self.getControl(self.LabelList[value][0]).getLabel()) )
			
			# set new value
			openbm.bcSetOBCValue(self.LabelList[value][1], int(self.getControl(self.LabelList[value][0]).getLabel()))
			self.getControl(self.LabelList[value][0]).setLabel("")
			
		
		# request of the value update
		openbm.bcReqOBCValue(self.LabelList[value][1])
			
		# release memory	
		del dialog
		
		pass
		
	# ---------------------	
	def onInit( self ):
		self.setup_all()

	# ---------------------	
	def onStop( self):
		self.close()
		self.updateTimer.cancel()

	# ---------------------	
	def onFocus( self, controlId ):
		pass

	# --------------------
	def update(self):

		if ((not self.nextUpdateLabel == self.selectedButton) and (len(self.getControl(self.LabelList[self.nextUpdateLabel][0]).getLabel()) == 0)):
			openbm.bcReqOBCValue(self.LabelList[self.nextUpdateLabel][1])

		self.nextUpdateLabel = self.nextUpdateLabel + 1
		if (self.nextUpdateLabel >= len(self.LabelList)):
			self.nextUpdateLabel = 0
		
	# ---------------------	
	def setup_all( self ):
		
		tts.sayAsync("On board computer")
			
		# disable all subwindows
		for i in range(len(MODE)):		
			self.getControl(MODE[i]).setVisible(False)

		# activate default window			
		self.getControl(MODE[self.obcMode]).setVisible(True)
		self.setFocusId(MODE[self.obcMode])
		
		self.getControl(OBC_LIMIT_LOGO_ID).setVisible(False)
		
		# update all labels
		openbm.bcSetOnStateCallback(self.onOBCStateChange)
		openbm.bcReqOBCState()
		
		# first setup update methods for the OBC data
		for i in self.LabelList:
			openbm.bcSetOnValueCallback(i[1], i[2])
			openbm.bcReqOBCValue(i[1])
			xbmc.sleep(50)
		
		# set repeatable timer to update labels without any value
		self.updateTimer.start()
		
		pass

	# ---------------------	OBC Callback Methods -------------------
	def onConsumption1Value(self, value):
		if (not self.selectedButton == 0):
			self.getControl(self.LabelList[0][0]).setLabel(str(value).lower())
		pass
		
	def onConsumption2Value(self, value):
		if (not self.selectedButton == 1):
			self.getControl(self.LabelList[1][0]).setLabel(str(value).lower())
		pass

	def onRangeValue(self, value):
		if (not self.selectedButton == 2):
			self.getControl(self.LabelList[2][0]).setLabel(str(value).lower())
		pass

	def onDistanceValue(self, value):
		if (not self.selectedButton == 3):
			self.getControl(self.LabelList[3][0]).setLabel(str(value).lower())
		pass

	def onArrivalValue(self, value):
		if (not self.selectedButton == 4):
			self.getControl(self.LabelList[4][0]).setLabel(str(value).lower())
		pass

	def onAverageSpeedValue(self, value):
		if (not self.selectedButton == 5):
			self.getControl(self.LabelList[5][0]).setLabel(str(value).lower())
		pass

	def onSpeedLimitValue(self, value):
		if (not self.selectedButton == 6):
			self.getControl(self.LabelList[6][0]).setLabel(str(value).lower())
		pass
		
	def onOutsideTemperatureValue(self, value):
		if (not self.selectedButton == 7):
			self.getControl(self.LabelList[7][0]).setLabel(str(value))
		pass
		
	def onOBCStateChange(self, limit, memo, stopwatch, auxheat, code):
		if (limit > 0):
			self.getControl(OBC_LIMIT_LOGO_ID).setVisible(True)
			self.LabelList[6][5] = True
		else:
			self.getControl(OBC_LIMIT_LOGO_ID).setVisible(False)
			self.LabelList[6][5] = False			
		pass
		
	# ---------------------
	def onAction(self, action):
		self.onAction1(action.getId())
		pass
		
	# ---------------------
	def onClick( self, controlId ):
		#self.onAction1(ACTION_SELECT_ITEM)   
		pass
	      
	# ---------------------
	def onAction1(self, action):

		# close window	
		if (action == ACTION_PREVIOUS_MENU) or (action == ACTION_CLOSE_DIALOG):
			self.onStop()

		# we are in the BC mode
		if (self.obcMode == MODE_BC):

			if (action == ACTION_MOVE_LEFT):
				if (self.selectedButton >= 0):
					self.selectedSubButton = (self.selectedSubButton - 1) % len(self.LabelList[self.selectedButton][3])
					self.getControl(self.LabelList[self.selectedButton][0]).setLabel(self.LabelList[self.selectedButton][3][self.selectedSubButton](self.selectedButton, False))
			elif (action == ACTION_MOVE_RIGHT):
				if (self.selectedButton >= 0):
					self.selectedSubButton = (self.selectedSubButton + 1) % len(self.LabelList[self.selectedButton][3])
					self.getControl(self.LabelList[self.selectedButton][0]).setLabel(self.LabelList[self.selectedButton][3][self.selectedSubButton](self.selectedButton, False))
	
			# if selected, then ask for user action
			elif (action == ACTION_SELECT_ITEM):
				buttonId = self.getFocusId() - OBC_BUTTON_BASE
				
				# button was selected before, so now execute the selected action function
				if (self.selectedButton >= 0):
	
					self.getControl(self.LabelList[self.selectedButton][0]).setLabel("")
					
					if (self.selectedButton == buttonId):
						self.LabelList[buttonId][3][self.selectedSubButton](buttonId, True)
						
					openbm.bcReqOBCValue(self.LabelList[self.selectedButton][1])
					self.selectedButton = -1
					self.selectedSubButton = 0
					
				# select button, however only if actions for this buttons exists
				elif (len(self.LabelList[buttonId][3])):
					self.getControl(self.LabelList[buttonId][0]).setLabel(self.LabelList[buttonId][3][0](buttonId, False))
					self.selectedButton = buttonId
					self.selectedSubButton = 0
					
			# on any other action rather then select, we remove the selected button
			else:
				ss = self.selectedButton
				self.selectedButton = -1
				self.selectedSubButton = 0
				if (ss >= 0):
					self.getControl(self.LabelList[ss][0]).setLabel("")
					openbm.bcReqOBCValue(self.LabelList[ss][1])
		
	   
		# if no button is activated and we pressed left or right, then switch current modes
		if (self.selectedButton < 0):
			if (action == ACTION_MOVE_LEFT): self.switchMode(False);
			if (action == ACTION_MOVE_RIGHT): self.switchMode(True);
			#dialog = xbmcgui.Dialog()
			#dialog.ok("Focus - ", str(self.getFocusId()))	
			
	# ---------------------
	def switchMode(self, next = True):

		self.getControl(MODE[self.obcMode]).setVisible(False)

		if (next == True):
			self.obcMode = (self.obcMode + 1) % len(MODE)
		else:
			self.obcMode = (self.obcMode - 1) % len(MODE)

		self.getControl(MODE[self.obcMode]).setVisible(True)
		self.setFocus(self.getControl(MODE[self.obcMode]))
				
		pass
		
		
