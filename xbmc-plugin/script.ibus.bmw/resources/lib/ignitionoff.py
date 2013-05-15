
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

# --------------------------------------------------
class GUI( xbmcgui.WindowXML ):
			
	# ---------------------	
	def __init__( self, *args, **kwargs ):

		pass

	# ---------------------	
	def onInit( self ):
		openbm.setOnMessage(self.onMessage)
		pass

	# ---------------------	
	def onStop( self):
		self.close()

	# ---------------------	
	def onFocus( self, controlId ):
		pass

	# ---------------------
	def onAction(self, action):
		self.onAction1(action.getId())
		pass
		
	# ---------------------
	def onAction1(self, action):

		# close window	
		if (action == ACTION_PREVIOUS_MENU) or (action == ACTION_CLOSE_DIALOG):
			self.onStop()

	# ---------------------
	def onMessage(self, src, dst, data):

	        if (openbm.bcGetIgnitionState() > 0):
			self.onStop()
		
                pass
		
