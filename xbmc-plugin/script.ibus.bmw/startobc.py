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

try:

	import bordcomputer
	uiBC = bordcomputer.GUI( "bc-skin.xml", os.getcwd())
	uiBC.doModal()
	del uiBC
	
except openbm.error,e:
	dialog = xbmcgui.Dialog()
	dialog.ok("Failed to start BMW on-board computer", str(e))	

