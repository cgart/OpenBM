# -*- coding: utf-8 -*-
# import and start Debugger session
#import rpdb2 
#rpdb2.start_embedded_debugger('pw')

# import the XBMC libraries so we can use the controls and functions of XBMC
import xbmc
import xbmcgui
import xbmcaddon
import os
import sys
import time
import threading 
import datetime

# Script properties
__scriptname__ = "BMW Daemon"
__author__ = "Art Tevs"
__GUI__    = "Art Tevs"
__version__ = "1.2"
__settings__ = xbmcaddon.Addon(id='script.ibus.bmw')
__language__ = __settings__.getLocalizedString


# add librarys and resource files into sys path
BASE_RESOURCE_PATH = xbmc.translatePath( os.path.join( os.getcwd(), 'resources' ) )
BASE_LIB_PATH = xbmc.translatePath( os.path.join( os.getcwd(), 'resources', 'lib' ) )
sys.path.append (BASE_RESOURCE_PATH)
sys.path.append (BASE_LIB_PATH)

# setup default variables
bmwLogoSmallImg = os.path.join ( BASE_RESOURCE_PATH, "bmw_logo_small.png")

import openbm
import radio
import bordcomputer
import tts
import ignitionoff

# Process pid file used as mutex to stop execution
_processPID = os.path.join( BASE_RESOURCE_PATH , "ibus.pid")
_radioState = 0
_obcActive = False
_ignitionOff = False
_cwd = os.getcwd()

#---------------------------------------------------------------------
# Show simple GUI, when ignition is off
#---------------------------------------------------------------------
def runIgnitionOffGUI():

	_ignitionOff = True
	
	# pause XBMC player
	xbmc.executebuiltin('XBMC.PlayerControl(play)')
		
	uiMid = ignitionoff.GUI( "ignition-off-skin.xml", _cwd)
	uiMid.doModal()
	del uiMid
		
	# resume if played before
	xbmc.executebuiltin('XBMC.PlayerControl(play)')

	# reset callback method
	openbm.setOnMessage(onMessage)

	_ignitionOff = False

#---------------------------------------------------------------------
# Run Radio gui parallel to the daemon
#---------------------------------------------------------------------
def runRadioGUI():

	# pause XBMC player
	xbmc.executebuiltin('XBMC.PlayerControl(play)')
		
	# show radio GUI
	uiMid = radio.GUI( "radio-skin.xml", _cwd)
	uiMid.setRadioState(_radioState)
	uiMid.doModal()
	del uiMid
		
	# resume if played before
	xbmc.executebuiltin('XBMC.PlayerControl(play)')

	# reset callback method
	openbm.setOnRadioStateChange(onRadioState)
	openbm.setOnMessage(onMessage)


#---------------------------------------------------------------------
# Run Bordcomputer GUI parallel to the rest
#---------------------------------------------------------------------
def runBordcomputerGUI():

	_obcActive = True
	
	# show radio GUI
	uiBC = bordcomputer.GUI( "bc-skin.xml", _cwd)
	uiBC.doModal()
	del uiBC

	# reset callbacks
	openbm.setOnMessage(onMessage)
	
	_obcActive = False
	
		
#---------------------------------------------------------------------
# On Disconnect we force to stop the script
#---------------------------------------------------------------------
def onDisconnect():
	xbmc.executebuiltin("XBMC.Notification(%s,%s,2000,%s)"%("Disconnected", "Connection to I-Bus server lost...", bmwLogoSmallImg))	
	if os.path.exists(_processPID):
		os.remove(_processPID)
	pass
	
#---------------------------------------------------------------------
# On Radio state change we check if we have to open/close radio window
#---------------------------------------------------------------------
def onRadioState(state):

	# radio gone to RADIO or CD mode, so we start radio GUI
	if (state == 0 or state == 1):
			
		# start radio gui in another thread and wait a short while to let it run before next events come
		_radioState = state
		threading.Thread(target=runRadioGUI).start()
		xbmc.sleep(500)
	
	# if we activate TONE mode
	if (state == 3):
		_radioState = state

#---------------------------------------------------------------------
# On OBC Button click, we activate OBC script if not active already
#---------------------------------------------------------------------
def onOBCState():

	# if not running, then start
	if (_obcActive == False):
			
		threading.Thread(target=runBordcomputerGUI).start()
		xbmc.sleep(500)

	# is already running, then change OBC state (telephone, obc, ...)
	else:
		pass

#---------------------------------------------------------------------
# Activated if Ignition is turned off
#---------------------------------------------------------------------
def onIgnitionState(state):

	if (_ignitionOff == False and state == 0):
	
		threading.Thread(target=runIgnitionOffGUI).start()
		xbmc.sleep(500)
	else:
		pass
		
#---------------------------------------------------------------------
# General reaction on IBus messages
#---------------------------------------------------------------------
def onMessage(src, dst, data):

	# CLOCK button up (so just short press) -> say current time:
	if (src == 0xF0 and dst == 0xFF and data[0] == 0x48 and data[1] == 0x87):
		tts.sayAsync(datetime.datetime.now().strftime("Current time is %H:%M, on %d %B"))

	#50 04 C8 3B A0 07
	if (src == 0x50 and dst == 0xC8 and data[0] == 0x3B and data[1] == 0xA0):
		tts.sayAsync(datetime.datetime.now().strftime("Current time is %H:%M, on %d %B"))
				
	# CLOCK button down (long down), activate OBC
	if (src == 0xF0 and dst == 0xFF and data[0] == 0x48 and data[1] == 0x47):
		onOBCState()
				
	pass
	
	
#---------------------------------------------------------------------
# Main Function - run a daemon checking for actions on ibus
#         it will execute corresponding scripts based on ibus messages
#---------------------------------------------------------------------
if ( __name__ == "__main__" ):

	# write file to lock execution (remove previous one)
	if os.path.exists(_processPID):
		os.remove(_processPID)        
	file( _processPID , "w" ).write( "" )

	try:
		openbm.setOnDisconnect(onDisconnect)
		openbm.midEnableEmulation()
		openbm.setOnRadioStateChange(onRadioState)
		openbm.setOnMessage(onMessage)
		
		# try to connect to IBus client, repeat the retries
		try:	
			openbm.start()
			_connectionLost = False
			
			# perform greetings and say current time
			#xbmc.executebuiltin("XBMC.Notification(%s,%s,2000,'')"%("Settings", __settings__.getSetting( "speech.greetings")))	
			if (__settings__.getSetting( "speech.greetings") == 'true'):
				now = datetime.datetime.now()
				greeting = ""
				if (now.hour < 4):
					greeting = 'Good night, '
				elif (now.hour < 12):
					greeting = 'Good morning, '
				elif (now.hour < 18):
					greeting = 'Good afternoon, '
				else:
					greeting = 'Good evening, '
				tts.sayAsync(greeting + now.strftime("Current time is %H:%M, on %d %B"))
			
			xbmc.executebuiltin("XBMC.Notification(%s,%s,2000,%s)"%("Connected", "BMW I-Bus connected...", bmwLogoSmallImg))		
			
		except openbm.error,e:
			dialog = xbmcgui.Dialog()
			dialog.ok("Connection to IBus failed!", str(e), "Please restart the application and the I-Bus gateway!")	
			raise
			
		# update BC's time and date
		openbm.bcSetTimeFromSystem()
		openbm.bcSetDateFromSystem()
		openbm.bcSetOnIgnitionStateCallback(onIgnitionState)
		
		# update MID's text fields	
		openbm.midReqUpdateFields()
		openbm.bcReqIgnitionState()
		
		# prepare time speak
		nextHour = (datetime.datetime.now().hour + 1) % 24
		
		# start main loop
		#while (not xbmc.abortRequested and os.path.exists(_processPID)):
		while (True):
			xbmc.sleep(1000)
			
			# if ignition turned off, then activate according gui element
			#if (openbm.bcGetIgnitionState() == 0):
			#    onIgnitionOff()
			    
			# if abort wished, then stop really
			if (xbmc.abortRequested):
				raise
				
			# check if we lost connection, then we will try to reconnect
			if (not os.path.exists(_processPID)):
				try:	
					openbm.start()
					xbmc.executebuiltin("XBMC.Notification(%s,%s,500,%s)"%("Reconnected", "BMW I-Bus connected...", bmwLogoSmallImg))		
					file( _processPID , "w" ).write( "" )
				except:
					pass
			
			# check if current time reached a full hour, then notice
			if (__settings__.getSetting( "speech.hourly") == 'true'):
				now = datetime.datetime.now()
				if (now.hour == nextHour and now.minute == 0):
					tts.say(now.strftime("It is now %H o clock"))
					nextHour = (now.hour + 1) % 24
				
				
	except:
		pass

	# clear pid file 	
	if os.path.exists(_processPID):
		os.remove(_processPID)        

	
