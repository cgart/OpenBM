import xbmc
import xbmcaddon
import threading
import os
import sys


__settings__ = xbmcaddon.Addon(id='script.ibus.bmw')
#__settings__ = sys.modules[ "__main__" ].__settings__


# -----------------------------------------------
# Method which will perform text to speech 
# using festival.
# the execution is hold while speaking
# -----------------------------------------------
def say(ss):
	#xbmc.executebuiltin("XBMC.Notification(%s,%s,2000,'')"%("Settings", str(float(__settings__.getSetting( "speech.volume" )))))	
	xbmc.executebuiltin('XBMC.SetVolume(90)')
	xbmc.sleep(100)
	xbmc.executebuiltin('XBMC.SetVolume(80)')
	xbmc.sleep(100)
	xbmc.executebuiltin('XBMC.SetVolume(70)')
	xbmc.sleep(100)
	xbmc.executebuiltin('XBMC.SetVolume(60)')

	os.system(('echo \'%s\' | text2wave -scale %f | paplay') % (ss, float(__settings__.getSetting( "speech.volume" ))))
	
	xbmc.executebuiltin('XBMC.SetVolume(70)')
	xbmc.sleep(100)
	xbmc.executebuiltin('XBMC.SetVolume(80)')
	xbmc.sleep(100)
	xbmc.executebuiltin('XBMC.SetVolume(90)')
	xbmc.sleep(100)
	xbmc.executebuiltin('XBMC.SetVolume(100)')
	
	
# -----------------------------------------------
# Peform also tts but now in a thread, returns immediately
# -----------------------------------------------
def sayAsync(ss):

	t1 = threading.Thread(target=say, args=(ss,))
	t1.start()
