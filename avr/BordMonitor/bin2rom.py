
#imports
import os
import sys
from crypttea.xtea import XTEA
from crypttea._common import BIG_ENDIAN, LITTLE_ENDIAN, MODE_ECB 
import binascii
import struct
from CONFIG import VERSION_MAJOR, VERSION_MINOR

CHUNKSIZE = 64


def crc16_update(crc, byte):
	crc = crc ^ byte
	for i in range(0,8):
		if (crc & 1):
			crc = (crc >> 1) ^ 0xA001
		else:
			crc = (crc >> 1)
	return crc


# Open file which is the first parameter
binfile = sys.argv[1]
hkey = open(sys.argv[2], 'r').read()
print 'User firmware password: ' + hkey

# read key as string of hex and convert into a list
key = ''
for i in range(0, 32, 2):
	key = str(key) + str(binascii.a2b_hex(str(hkey[i]) + str(hkey[i+1])))

xtea = XTEA.new(key, mode=MODE_ECB, endian=LITTLE_ENDIAN) 
outfile = open('firmware.rom', 'wb')

# read binary file till the end
fsize = os.path.getsize(binfile)
fpos = 0
f = open(binfile, 'rb')
crc = 0xFFFF

# Print file header
outfile.write(struct.pack('BBBB', ord('O'), ord('B'), ord('M'), 1))
outfile.write(struct.pack('H', CHUNKSIZE))
outfile.write(struct.pack('BB', VERSION_MAJOR, VERSION_MINOR))

try:
	
	while (fsize - fpos > 0):
		chunk = ''
		
		# read 8 byte chunks from file
		if (fsize - fpos >= CHUNKSIZE):
			#chunk.append(f.read(CHUNKSIZE))
			chunk += f.read(CHUNKSIZE)
			fpos += CHUNKSIZE
		else:
			#chunk.append(f.read(fsize - fpos))
			chunk += f.read(fsize - fpos)
			for j in range(0,CHUNKSIZE-(fsize-fpos)):
				chunk += chr(0)
				#chunk.append(chr(0))
			fpos += (fsize-fpos)
				
		# update crc sum
		for j in range(0, CHUNKSIZE):
			crc = crc16_update(crc, ord(chunk[j]))
		
		# process chunk
		ec = xtea.encrypt(chunk)
		
		# write processed chunk to output file
		outfile.write(ec)
finally:
    f.close()

# put crc
print "Version=" + str(VERSION_MAJOR) + "." + str(VERSION_MINOR)
print 'CRC16 = 0x%X%X' % (ord(struct.pack('B', (crc & 0xFF00) >> 8)), ord(struct.pack('B', crc & 0xFF)))

outfile.write(struct.pack('BB', (crc & 0xFF00) >> 8, crc & 0xFF))

print "firmware.rom file generated!"


