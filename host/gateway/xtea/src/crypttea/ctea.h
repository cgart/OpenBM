/* Copyright (C) 2007 Christian Heimes <christian (at) cheimes (dot) de>
   pyrexc optimized version
   License: Public Domain
*/

#define MODE_ECB 1 // Electronic Code Book
#define MODE_CBC 2 // Cipher Block Chaining
#define MODE_CFB 4 // Cipher Feedback 
#define MODE_OFB 5 // Output Feedback
#define MODE_CTR 6 // Counter
#define DELTA 0x9E3779B9

typedef unsigned long ULong;
typedef unsigned int UInt;

int btea_decipher(ULong *const v, const ULong *const k, const UInt n);
int btea_encipher(ULong *const v, const ULong *const k, const UInt n);
int tea_decipher(ULong *const v, const ULong *const k, const UInt n);
int tea_encipher(ULong *const v, const ULong *const k, const UInt n);
int xtea_decipher(ULong *const v, const ULong *const k, const UInt n);
int xtea_encipher(ULong *const v, const ULong *const k, const UInt n);
int xxtea_decipher(ULong *const v, const ULong *const k, const UInt n);
int xxtea_encipher(ULong *const v, const ULong *const k, const UInt n);

