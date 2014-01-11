/* ***************************************************************************

  serialcom.h

  define common serial com parameter definitions

  Feb 25, 2012 - created

*************************************************************************** */

#ifndef __SERIALCOM_H__
#define __SERIALCOM_H__

#include  <windows.h>

// comm port setup
#define COM_BAUD     CBR_9600
#define COM_BITS     8
#define COM_PARITY   NOPARITY
#define COM_STOPBIT  ONESTOPBIT

// comm port timeouts
#define TIMEOUT_READ_INTERVAL     2
#define TIMEOUT_READ_MULTIPLIER   0
#define TIMEOUT_READ_CONSTANT     2
#define TIMEOUT_WRITE_MULTIPLIER  1
#define TIMEOUT_WRITE_CONSTANT    2


#endif /* __SERIALCOM_H__ */