/* ***************************************************************************

  RC_SMTE.H

  Remote call interface to Small Multi Task Executive (SMTE) header.

  May 24 2010 - Created

*************************************************************************** */

#ifndef __RC_SMTE_H__
#define __RC_SMTE_H__

#include "serialcom.h"

/* -----------------------------------------
   types and definitions
----------------------------------------- */

//typedef unsigned char BYTE;   // 8 bit type
//typedef unsigned int  WORD;   // 16 bit type
//typedef unsigned long DWORD;  // 32 bit type

#define  DEFAULT_BAUD    COM_BAUD
#define  DEFAULT_BITS    COM_BITS
#define  DEFAULT_PARITY  COM_PARITY
#define  DEFAULT_STOP    COM_STOPBIT

/* -----------------------------------------
   function prototypes
----------------------------------------- */

BOOL                                                // setup a remote connection to AMR through COM port
SMTERC_initRemoteConnection(char*       szPortName, // COM port to use or "" for default
                            const DWORD dwBaud,     // bad rare
                            const BYTE  bBits,      // data bits
                            const BYTE  bParity,    // parity
                            const BYTE  bStopBits); // stop bits

void                                                // remove connection resources
SMTERC_deleteRemoteConnection(void);

int
SMTERC_getTidByName(const char* szTaskName);    // pointer to task name

BOOL
SMTERC_putMsg(int nTid,                         // destination task
              short int nPayload,               // message type identifier
              WORD      wPayload,               // word payload
              DWORD     dwPayload);             // double word payload

int                                             // returns Q_EMPTY if no messages,
                                                // or bPayload if got message
SMTERC_getMsg(int*    pnPayload,                // message, byte payload
              WORD*   pwPayload,                // word payload
              DWORD*  pdwPayload);              // double word payload

int                                             // returns Q_EMPTY if timed out,
                                                // or bPayload if got message
SMTERC_waitMsg(int     nMsg,                    // message to wait for or '__ANY__'
               DWORD   dwTimeOut,               // wait time out value, 0 = forever
               WORD*   pwPayload,               // word payload
               DWORD*  pdwPayload);             // double word payload

void                                            // flush all pending messages
SMTERC_flushMsgQ(void);                         // of calling task

/* -----------------------------------------
   global macro definitions
----------------------------------------- */

#define PACK_DW(DOUBLE, WORD1, WORD2)      \
           {                               \
            DOUBLE  = (DWORD) WORD1 << 16; \
            DOUBLE |= (WORD) WORD2;        \
           }

/* -----------------------------------------
   global flag definitions
----------------------------------------- */

#define Q_EMPTY           0              // message queue empty flag
#define __ANY__           0

#endif  // __RC_SMTE_H__
