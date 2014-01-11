/* ***************************************************************************

  T_TASK_STUB.C

  Task stub for NAV task that resides on PC.
  Task will convert and forward messages between COM port and ARM tasks.

  April 1 2012 - modified message structure
  May  16 2010 - Created

*************************************************************************** */

#include  <string.h>
#include  <dos.h>

#include  "v25.h"
#include  "smte.h"
#include  "internal.h"
#include  "remote_task_comm.h"
#include  "t_task_stub.h"

#pragma intrinsic memcpy  // compile memcpy() as inline
#pragma intrinsic memset  // compile memset() as inline
#pragma intrinsic strncpy // compile strncpy() as inline

/* -----------------------------------------
   module definitions
----------------------------------------- */

#define  MAX_MESSAGES      11    // max messages in circular buffer

#define  CON_COMM_VEC      13    // serial recieve interrupt channel 0
#define  CON_COMM_RX_INT   0x20  // enable and set to macro service
#define  CON_COMM_RX_MAC   0x11  // Rx macro ctrl 8-bit, SFR to memory, ch 1

#define  CON_RX_REGISTER   0x60  // Rx register

/* -----------------------------------------
   task globals
----------------------------------------- */

// NOTE: buffer is base index 1. index 0 is used for tem store

volatile union T_messageBlock messageBuffer[MAX_MESSAGES]; // circular buffer
volatile int                  nStubMsgIn    = 1;           // input index
volatile int                  nStubMsgOut   = 1;           // output index
volatile int                  nStubMsgCount = 0;           // message blocks

/* -----------------------------------------
   conCommIsr()

   Console communication port ISR
----------------------------------------- */
void interrupt
conCommIsr(void)
{
 // copy received message data into buffer
 memcpy((void*) &messageBuffer[nStubMsgIn],
        (void*) &messageBuffer[0],
         sizeof(union T_messageBlock));

 // manage circular buffer
 nStubMsgCount++;
 nStubMsgIn++;
 if ( nStubMsgIn == MAX_MESSAGES )
    {
     nStubMsgIn = 1;
    }

 // re-initialize macro registers of channel #1 (second channel)
 ((struct macroChannel_tag _far*) MK_FP(0xf000, 0xfe00 + sizeof(struct macroChannel_tag)))->bMSC  = sizeof(union T_messageBlock);
 ((struct macroChannel_tag _far*) MK_FP(0xf000, 0xfe00 + sizeof(struct macroChannel_tag)))->wMSP  = FP_OFF(&messageBuffer[0]);

 // reselect macro mode
 ((struct SFR _far*) MK_FP(0xf000, 0xff00))->sric0 |= CON_COMM_RX_INT;

 // ISR epilog
 asm { db  0x0f   // the function will only be hooked if __V25__ is defined
       db  0x92   // so this is ok.
     }
}

/* -----------------------------------------
   conCommInit()

   Console communication port setup:

   Interuupt vector set to point to ISR, and
   operate in macro service mode.
   Macro service executes to get TASK_MESSAGE_LEN
   bytes and ISR transfers them to a circuar
   buffer.

   returns '-1' if init Ok, '0' if failed.
----------------------------------------- */

#pragma argsused

int
conCommInit(void)
{

 #ifdef __V25__

 WORD _far*                    wpVector;
 struct SFR _far*              pSfr;
 struct macroChannel_tag _far* pMacroChannel;

 // initialize message buffer to 0
 memset(&messageBuffer[0], 0, sizeof(union T_messageBlock) * MAX_MESSAGES);

 // setup console interrupt vector

 asm { cli }

 wpVector      = MK_FP(0, (CON_COMM_VEC * 4));
 *wpVector++   = FP_OFF(conCommIsr);
 *wpVector     = FP_SEG(conCommIsr);

 // console comm port macro service setup

 pSfr        = MK_FP(0xf000, 0xff00);
 pSfr->srms0 = CON_COMM_RX_MAC;
 pSfr->sric0 = (pSfr->sric0 & 0x8f) | CON_COMM_RX_INT;

 // macro channel setup of channel #1 (second channel)

 pMacroChannel = MK_FP(0xf000, 0xfe00 + sizeof(struct macroChannel_tag));

 pMacroChannel->bSFRP = CON_RX_REGISTER;
 pMacroChannel->bMSC  = sizeof(union T_messageBlock);
 pMacroChannel->bSCHR = 0;
 pMacroChannel->wMSP  = FP_OFF(&messageBuffer[0]);
 pMacroChannel->wMSS  = _DS;

 asm { sti }
 
 #endif // __V25__

 return -1;
}

/* -----------------------------------------
   calculateChecksum()

   calculate byte XOR checksum on bytes of a string.
   return the byte checksum
----------------------------------------- */
BYTE
calculateChecksum(BYTE* pbBytes)
{
 int  i;
 BYTE bChecksum = 0;

 for (i = 0; i < (sizeof(union T_messageBlock) - 1); i++)
    {
     bChecksum ^= *(pbBytes + i);
    }

 return bChecksum;
}

/* -----------------------------------------
   forwardConMessage()

   create message string and forward to
   PC based task through console
----------------------------------------- */
void
forwardConMessage(BYTE  bType,
                  int   nMsg,
                  WORD  wPayload,
                  DWORD dwPayload)
{
 union T_messageBlock messageBlockToSend;

 memcpy(&messageBlockToSend.msgMessageFormat.header[0], TASK_MSG_TOKEN, sizeof(TASK_MSG_TOKEN));
 messageBlockToSend.msgMessageFormat.bType     = bType;
 messageBlockToSend.msgMessageFormat.nMsg      = nMsg;
 messageBlockToSend.msgMessageFormat.wPayload  = wPayload;
 messageBlockToSend.msgMessageFormat.dwPayload = dwPayload;
 messageBlockToSend.msgMessageFormat.bChecksum = calculateChecksum(messageBlockToSend.msgChecksumFormat.msgMsgBytesOnly);

 sendn(&messageBlockToSend.msgBytesFormat[0], sizeof(union T_messageBlock));
}

/* -----------------------------------------
   task function code
----------------------------------------- */

void
t_nav_stub(void)
{
 int     nMsg;
 WORD    wPayload;
 DWORD   dwPayload;
 int     i;

 char  szName[T_NAME_LENGTH + 1];

 union T_messageBlock messageTemp;

 //print("stub\r\n");

 // get message from AMR tasks
 nMsg = getMsg(&nMsg, &wPayload, &dwPayload);

 // parse message using a switch
 // because I might want to do other things with specific messages
 switch ( nMsg )
    {
     case Q_EMPTY: // Q_EMPTY then there is nothing to do
                   break;
          default: // package message into string and send over COM
                   forwardConMessage(0, nMsg, wPayload, dwPayload);
    } // switch on nMsg


 // see if there are any messages from CON to process and forward
 for ( i = 0; i < nStubMsgCount; i++ )
    {
     // copy the message to a temp location
     memcpy(&messageTemp, &messageBuffer[nStubMsgOut], sizeof(union T_messageBlock));

     // manage circular buffer pointer
     nStubMsgOut++;
     nStubMsgCount--;
     if ( nStubMsgOut == MAX_MESSAGES )
        {
         nStubMsgOut = 1;
        }

     // check message frame
     if ( messageTemp.msgBytesFormat[0] != '!' )
        {
         // re-initialize macro registers
         ((struct macroChannel_tag _far*) MK_FP(0xf000, 0xfe00 + sizeof(struct macroChannel_tag)))->bMSC  = sizeof(union T_messageBlock);
         ((struct macroChannel_tag _far*) MK_FP(0xf000, 0xfe00 + sizeof(struct macroChannel_tag)))->wMSP  = FP_OFF(&messageBuffer[0]);
         // reset circular buffer
         nStubMsgOut   = 1;
         nStubMsgIn    = 1;
         nStubMsgCount = 0;
         putDebugMsg(DB_TRACE, DB_BAD_MSG, __LINE__); // @@ message frame error
         continue;
        }

     // evaluate the checksum
     if ( messageTemp.msgChecksumFormat.bChecksum == calculateChecksum(messageTemp.msgChecksumFormat.msgMsgBytesOnly))
        {
         switch ( messageTemp.msgMessageFormat.bType )
            {
             case MSG_TYPE_STRING: // check for task ID and return to PC task
                                   szName[0] = 0;
                                   strncpy(szName, &messageTemp.msgStringFormat.charString[0], T_NAME_LENGTH + 1);
                                   nMsg = getTidByName(szName);
                                   if ( nMsg == 0 )
                                      {
                                       putDebugMsg(DB_TRACE, DB_BAD_TNAME, __LINE__); // @@ bad task name or task name not found
                                      }
                                   else
                                      {
                                       forwardConMessage(0, nMsg, 0, 0L);
                                      }
                                   break;
                          default: // forward to an AMR task
                                   putMsg(messageTemp.msgMessageFormat.bType,
                                          messageTemp.msgMessageFormat.nMsg,
                                          messageTemp.msgMessageFormat.wPayload,
                                          messageTemp.msgMessageFormat.dwPayload);
            }
        } // checksum is ok
     else
        {
         putDebugMsg(DB_TRACE, DB_BAD_MSG, __LINE__); // @@ bad message checksum
        } // checksum is bad
    } // for loop on buffer
}
