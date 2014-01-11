/* ***************************************************************************

  RC_SMTE.C

  Remote call interface to Small Multi Task Executive (SMTE) header.

  May 24 2010 - Created

*************************************************************************** */

#include <windows.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "rc_smte.h"
#include "remote_task_comm.h"
#include "serialcom.h"

/* -----------------------------------------
   local definitions
----------------------------------------- */

#define  BUFFER         512             // 512 characters in a buffer
#define  NAME_LEN       20              // max. length of com port name string

#define  MESSAGES       10              // messages in circular buffer

#define  DEFAULT_COM    "\\\\.\\COM1"   // default parameters for com port

/* -----------------------------------------
   globals
----------------------------------------- */

HANDLE       hComPort;                                   // com port handle

BYTE         bReadBuff[BUFFER];                          // raw input buffers
BYTE         bWriteBuff[BUFFER];

OVERLAPPED   OverlappedIOStructure = {0, 0, 0, 0, NULL}; // for overlapped IO
DWORD        dwBytesWritten;
DWORD        dwBytesToRead;

HANDLE       hReadMessageThread;                         // thread info
DWORD        dwThreadId;

volatile int nMessageIn            = 0;                  // circular message buffer
volatile int nMessageOut           = 0;
volatile int nMessageCount         = 0;

union T_messageBlock messageBuffer[MESSAGES];
union T_messageBlock tempMessage;
DWORD        dwBytesReadInTemp;

/* -----------------------------------------
   RC_SMTE API
----------------------------------------- */

/* -----------------------------------------------------
   calculateChecksum()

   Canlculate string checksum.

   Returns:
   XOR checksum value of string characters

----------------------------------------------------- */
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

/* -----------------------------------------------------
   checksumOk()

   Test string checksum.

   Returns:
   TRUE for checksum ok and FALSE for bad checksum
----------------------------------------------------- */
BOOL
checksumOk(union T_messageBlock* pMessageBlock)
{
 BYTE bChecksum;

 bChecksum = pMessageBlock->msgChecksumFormat.bChecksum;

 if ( bChecksum == calculateChecksum(pMessageBlock->msgChecksumFormat.msgMsgBytesOnly) )
    {
     return TRUE;
    }
 else
    {
     return FALSE;
    }
}

/* ---------------------------------------------------------
   readCompletionRoutine()

   Routine called from the read thread when message is read
   from COM port

   Returns:
   <none>
--------------------------------------------------------- */
#pragma argsused

void
WINAPI readCompletionRoutine(DWORD        dwErrorCode,
                             DWORD        dwNumberOfBytesRead,
                             LPOVERLAPPED lpOverlapped)
{
 DWORD dwBytesToAdd;
 DWORD dwBytesToCompleteMessage;

 if ( dwErrorCode == 0 )
    {
     // got some characters, start assempling a TASK message and
     // when we have TASK_MESSAGE_LEN characters in temp insert it into
     // the circular buffer
     dwBytesToCompleteMessage = sizeof(union T_messageBlock) - dwBytesReadInTemp;

     if ( dwBytesToCompleteMessage > dwNumberOfBytesRead )
        {
         dwBytesToAdd = dwNumberOfBytesRead;
        }
     else
        {
         dwBytesToAdd = dwBytesToCompleteMessage;
        }

     memcpy(&tempMessage, bReadBuff, dwBytesToAdd);
     dwBytesReadInTemp += dwBytesToAdd;
     
     if ( dwBytesReadInTemp == sizeof(union T_messageBlock) )
        {
         memcpy(&messageBuffer[nMessageIn], &tempMessage, sizeof(union T_messageBlock));
         nMessageIn++;
         nMessageCount++;
         if ( nMessageIn == MESSAGES ) nMessageIn = 0;
         memset(&tempMessage, 0, sizeof(union T_messageBlock));
         dwBytesReadInTemp = 0;
         memcpy(&tempMessage,
                &bReadBuff[dwBytesToCompleteMessage],
                dwNumberOfBytesRead - dwBytesToCompleteMessage);
        }
    }
 else
    {
     // if an error occured, then discard the current message from temp
     memset(&tempMessage, 0, sizeof(union T_messageBlock));
    }
}

/* ---------------------------------------------------------
   readMessageThread()

   Thread to read messages and store in circular buffer

   Returns:
   <none>
--------------------------------------------------------- */
#pragma argsused

DWORD
WINAPI readMessageThread(LPVOID lpParam)
{
 while (1)
    {
     ReadFileEx(hComPort,
                bReadBuff,
                sizeof(union T_messageBlock),
                &OverlappedIOStructure,
                readCompletionRoutine);
     SleepEx(INFINITE, TRUE);
    }
}

/* ---------------------------------------------------------
   readMessage()

   Read message from message buffer.
   Message is always a string of length TASK_MESSAGE_LEN
   as defined in t_task_stub.h

   Returns:
   TRUE for success and FALSE for failure.

--------------------------------------------------------- */
#pragma argsused

BOOL
readMessage(BYTE* pbMessage)
{
 if ( nMessageCount > 0 )
    {
     memcpy(pbMessage, &messageBuffer[nMessageOut], sizeof(union T_messageBlock));
     //printf("readMessage(): %d <%s>\n", nMessageCount, szMessage);
     nMessageOut++;
     nMessageCount--;
     if ( nMessageOut == MESSAGES ) nMessageOut = 0;
     return TRUE;
    }

 return FALSE;
}

/* ---------------------------------------------------------
   writeMessage()

   Write message to message buffer.
   Message is always a string of length TASK_MESSAGE_LEN
   as defined in t_task_stub.h
   This function uses Win32 WriteFile() API with async IO.

   Returns:
   TRUE for success and FALSE for failure.

--------------------------------------------------------- */
BOOL
writeMessage(const BYTE* pbMessage)
{
 //printf("writeMessage(): <%s>\n", szMessage);

 return WriteFile(hComPort,
                  pbMessage,
                  sizeof(union T_messageBlock),
                  &dwBytesWritten,
                  &OverlappedIOStructure);
}

/* -----------------------------------------------------
   setComPort()

   this function sets port parameters
   for port hSerial

   Returns:
   TRUE on seccess and FALSE on failure

   ----------------------------------------------------- */
BOOL
setComPort(const HANDLE hSerial,
           const DWORD  dwBaud,
           const BYTE   bBits,
           const BYTE   bParity,
           const BYTE   bStopBits)
{
 DCB dcbSerialParams   = {0};

 //printf("\tsetComPort()\n");

 dcbSerialParams.DCBlength=sizeof(DCB);
 if (!GetCommState(hSerial, &dcbSerialParams))
    {
     //printf("\tFAIL: GetCommState(), line %d error %d\n", __LINE__, GetLastError());
     return FALSE;
    }

 dcbSerialParams.BaudRate = dwBaud;
 dcbSerialParams.ByteSize = bBits;
 dcbSerialParams.StopBits = bStopBits;
 dcbSerialParams.Parity   = bParity;

 if(!SetCommState(hSerial, &dcbSerialParams))
    {
     //printf("\tFAIL: SetCommState(), line %d error %d\n", __LINE__, GetLastError());
     return FALSE;
    }

 return TRUE;
}

/* -----------------------------------------------------
   openComPort()

   open comm port sPortName and
   return file handle hSerial for read/write operations
   this function prints/sets port parameters
   this function sets IO time outs

   Returns:
   TRUE on seccess and FALSE on failure

   ----------------------------------------------------- */
BOOL
openComPort(HANDLE*     phSerial,
            const char* psPortName,
            const DWORD dwBaud,
            const BYTE  bBits,
            const BYTE  bParity,
            const BYTE  bStopBits)
{
 DCB          dcbSerialParams = {0};
 COMMTIMEOUTS timeouts        = {0};

 //printf("openComPort()\n\t%s\n",psPortName);

 // open COM port

 *phSerial = CreateFile(psPortName,
                        GENERIC_READ | GENERIC_WRITE,
                        0,
                        0,
                        OPEN_EXISTING,
                        FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
                        0);

 if(*phSerial == INVALID_HANDLE_VALUE)
    {
     printf("FAIL: CreateFile(%s), line %d error %d\n", psPortName, __LINE__, GetLastError());
     return FALSE;
    }

 // set port parameters

 if ( !setComPort(*phSerial, dwBaud, bBits, bParity, bStopBits) )
    {
     printf("FAIL: setComPort(), line %d error %d\n", __LINE__, GetLastError());
     return FALSE;
    }

 dcbSerialParams.DCBlength=sizeof(DCB);
 GetCommState(*phSerial, &dcbSerialParams);
 //printf("\tBaud=%d Bits=%d Stop=%d Parity=%d\n",
 //       dcbSerialParams.BaudRate,
 //       dcbSerialParams.ByteSize,
 //       dcbSerialParams.StopBits,
 //       dcbSerialParams.Parity);

 // setup port timeouts
 timeouts.ReadIntervalTimeout         = TIMEOUT_READ_INTERVAL;
 timeouts.ReadTotalTimeoutMultiplier  = TIMEOUT_READ_MULTIPLIER;
 timeouts.ReadTotalTimeoutConstant    = TIMEOUT_READ_CONSTANT;
 timeouts.WriteTotalTimeoutMultiplier = TIMEOUT_WRITE_MULTIPLIER;
 timeouts.WriteTotalTimeoutConstant   = TIMEOUT_WRITE_CONSTANT;
 if ( !SetCommTimeouts(*phSerial, &timeouts) )
    {
     printf("FAIL: SetCommTimeouts(), line %d error %d\n", __LINE__, GetLastError());
     return FALSE;
    }

 // setup the read message thread
 hReadMessageThread = CreateThread(NULL,
                                   0,
                                   readMessageThread,
                                   NULL,
                                   0,
                                   &dwThreadId);
 if ( hReadMessageThread == NULL )
    {
	 printf("FAIL: CreateThread(), line %d error %d\n", __LINE__, GetLastError());
     return FALSE;
    }

 return TRUE;
}

/* ---------------------------------------------------------
   SMTERC_initRemoteConnection()

   Setup a remote connection to AMR through COM port.
   Function returns TRUE for success and FALSE for failure.

--------------------------------------------------------- */
BOOL
SMTERC_initRemoteConnection(char*       szPortName,
                            const DWORD dwBaud,
                            const BYTE  bBits,
                            const BYTE  bParity,
                            const BYTE  bStopBits)
{
 char*   szComPort[NAME_LEN + 1] = {0};

 if ( strlen(szPortName) == 0 )
    {
     strncpy((char *) szComPort, DEFAULT_COM, NAME_LEN);
    }
 else
    {
     strncpy((char *) szComPort, szPortName, NAME_LEN);
    }

 if ( !openComPort(&hComPort, (char *) szComPort, dwBaud, bBits, bParity, bStopBits) )
    {
     return FALSE;
    }

 return TRUE;
}

/* ---------------------------------------------------------
   SMTERC_deleteRemoteConnection()

   This function deletes and clears all remote SMTE
   connection objects.

   --------------------------------------------------------- */
void
SMTERC_deleteRemoteConnection(void)
{
 CloseHandle(hReadMessageThread);
 CloseHandle(hComPort);
}

/* ---------------------------------------------------------
   SMTERC_putMsg()

   This function sends a message to the destination
   task ID 'bTid'.

   Return:
   FALSE on failure and TRUE on success

--------------------------------------------------------- */
BOOL
SMTERC_putMsg(int       nTid,
              short int nPayload,
              WORD      wPayload,
              DWORD     dwPayload)
{
 union T_messageBlock TempMessage;
 BYTE  bChecksum;

 // create message w/out checksum
 strncpy(TempMessage.msgMessageFormat.header, TASK_MSG_TOKEN, sizeof(TASK_MSG_TOKEN));
 TempMessage.msgMessageFormat.bType     = (BYTE) nTid;
 TempMessage.msgMessageFormat.nMsg      = nPayload;
 TempMessage.msgMessageFormat.wPayload  = wPayload;
 TempMessage.msgMessageFormat.dwPayload = dwPayload;

 // calculate and add checksum
 bChecksum = calculateChecksum(TempMessage.msgChecksumFormat.msgMsgBytesOnly);
 TempMessage.msgStringFormat.bChecksum = bChecksum;

 // send message and wait for response
 return writeMessage(TempMessage.msgBytesFormat);
}

/* ---------------------------------------------------------
   SMTERC_getMsg()

   This function returns a message from the task's msg
   queue.

   Return:
   Function returns Q_EMPTY if failed or if queue is empty and
   bPayload if not.

--------------------------------------------------------- */
int
SMTERC_getMsg(int*    pnPayload,
              WORD*   pwPayload,
              DWORD*  pdwPayload)
{
 union T_messageBlock ReceivedMessage;

 if ( !readMessage(ReceivedMessage.msgBytesFormat) )
    {
     return Q_EMPTY;
    }

 //printf("SMTERC_getMsg(): <%s>\n", ReceivedMessage);

 // extract task payload and return
 if ( checksumOk(&ReceivedMessage) )
    {
     *pnPayload  = ReceivedMessage.msgMessageFormat.nMsg;
     *pwPayload  = ReceivedMessage.msgMessageFormat.wPayload;
     *pdwPayload = ReceivedMessage.msgMessageFormat.dwPayload;
     return *pnPayload;
    }
 else
    {
     return Q_EMPTY;
    }
}

/* ---------------------------------------------------------
   SMTERC_waitMsg()

   This function blocks until a message 'bMsg' has been received
   or time-out reached.
   if 'bMsg' is __ANY__ the function will return the first message received
   upon return, 'pwPayload' and 'pdwPayload' contain message
   payloads and function returns 'bMsg'.

   Block forever with 'dwTimeOut' = 0

   Function returns Q_EMPTY if function timed out before
   message has been received, or bMsg received if not failed and
   not timed out.

   Function will discard messages that do not match bMsg.

--------------------------------------------------------- */
int
SMTERC_waitMsg(int     nMsg,
               DWORD   dwTimeOut,
               WORD*   pwPayload,
               DWORD*  pdwPayload)
{
 int    nPayload;
 DWORD  dwExitTime;

 if ( dwTimeOut == 0 )
    {
     dwExitTime = 0;
    }
 else
    {
     dwExitTime = GetTickCount() + dwTimeOut;
    }

 // wait until timed out or message found in queue
 while ( (dwExitTime >= GetTickCount()) || (dwExitTime == 0) )
    {
     // get a message from the queue
     while ( (nPayload = SMTERC_getMsg(&nPayload, pwPayload, pdwPayload)) != Q_EMPTY )
        {
            // return it if it matches requested message
			if ( (nMsg == __ANY__) || (nMsg == nPayload) )
			{
				return nPayload;
            }
        } // wait for message
        
     Sleep(5);
    } // wait for time-out

 return Q_EMPTY;
}

/* ---------------------------------------------------------
   SMTERC_getTidByName()

   This function returns a task id that matches the
   input task name 'szTaskName'.

   Return:
   Function returns the task id, or '0' if name match
   failed.

--------------------------------------------------------- */
int
SMTERC_getTidByName(const char* szTaskName) // pointer to task name
{
 union T_messageBlock TempMessage;

 BYTE  bChecksum;

 int   nTaskId;
 WORD  wPayload   = 0xffff;
 DWORD dwPayload  = 0xdeadbeef;

 // create message w/out checksum
 strncpy(TempMessage.msgStringFormat.header, TASK_MSG_TOKEN, sizeof(TASK_MSG_TOKEN));
 TempMessage.msgStringFormat.bType = MSG_TYPE_STRING;
 strncpy(TempMessage.msgStringFormat.charString, szTaskName, T_NAME_LENGTH);

 // calculate and add checksum
 bChecksum = calculateChecksum(TempMessage.msgChecksumFormat.msgMsgBytesOnly);
 TempMessage.msgStringFormat.bChecksum = bChecksum;

 // send message and wait for response
 if ( !writeMessage(TempMessage.msgBytesFormat) ) return 0;

 // return result, only wait for 5 sec time out
 nTaskId = SMTERC_waitMsg(__ANY__, 5000, &wPayload, &dwPayload);

 return nTaskId;
}

/* ---------------------------------------------------------
   SMTERC_flushMsgQ()

   This function will flush all messges from the calling
   task's message queue.
--------------------------------------------------------- */
void
SMTERC_flushMsgQ(void)
{
 nMessageIn    = 0;
 nMessageOut   = 0;
 nMessageCount = 0;
}
