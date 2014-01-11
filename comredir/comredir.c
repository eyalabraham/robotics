/* **************************************************************************
   comredit.c

   COM port redirector.

   May  4, 2010 : Created
   May 24, 2010 : Added muti-threading
   May 30, 2010 : Added Async-IO
   Feb 24, 2012 : dapted to 19200 BAUD and fixed error handling

************************************************************************** */

#include <windows.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "remote_task_comm.h"
#include "serialcom.h"

#define AMR_COM          "\\\\.\\COM1"
//#define AMR_COM          "\\\\.\\CNCA30"
#define CONSOLE_COM      "\\\\.\\COM10"
#define TASK_COM         "\\\\.\\COM20"

#define BUFFER           200  // buffer for characters read/write 

#define WRITE_TO_CONSOLE 1
#define WRITE_TO_TASK    2
#define NEED_MORE_READ   3

/* -----------------------------------------------------
   type definitions and enumerations
   ----------------------------------------------------- */

enum    threadEnum
        {
         AMR = 0,
         CON,
         TASK,
         MAX_THREADS = TASK + 1
        };

/* -----------------------------------------------------
   Globals
   ----------------------------------------------------- */

HANDLE     hAMRCOM;
HANDLE     hConsoleCOM;
HANDLE     hTaskCOM;

char       szAMRBuff[BUFFER];
char       szConsoleBuff[BUFFER];
char       szTaskBuff[BUFFER];

OVERLAPPED overlappedIoAMR      = {0, 0, 0, 0, 0};
OVERLAPPED overlappedIoConsole  = {0, 0, 0, 0, 0};
OVERLAPPED overlappedIoTask     = {0, 0, 0, 0, 0};

HANDLE     hAMRIoEvent;
HANDLE     hConsoleIoEvent;
HANDLE     hTaskIoEvent;

DWORD      dwBytesReadFromAMR     = 0;
DWORD      dwBytesReadFromConsole = 0;
DWORD      dwBytesReadFromTask    = 0;

DWORD      dwThreadIdArray[MAX_THREADS];
HANDLE     hThreadArray[MAX_THREADS];

/* -----------------------------------------------------
   setComPort()

   this function sets port parameters
   for port hSerial

   Function returns TRUE for success and FALSE for failure.

   ----------------------------------------------------- */
BOOL
setComPort(const HANDLE hSerial,
           const DWORD  dwBaud,
           const BYTE   bBits,
           const BYTE   bParity,
           const BYTE   bStopBits)
{
 DCB dcbSerialParams   = {0};

 printf("setComPort()\n");

 dcbSerialParams.DCBlength=sizeof(DCB);
 if (!GetCommState(hSerial, &dcbSerialParams))
    {
     printf("FAIL: GetCommState(), error %d on line %d\n", GetLastError(), __LINE__);
     return FALSE;
    }

 dcbSerialParams.BaudRate      = dwBaud;
 dcbSerialParams.ByteSize      = bBits;
 dcbSerialParams.StopBits      = bStopBits;
 dcbSerialParams.Parity        = bParity;
 dcbSerialParams.fParity       = ( bParity == NOPARITY ) ? FALSE : TRUE;
 dcbSerialParams.fAbortOnError = TRUE; // error reset in handled in threadAMR() by  call to ClearCommError()

 if(!SetCommState(hSerial, &dcbSerialParams))
    {
     printf("FAIL: SetCommState(), error %d on line %d\n", GetLastError(), __LINE__);
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

   Function returns TRUE for success and FALSE for failure.

   ----------------------------------------------------- */
BOOL
openComPort(HANDLE*     phSerial,
            const char* psPortName,
            const DWORD dwBaud,
            const BYTE  bBits,
            const BYTE  bParity,
            const BYTE  bStopBits)
{
 DCB dcbSerialParams   = {0};
 COMMTIMEOUTS timeouts = {0};

 printf("openComPort()\n\t%s\n", psPortName);

 // open COM port

 *phSerial = CreateFile(psPortName,
                        GENERIC_READ | GENERIC_WRITE,
                        0,
                        0,
                        OPEN_EXISTING,
                        FILE_FLAG_OVERLAPPED | FILE_FLAG_WRITE_THROUGH,
                        0);

 if(*phSerial == INVALID_HANDLE_VALUE)
    {
     printf("FAIL: CreateFile(), error %d on line %d\n", GetLastError(), __LINE__);
     return FALSE;
    }

 // set port parameters

 if ( !setComPort(*phSerial, dwBaud, bBits, bParity, bStopBits) )
    {
     printf("FAIL: setComPort(), error %d on line %d\n", GetLastError(), __LINE__);
     return FALSE;
    }

 dcbSerialParams.DCBlength=sizeof(DCB);
 GetCommState(*phSerial, &dcbSerialParams);
 printf("\tBaud=%d Bits=%d Stop=%d Parity=%d\n",
        dcbSerialParams.BaudRate,
        dcbSerialParams.ByteSize,
        dcbSerialParams.StopBits,
        dcbSerialParams.Parity);

 // setup port timeouts
 timeouts.ReadIntervalTimeout         = TIMEOUT_READ_INTERVAL;
 timeouts.ReadTotalTimeoutMultiplier  = TIMEOUT_READ_MULTIPLIER;
 timeouts.ReadTotalTimeoutConstant    = TIMEOUT_READ_CONSTANT;
 timeouts.WriteTotalTimeoutMultiplier = TIMEOUT_WRITE_MULTIPLIER;
 timeouts.WriteTotalTimeoutConstant   = TIMEOUT_WRITE_CONSTANT;
 
 if ( !SetCommTimeouts(*phSerial, &timeouts) )
    {
     printf("FAIL: SetCommTimeouts(), error %d on line %d\n", GetLastError(), __LINE__);
     return FALSE;
    }

 return TRUE;
}

/* -----------------------------------------------------
   peekAMRBuff()

   the functions scans a segment of buffer and looks for
   sequences that determine where the data should be transmitted to.
   returns a destination code and how many bytes should be
   transmitted from current nWritePos

   ----------------------------------------------------- */
int
peekAMRBuff(const char* pszBuffer,
            const int   nWritePos,
            const DWORD dwBytesInAMRBuff,
            DWORD*      pdwBytesToWrite)
{
 static char szMessageToken[]      = TASK_MSG_TOKEN;
 char        szSegment[BUFFER + 1] = {0};
 char*       pszMessagePos;
 int         nReturnValue;

 if ( dwBytesInAMRBuff == 0 )
    {
     *pdwBytesToWrite = 0;
     return NEED_MORE_READ;
    }

 memcpy(szSegment, pszBuffer + nWritePos, dwBytesInAMRBuff);

 pszMessagePos = memchr(szSegment, szMessageToken[0], dwBytesInAMRBuff);
 if ( pszMessagePos == NULL )
    // there is no '!' character so this is not a TASK command string
    // so write everthing to the console
    {
     *pdwBytesToWrite = dwBytesInAMRBuff;
     nReturnValue = WRITE_TO_CONSOLE;
    }
 else if ( szSegment == pszMessagePos )
    // we have a '!' character in the first string position so this
    // might be a TASK commend string
    {
     if ( dwBytesInAMRBuff < sizeof(TASK_MSG_TOKEN) )
        {
         // not a full length token so collect more characters
         nReturnValue = NEED_MORE_READ;
        }
     else
        {
         // full length token so check if this is a token
         if ( memcmp(szSegment, szMessageToken, sizeof(TASK_MSG_TOKEN) ) == 0 )
            {
             // this is a token so collect full message string
             // before sending to TASK
             if ( dwBytesInAMRBuff >= sizeof(union T_messageBlock) )
                {
                 *pdwBytesToWrite = sizeof(union T_messageBlock);
                 nReturnValue = WRITE_TO_TASK;
                }
             else
                {
                 nReturnValue = NEED_MORE_READ;
                }
            }
         else
            {
             // just a single '!' so send to console
             *pdwBytesToWrite = dwBytesInAMRBuff;
             nReturnValue = WRITE_TO_CONSOLE;
            }
        }
    }
 else
    // '!' is not first character so return a response that sends
    // everything in front of the '!' to the console making it the first
    // character in the next peek
    {
     *pdwBytesToWrite = pszMessagePos - szSegment;
     nReturnValue = WRITE_TO_CONSOLE;
    }

 return nReturnValue;
}

/* -----------------------------------------------------
   callbackAMR() AMR IO completion callback function

   ----------------------------------------------------- */
#pragma argsused

void
WINAPI callbackAMRIoThread(DWORD        dwErrorCode,
                           DWORD        dwNumberOfBytesTransfered,
                           LPOVERLAPPED lpOverlapped)
{
 dwBytesReadFromAMR = dwNumberOfBytesTransfered;

 if( dwErrorCode != 0 )
    {
     // error occurred. Report to user
     printf("FAIL: callbackAMRIoThread(), error %d on line %d\n", GetLastError(), __LINE__);
    }

 SetEvent(lpOverlapped->hEvent);
}

/* -----------------------------------------------------
   threadAMR()   AMR IO reading thread.

   ----------------------------------------------------- */
#pragma argsused

DWORD
WINAPI threadAMR(LPVOID lpParam)
{
 int     nReadPos         = 0;
 int     nWritePos        = 0;
 DWORD   dwBytesInAMRBuff = 0;
 DWORD   dwMaxChunkRead   = 0;
 DWORD   dwBytesToWrite   = 0;
 DWORD   dwErrors;
    
 while (1)
 {
     // wait for IO completion on AMR com port
     WaitForSingleObject(hAMRIoEvent, INFINITE);

     // setup hAMRCOM overlapped IO structure
     overlappedIoAMR.Internal = 0;
     overlappedIoAMR.InternalHigh = 0;
     overlappedIoAMR.Offset = 0;
     overlappedIoAMR.OffsetHigh = 0;
     overlappedIoAMR.hEvent = hAMRIoEvent;

     // read data from AMR com port
     if ( ReadFileEx(hAMRCOM,
                     &szAMRBuff[nReadPos],
                     (BUFFER - nReadPos),
                     &overlappedIoAMR,
                     callbackAMRIoThread) )
     {
         // suspend thread until IO completes
         SleepEx(INFINITE, TRUE);
     }
     else
     {
        /*
          Value                 Meaning
          -------------------   -------------------------------------------
          CE_RXOVER   0x0001    An input buffer overflow has occurred. There is either no room in the input buffer,
                                or a character was received after the end-of-file (EOF) character.
          CE_OVERRUN  0x0002    A character-buffer overrun has occurred. The next character is lost.
          CE_RXPARITY 0x0004    The hardware detected a parity error.
          CE_FRAME    0x0008    The hardware detected a framing error.
          CE_BREAK    0x0010    The hardware detected a break condition.
        */
        if (!ClearCommError(hAMRCOM, &dwErrors, NULL))
        {
           printf("FAIL: ClearCommError(), error %d on line %d\n", GetLastError(), __LINE__);
           ExitThread(0);
        }
        printf("FAIL: ReadFileEx(), ClearCommError=0x%x GetLastError=%d on line %d\n", dwErrors, GetLastError(), __LINE__);
        dwBytesReadFromAMR = 0;
        SetEvent(hAMRIoEvent);
     }

     // track max. buffer usage for debug
     if ( dwMaxChunkRead < dwBytesReadFromAMR )
     {
         dwMaxChunkRead = dwBytesReadFromAMR;
         printf("INFO: AMR buff nMaxChunkRead=%d\n", dwMaxChunkRead);
     }

     nReadPos += dwBytesReadFromAMR;
     dwBytesInAMRBuff += dwBytesReadFromAMR;

     switch ( peekAMRBuff(szAMRBuff,
                          nWritePos,
                          dwBytesInAMRBuff,
                          &dwBytesToWrite) )
     {
         case WRITE_TO_CONSOLE: // wait for IO completion on Console port
                                WaitForSingleObject(hConsoleIoEvent, INFINITE);

                                // setup hConsoleCOM overlapped IO structure
                                overlappedIoConsole.Internal = 0;
                                overlappedIoConsole.InternalHigh = 0;
                                overlappedIoConsole.Offset = 0;
                                overlappedIoConsole.OffsetHigh = 0;
                                overlappedIoConsole.hEvent = hConsoleIoEvent;

                                // read data from Task com port
                                if ( WriteFileEx(hConsoleCOM,
                                                 &szAMRBuff[nWritePos],
                                                 dwBytesToWrite,
                                                 &overlappedIoConsole,
                                                 callbackAMRIoThread) )
                                {
                                    // suspend thread until IO completes
                                    SleepEx(INFINITE, TRUE);
                                }
                                else
                                {
                                    printf("FAIL: WriteFileEx(), error %d on line %d\n", GetLastError(), __LINE__);
                                    SetEvent(hConsoleIoEvent);
                                }

                                // assuming all bytes to write were in fact written
                                dwBytesInAMRBuff -= dwBytesToWrite;
                                nWritePos += dwBytesToWrite;
                                break;
                                
         case WRITE_TO_TASK:    // wait for IO completion on Task port
                                WaitForSingleObject(hTaskIoEvent, INFINITE);

                                // setup hTaskCOM overlapped IO structure
                                overlappedIoTask.Internal = 0;
                                overlappedIoTask.InternalHigh = 0;
                                overlappedIoTask.Offset = 0;
                                overlappedIoTask.OffsetHigh = 0;
                                overlappedIoTask.hEvent = hTaskIoEvent;

                                // read data from Task com port
                                if ( WriteFileEx(hTaskCOM,
                                                 &szAMRBuff[nWritePos],
                                                 dwBytesToWrite,
                                                 &overlappedIoTask,
                                                 callbackAMRIoThread) )
                                {
                                    // suspend thread until IO completes
                                    SleepEx(INFINITE, TRUE);
                                }
                                else
                                {
                                    printf("FAIL: WriteFileEx(), error %d on line %d\n", GetLastError(), __LINE__);
                                    SetEvent(hTaskIoEvent);
                                }

                                // assuming all bytes to write were in fact written
                                dwBytesInAMRBuff -= dwBytesToWrite;
                                nWritePos += dwBytesToWrite;
                                break;

         case NEED_MORE_READ:   break; // do nothing, need more characters ...
         
         default:               printf("FAIL: unhandled return from peekAMRBuff() ... terminating.\n");
                                ExitThread(1);
     } // switch on peekAMRBuff()

     // reset pointers to start of buffer if
     // read and write positions are equal
     if ( (nWritePos == nReadPos) && (nReadPos != 0) )
     {
         nWritePos        = 0;
         nReadPos         = 0;
         dwBytesInAMRBuff = 0;
     }
 }
}

/* -----------------------------------------------------
   callbackConsoleIoThread()

   ----------------------------------------------------- */
#pragma argsused

void
WINAPI callbackConsoleIoThread(DWORD        dwErrorCode,
                               DWORD        dwNumberOfBytesTransfered,
                               LPOVERLAPPED lpOverlapped)
{
 dwBytesReadFromConsole = dwNumberOfBytesTransfered;

 if ( dwErrorCode != 0 )
    {
     // error occurred. Report to user
     printf("FAIL: callbackConsoleIoThread(), error %d on line %d\n", GetLastError(), __LINE__);
    }

 SetEvent(lpOverlapped->hEvent);
}

/* -----------------------------------------------------
   threadConsole()

   ----------------------------------------------------- */
#pragma argsused

DWORD
WINAPI threadConsole(LPVOID lpParam)
{
 while (1)
    {
     // wait for IO completion on Console com port
     WaitForSingleObject(hConsoleIoEvent, INFINITE);

     // setup hConsoleCOM overlapped IO structure
     overlappedIoConsole.Internal = 0;
     overlappedIoConsole.InternalHigh = 0;
     overlappedIoConsole.Offset = 0;
     overlappedIoConsole.OffsetHigh = 0;
     overlappedIoConsole.hEvent = hConsoleIoEvent;

     // read data from Task com port
     if ( ReadFileEx(hConsoleCOM,
                     szConsoleBuff,
                     BUFFER,
                     &overlappedIoConsole,
                     callbackConsoleIoThread) )
        {
         // suspend thread until IO completes
         SleepEx(INFINITE, TRUE);
        }
     else
        {
         printf("FAIL: ReadFileEx(), error %d on line %d\n", GetLastError(), __LINE__);
         dwBytesReadFromConsole = 0;
         SetEvent(hConsoleIoEvent);
        }

     // Wait for IO completion to AMT port
     WaitForSingleObject(hAMRIoEvent, INFINITE);

     // setup hAMRCOM overlapped IO structure
     overlappedIoAMR.Internal = 0;
     overlappedIoAMR.InternalHigh = 0;
     overlappedIoAMR.Offset = 0;
     overlappedIoAMR.OffsetHigh = 0;
     overlappedIoAMR.hEvent = hAMRIoEvent;

     // write data to AMR com port
     if ( WriteFileEx(hAMRCOM,
                      szConsoleBuff,
                      dwBytesReadFromConsole,
                      &overlappedIoAMR,
                      callbackConsoleIoThread) )
        {
         // suspend thread until IO completes
         SleepEx(INFINITE, TRUE);
        }
     else
        {
         printf("FAIL: WriteFileEx(), error %d on line %d\n", GetLastError(), __LINE__);
         SetEvent(hAMRIoEvent);
        }
    }
}

/* -----------------------------------------------------
   callbackTaskIoThread()

   ----------------------------------------------------- */
#pragma argsused

void
WINAPI callbackTaskIoThread(DWORD        dwErrorCode,
                            DWORD        dwNumberOfBytesTransfered,
                            LPOVERLAPPED lpOverlapped)
{
 dwBytesReadFromTask = dwNumberOfBytesTransfered;

 if ( dwErrorCode != 0 )
    {
     // error occurred. Report to user
     printf("FAIL: callbackTaskIoThread(), error %d on line %d\n", GetLastError(), __LINE__);
    }

 SetEvent(lpOverlapped->hEvent);
}

/* -----------------------------------------------------
   threadTask()

   ----------------------------------------------------- */
#pragma argsused

DWORD
WINAPI threadTask(LPVOID lpParam)
{
 while (1)
    {
     // wait for IO completion on Task com port
     WaitForSingleObject(hTaskIoEvent, INFINITE);

     // setup hTaskCOM overlapped IO structure
     overlappedIoTask.Internal = 0;
     overlappedIoTask.InternalHigh = 0;
     overlappedIoTask.Offset = 0;
     overlappedIoTask.OffsetHigh = 0;
     overlappedIoTask.hEvent = hTaskIoEvent;

     // read data from Task com port
     if ( ReadFileEx(hTaskCOM,
                     szTaskBuff,
                     BUFFER,
                     &overlappedIoTask,
                     callbackTaskIoThread) )
        {
         // suspend thread until IO completes
         SleepEx(INFINITE, TRUE);
        }
     else
        {
         printf("FAIL: ReadFileEx(), error %d on line %d\n", GetLastError(), __LINE__);
         dwBytesReadFromTask = 0;
         SetEvent(hTaskIoEvent);
        }

     // Wait for IO completion to AMR port
     WaitForSingleObject(hAMRIoEvent, INFINITE);

     // setup hAMRCOM overlapped IO structure
     overlappedIoAMR.Internal = 0;
     overlappedIoAMR.InternalHigh = 0;
     overlappedIoAMR.Offset = 0;
     overlappedIoAMR.OffsetHigh = 0;
     overlappedIoAMR.hEvent = hAMRIoEvent;

     // write data to AMR com port
     if ( WriteFileEx(hAMRCOM,
                      szTaskBuff,
                      dwBytesReadFromTask,
                      &overlappedIoAMR,
                      callbackTaskIoThread) )
        {
         // suspend thread until IO completes
         SleepEx(INFINITE, TRUE);
        }
     else
        {
         printf("FAIL: WriteFileEx(), error %d on line %d\n", GetLastError(), __LINE__);
         SetEvent(hAMRIoEvent);
        }
    }
}

/* -----------------------------------------------------
   main()

   ----------------------------------------------------- */

/* structure of RS232 clients around redirector

   [  AMR      ]
   +-----------+
   | COM1      |
   +-----------+
         |
      [redirector]-------+
         |               |
   +-----------+    +-----------+
   | COM10     |    | COM20     |
   +-----------+    +-----------+
   [ Console   ]    [ Task      ]

*/

int main(int argc, char* argv[])
{
 int   i;
 DWORD dwWaitObjectsResult;
 
 DWORD dwBaud  = COM_BAUD;
 BYTE  bParity = COM_PARITY;
 
 printf("Built: %s, %s\n", __TIME__, __DATE__);

 // parse command line parameters:
 // comredir [-b <rate>] [-p <parity>]
 //   <rate> = 9600, 19200, 38400 (default='9600')
 //   <parity> = odd, even, none (default='none')

 // something quick and dirty to pase command line parameters...
 for ( i = 1; i < argc; i++ )
 {
    if (i + 1 != argc) // Check that we haven't finished parsing already
       if ( strcmp(argv[i], "-b") == 0 )
       {
          if ( strcmp(argv[i + 1], "19200") == 0 ) 
          {
             dwBaud = CBR_19200;
          }
          else if ( strcmp(argv[i + 1], "38400") == 0 )
          {
             dwBaud = CBR_38400;
          }
          else
          {
             dwBaud = COM_BAUD; // the default is 9600 BAUD
          }
       }
       else if ( strcmp(argv[i], "-p") == 0 )
       {
          if ( strcmp(argv[i + 1], "odd") == 0 ) 
          {
             bParity = ODDPARITY;
          }
          else if ( strcmp(argv[i + 1], "even") == 0 )
          {
             bParity = EVENPARITY;
          }
          else
          {
             bParity = COM_PARITY; // the default is 'none'
          }
       }
       else
       {
          printf("FAIL: main(), bad command line option %s\n", argv[i]);
          ExitProcess(1);
       }
 }
   
 // elevate process priority class
 SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS);
 
 // open ports and configure parameters
 // if error opening COM port exit with code 1
 if ( !openComPort(&hAMRCOM, AMR_COM, dwBaud, COM_BITS, bParity, COM_STOPBIT) )
 {
    printf("FAIL: main(), openComPort(AMR) error %d on line %d\n", GetLastError(), __LINE__);
    ExitProcess(1);
 }

 if ( !openComPort(&hConsoleCOM, CONSOLE_COM, dwBaud, COM_BITS, bParity, COM_STOPBIT) )
 {
    printf("FAIL: main(), openComPort(CONSOLE) error %d on line %d\n", GetLastError(), __LINE__);
    ExitProcess(1);
 }

 if ( !openComPort(&hTaskCOM, TASK_COM, dwBaud, COM_BITS, bParity, COM_STOPBIT) )
 {
    printf("FAIL: main(), openComPort(TASK) error %d on line %d\n", GetLastError(), __LINE__);
    ExitProcess(1);
 }

 // create event handles to use for IO syncronization
 hAMRIoEvent = CreateEvent(NULL,   // default security
                           FALSE,  // automatic reset to non-signaled
                           TRUE,   // create signaled
                           NULL);  // no name

 if ( hAMRIoEvent == NULL )
 {
    printf("FAIL: main(), CreateEvent() error %d on line %d\n", GetLastError(), __LINE__);
    ExitProcess(2);
 }

 hConsoleIoEvent = CreateEvent(NULL, FALSE, TRUE, NULL);

 if ( hConsoleIoEvent == NULL )
 {
    printf("FAIL: main(), CreateEvent() error %d on line %d\n", GetLastError(), __LINE__);
    ExitProcess(2);
 }

 hTaskIoEvent = CreateEvent(NULL, FALSE, TRUE, NULL);

 if ( hTaskIoEvent == NULL )
 {
    printf("FAIL: main(), CreateEvent() error %d on line %d\n", GetLastError(), __LINE__);
    ExitProcess(2);
 }

 // create threads and start port reading and redirection
 // if error creating thread exit with code 3
 hThreadArray[AMR] = CreateThread(NULL,                     // default security
                                  0,                        // default stack size
                                  threadAMR,                // thread function
                                  NULL,                     // argument for thread function
                                  0,                        // default create flags
                                  &dwThreadIdArray[AMR]);   // return thread id

 hThreadArray[CON] = CreateThread(NULL, 0, threadConsole, NULL, 0, &dwThreadIdArray[CON]);

 hThreadArray[TASK] = CreateThread(NULL, 0, threadTask, NULL, 0, &dwThreadIdArray[TASK]);

 for (i = 0; i < MAX_THREADS; i++)
 {
    if ( hThreadArray[i] == NULL )
    {
        printf("FAIL: main(), NULL thread handle number %d on line %d\n", i, __LINE__);
        ExitProcess(3);
    }
 }

 // wait for threads to terminate
 dwWaitObjectsResult = WaitForMultipleObjects(MAX_THREADS, hThreadArray, FALSE, INFINITE);
 switch ( dwWaitObjectsResult )
 {
    case WAIT_FAILED:
         printf("FAIL: main(), WaitForMultipleObjects(), error %d on line %d\n", GetLastError(), __LINE__);
         break;

    case WAIT_OBJECT_0:
    case (WAIT_OBJECT_0 + 1):
    case (WAIT_OBJECT_0 + 2):
         printf("FAIL: main(), WaitForMultipleObjects() thread exited=%d line %d\n", dwWaitObjectsResult - WAIT_OBJECT_0, __LINE__);
         break;

    default:
         printf("FAIL: main(), WaitForMultipleObjects() thread exit line %d\n", __LINE__);
 }

 printf("Terminating.");

 // close thread and comm handles and delete critical section object
 for (i = 0; i < MAX_THREADS; i++)
 {
    CloseHandle(hThreadArray[i]);
 }

 CloseHandle(hAMRCOM);
 CloseHandle(hConsoleCOM);
 CloseHandle(hTaskCOM);

 return 0;
}
