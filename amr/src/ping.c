/* ***************************************************************************

  PING.C

  Console app for testing communication timing between laptop and V25

  March 30 2012 - Created

*************************************************************************** */

#include <windows.h>
#include <stdio.h>

#include "rc_smte.h"
#include "messages.h"
#include "names.h"

/* -----------------------------------------
   local definitions
----------------------------------------- */

#define  SLEW_RATE 500        // mSec interval to increase motor speed
#define  MAX_SPEED MV_SPEED_4 // max running speed

#define  PORT   "\\\\.\\COM21"

#define  BUFFER    128

/* -----------------------------------------
   funciton prototypes
----------------------------------------- */

void ErrorExit(LPSTR);

/* -----------------------------------------
   main()
----------------------------------------- */
int
main(void)
{
 int   nControlTask;
 int   nNavStubTask;

 DWORD  t0,
        t1,
        dt,
        dwMin         = 999,
        dwMax         = 0,
        dwSum         = 0,
        dwLoop        = 0,
        dwAverageTime;

 int    nMsg;
 WORD   wPayload;
 DWORD  dwPayload;

 HANDLE       hStdin;
 DWORD        dwNumRead,
              fdwSaveOldMode,
              i;
 INPUT_RECORD irInBuf[BUFFER];

 BOOL         fQuit = FALSE;

 printf("BUILD: ping.c %s, %s\n", __DATE__, __TIME__);

 //printf("BYTE %d WORD %d DWORD %d short int %d\n", sizeof(BYTE), sizeof(WORD), sizeof(DWORD), sizeof(short int));

 // Get the standard input handle.
 hStdin = GetStdHandle(STD_INPUT_HANDLE);
 if (hStdin == INVALID_HANDLE_VALUE)
    ErrorExit("GetStdHandle() failed.");

 // Save the current input mode, to be restored on exit.
 if (! GetConsoleMode(hStdin, &fdwSaveOldMode) )
    ErrorExit("GetConsoleMode() failed.");

 // initialize remote connection to AMR
 if ( !SMTERC_initRemoteConnection(PORT,
                                   CBR_19200, // DEFAULT_BAUD,
                                   DEFAULT_BITS,
                                   DEFAULT_PARITY,
                                   DEFAULT_STOP) )
    {
     ErrorExit("SMTERC_initRemoteConnection() failed.");
    }

 // get control task's ID
 nControlTask = SMTERC_getTidByName(TASK_NAME_CONTROL);
 if ( nControlTask == 0 )
    {
     SMTERC_deleteRemoteConnection();
     ErrorExit("SMTERC_getTidByName() task ID not found.");
    }

 nNavStubTask = SMTERC_getTidByName(TASK_NAME_NAV);
 if ( nNavStubTask == 0 )
    {
     SMTERC_deleteRemoteConnection();
     ErrorExit("SMTERC_getTidByName() task ID not found.");
    }

 printf("control=%d\n", nControlTask);
 printf("nav stub=%d\n", nNavStubTask);
 printf("INFO: press any key to quit\n");

 // Loop to read and handle the input events.
 do
 {
     t0 = GetTickCount();
     SMTERC_putMsg(nControlTask, MS_PING, (WORD) nNavStubTask, DW_DONT_CARE);
     nMsg = SMTERC_waitMsg(MS_PING, 500, &wPayload, &dwPayload);
     if ( nMsg == Q_EMPTY )
     {
        printf("\nERR: time out\n");
        break;
     }
     t1 = GetTickCount();
     dt = t1 - t0;
     if ( dt < dwMin ) dwMin = dt;
     if ( dt > dwMax ) dwMax = dt;
     dwSum += dt;
     dwLoop++;
     dwAverageTime = dwSum / dwLoop;
     printf("\tping time %d mSec, average %d mSec, range [%d,%d] (%d)    \r", dt, dwAverageTime, dwMax, dwMin, dwLoop);

	  // insert delay and use this time to get keyboard input
	  if ( WaitForSingleObject(hStdin, 1000) == WAIT_OBJECT_0 )
	  {
	     ReadConsoleInput(hStdin, irInBuf, BUFFER, &dwNumRead);
		  for ( i = 0; i < dwNumRead; i++)
		  {
			  if ( irInBuf[i].EventType == KEY_EVENT )
			  {
				  fQuit = 1;
			  }
		  }
	  }
 }
 while ( !fQuit );

 printf("\nINFO: exiting\n");
 SMTERC_deleteRemoteConnection();

 return 0;
}

/* -----------------------------------------
   ErrorExit()

   print error string to stderr and exit
----------------------------------------- */
void
ErrorExit(LPSTR lpszMessage)
{
 printf("ERR: %s\n", lpszMessage);
 ExitProcess(1);
}


