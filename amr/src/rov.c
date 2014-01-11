/* ***************************************************************************

  ROV.C

  Console app for keybord control of AMR robot; control uses arrow keys:
  UP    - forward
  DOWN  - backward
  LEFT  - left turn
  RIGHT - right turn
  'q'   - quit app.
  'd'   - initiate log dump on AMR

  The app allows user to enter/modify the default PID constants that are
  initialized on the AMR with the command.lst file.

  based on code from:
   http://msdn.microsoft.com/en-us/library/ms685035%28v=VS.85%29.aspx

  June 13 2010 - Created

*************************************************************************** */

#include <windows.h>
#include <stdio.h>
#include <math.h>

#include "fxp.h"
#include "rc_smte.h"
#include "messages.h"
#include "names.h"

/* -----------------------------------------
   local definitions
----------------------------------------- */

#define  SLEW_RATE 500        // mSec interval to increase motor speed
#define  MAX_SPEED MV_SPEED_3 // max running speed

#define  PORT      "\\\\.\\COM21"

#define  BUFFER    128

#define  LEFT_KEY  37 // keyboard scan codes
#define  UP_KEY    38
#define  RIGHT_KEY 39
#define  DOWN_KEY  40
#define  Q_KEY     81
#define  D_KEY     68
#define  R_KEY     82
#define  F_KEY     70
#define  B_KEY     66

#define  TEST_MOVE 3200

/* -----------------------------------------
   funciton prototypes
----------------------------------------- */

void ErrorExit(LPSTR);
BOOL KeyEventProc(KEY_EVENT_RECORD*);
void MouseEventProc(MOUSE_EVENT_RECORD*);
void ResizeEventProc(WINDOW_BUFFER_SIZE_RECORD*);
BOOL SendCommand(int);
BOOL SlewMotor(int);

/* -----------------------------------------
   globals
----------------------------------------- */

int   nControlTask   = Q_EMPTY;
int   nActiveCommand = 0;

/* -----------------------------------------
   main()
----------------------------------------- */
int
main(VOID)
{
 HANDLE       hStdin;
 DWORD        cNumRead,
              fdwMode,
              fdwSaveOldMode,
              i;
 INPUT_RECORD irInBuf[BUFFER];

 float        fCp = 0;
 float        fCi = 0;
 float        fCd = 0;

 DWORD        dwPayload;

 BOOL         fQuit = FALSE;

 printf("BUILD: rov.c %s, %s\n", __DATE__, __TIME__);

 // Get the standard input handle.
 hStdin = GetStdHandle(STD_INPUT_HANDLE);
 if (hStdin == INVALID_HANDLE_VALUE)
    ErrorExit("GetStdHandle() failed.");

 // Save the current input mode, to be restored on exit.
 if (! GetConsoleMode(hStdin, &fdwSaveOldMode) )
    ErrorExit("GetConsoleMode() failed.");

 // Enable the window and mouse input events.
 // fdwMode = ENABLE_WINDOW_INPUT | ENABLE_MOUSE_INPUT;
 // if (! SetConsoleMode(hStdin, fdwMode) )
 //    ErrorExit("SetConsoleMode() failed");

 // initialize remote connection to AMR
 if ( !SMTERC_initRemoteConnection(PORT,
                                   CBR_19200, //DEFAULT_BAUD,
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

 printf("INFO: control = %d\n", nControlTask);

 // get PID constants and use to overwrite defaults
 printf("enter PID constants as Q10.5 fixed point <Cp Ci Cd>: ");
 scanf("%g %g %g", &fCp, &fCi, &fCd);
 PACK_DW(dwPayload, (WORD) FLOAT2Qmn(fCi), (WORD) FLOAT2Qmn(fCd));
 SMTERC_putMsg(nControlTask, MS_PID, (WORD) FLOAT2Qmn(fCp), dwPayload);

 printf("PID constants: Cp=0x%x Ci=0x%x Cd=0x%x\n", (WORD) FLOAT2Qmn(fCp), (WORD) FLOAT2Qmn(fCi), (WORD) FLOAT2Qmn(fCd));

 // override default control algorithm
 SMTERC_putMsg(nControlTask,
               MS_CONTROL_ALGOR,
               MS_PID_CTRL,       // or N_OPEN_LOOP with steering
               DW_DONT_CARE);

 // set default motor speed
 SMTERC_putMsg(nControlTask, MV_SPEED_CMD, MV_SPEED_MIN, DW_DONT_CARE);

 printf("INFO: use arrow keys to control\n");
 printf("INFO: 'D'   - intiate log dump\n");
 printf("INFO: 'R'   - reset data log\n");
 printf("INFO: 'F'   - test run forward\n");
 printf("INFO: 'B'   - test run backward\n");
 printf("INFO: 'Q'   - quit\n");

 // Loop to read and handle the input events.
 do
    {
     // Wait for the events.
     if (! ReadConsoleInput(hStdin,      // input buffer handle
                            irInBuf,     // buffer to read into
                            BUFFER,      // size of read buffer
                            &cNumRead) ) // number of records read
        {
         ErrorExit("ReadConsoleInput()");
        }
     // Dispatch the events to the appropriate handler.
     for ( i = 0; i < cNumRead; i++)
        {
         switch ( irInBuf[i].EventType )
            {
             case KEY_EVENT:   // keyboard input
                  fQuit = KeyEventProc(&irInBuf[i].Event.KeyEvent);
                  break;

             case WINDOW_BUFFER_SIZE_EVENT: // ignore screen buf. resizing
             case MOUSE_EVENT:              // ignore mouse input
             case FOCUS_EVENT:              // ignore focus events
             case MENU_EVENT:               // ignore menu events
                  break;

             default:
                  ErrorExit("Unknown event type");
                  break;
            } // end switch on EventType
        }
    }
 while ( !fQuit );

 printf("INFO: exiting\n");
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

/* -----------------------------------------
   SendCommand()

   send command to AMR, print it on console

   Return:
    TRUE of success FALSE on failure
----------------------------------------- */
BOOL
SendCommand(int nCommand)
{
 if ( nCommand == nActiveCommand )
    return TRUE;

 //printf("SendCommand(%d)\n", bCommand);

 nActiveCommand = nCommand;

 return SMTERC_putMsg(nControlTask,
                      (short int) nCommand,
                      W_DONT_CARE,
                      DW_DONT_CARE);
}

/* -----------------------------------------
   SlewMotor()

   slew motor speed
----------------------------------------- */
BOOL
SlewMotor(int nCommand)
{
 static DWORD dwTimeDelta = 0;
 static BYTE  bSpeed      = MV_SPEED_MIN;

 // for stop command, reset speed and send stop
 if ( nCommand == MV_STOP_CMD )
 {
  bSpeed = MV_SPEED_MIN;
  return SendCommand(MV_STOP_CMD);
 }

 // for move command, set speed and send move
 if ( SLEW_RATE <= (GetTickCount() - dwTimeDelta))
 {
  if ( bSpeed <= MAX_SPEED )
  {
   SMTERC_putMsg(nControlTask, MV_SPEED_CMD, bSpeed, DW_DONT_CARE);
   bSpeed++;
   dwTimeDelta = GetTickCount();
   return SendCommand(nCommand);
  }
 }

 return FALSE;
}

/* -----------------------------------------
   KeyEventProc()

   handle keyboard events
----------------------------------------- */
BOOL
KeyEventProc(KEY_EVENT_RECORD* pKer)
{
 BOOL fReturn = FALSE;

 //printf("Key event: ");

 if(pKer->bKeyDown)
    {
     //printf("key pressed %c %d\n", pKer->uChar.AsciiChar, pKer->wVirtualKeyCode);
     switch ( pKer->wVirtualKeyCode )
        {
         case Q_KEY:     // flag program quit
              SlewMotor(MV_STOP_CMD);
              fReturn = TRUE;
              break;
         case D_KEY:     // print log dump
              SMTERC_putMsg(nControlTask, MS_DATA_LOG, MS_PRINT, DW_DONT_CARE);
              printf("INFO: log dump initiated\n");
              break;
         case F_KEY:     // test run forward
              SMTERC_putMsg(nControlTask, MV_SPEED_CMD, MV_SPEED_2, DW_DONT_CARE);
              SMTERC_putMsg(nControlTask, MV_LINEAR_CMD, TEST_MOVE, DW_DONT_CARE);
              SMTERC_putMsg(nControlTask, MV_SPEED_CMD, MV_SPEED_MIN, DW_DONT_CARE);
              break;
         case B_KEY:     // test run backward
              SMTERC_putMsg(nControlTask, MV_SPEED_CMD, MV_SPEED_2, DW_DONT_CARE);
              SMTERC_putMsg(nControlTask, MV_LINEAR_CMD, -TEST_MOVE, DW_DONT_CARE);
              SMTERC_putMsg(nControlTask, MV_SPEED_CMD, MV_SPEED_MIN, DW_DONT_CARE);
              break;
         case R_KEY:     // reset log buffer
              SMTERC_putMsg(nControlTask, MS_DATA_LOG, MS_RESET, DW_DONT_CARE);
              printf("INFO: reset data log\n");
              break;
         case UP_KEY:    // move forward
              SlewMotor(MV_FORWARD_CMD);
              break;
         case DOWN_KEY:  // move backward
              SlewMotor(MV_BACK_CMD);
              break;
         case LEFT_KEY:  // turn left
              SlewMotor(MV_LEFT_CMD);
              break;
         case RIGHT_KEY: // turn right
              SlewMotor(MV_RIGHT_CMD);
              break;
         default: ;
        }
    }
 else
    {
     //printf("key released %c %d\n", pKer->uChar.AsciiChar, pKer->wVirtualKeyCode);
     switch ( pKer->wVirtualKeyCode )
        {
         case UP_KEY:    // stop if any key was released
         case DOWN_KEY:
         case LEFT_KEY:
         case RIGHT_KEY:
              SlewMotor(MV_STOP_CMD);
              break;
         default: ;
        }
    }

 return fReturn;
}

/* -----------------------------------------
   MouseEventProc()

   handle mouse events
----------------------------------------- */
void
MouseEventProc(MOUSE_EVENT_RECORD* pMer)
{
#ifndef MOUSE_WHEELED
#define MOUSE_WHEELED  0x0004
#endif

#ifndef MOUSE_HWHEELED
#define MOUSE_HWHEELED 0x0008
#endif

 printf("Mouse event: ");

 switch(pMer->dwEventFlags)
    {
     case 0:
          printf("button press\n");
          break;
     case DOUBLE_CLICK:
          printf("double click\n");
          break;
     case MOUSE_HWHEELED:
          printf("horizontal mouse wheel\n");
          break;
     case MOUSE_MOVED:
          printf("mouse moved\n");
          break;
     case MOUSE_WHEELED:
          printf("vertical mouse wheel\n");
          break;
     default:
          printf("unknown\n");
          break;
    }
}

/* -----------------------------------------
   ResizeEventProc()

   Screen buffer resizing event
----------------------------------------- */
void
ResizeEventProc(WINDOW_BUFFER_SIZE_RECORD* pWbsr)
{
 printf("Resize event\n");
}


