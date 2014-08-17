/* ***************************************************************************

  WALLFOLLOWER.C

  wall following app.

  2.17.2012 - Created

*************************************************************************** */

#define  _CRT_SECURE_NO_WARNINGS // suppress compiler errors for unsafe scsnf()

#include <windows.h>
#include <stdio.h>
#include <math.h>

#include "rc_smte.h"
#include "messages.h"
#include "names.h"
#include "fxp.h"
#include "platform.h"

/* -----------------------------------------
   local definitions
----------------------------------------- */

#define  FILE_NAME          "C:\\myDocs\\trace\\wallfollow" // csv format trace file
#define  TXT_BUF            80

#define  BUFFER             128        // console input buffer

#define  PORT               "\\\\.\\COM21"
#define  DETECTION_DISTANCE 30         // wall object detection/tracking distance
#define  PI                 3.1416

#define  ALIGNMENT_RES      15         // measurement steps for alignment

#define  CENTER_SPEED       MV_SPEED_2 // motor center speed
#define  LOOP_TIME          125        // PID update rate at ~8Hz - in reality it is ~7Hz
#define  DEAD_BAND          0.0
#define  INTEG_LIMIT        0.6        // 60% of full scale for integrator wind-up limit
#define  STEER_LIMIT        8          // +/- STEER_LIMIT limit on PID output, max. of 'MAX_SPREAD' in t_ctrlex.c
#define  TRACK_ANGLE        PI/3       // sensor pointing direction for tracking
#define  _Cp                0.6        // PID constants
#define  _Ci                0.0
#define  _Cd                0.4

#define  SONAR_RANGE_TRIGGGER DETECTION_DISTANCE // default SONAR range trigger in [cm]
#define  DETECT_INTERVAL      167      // SONAR detection interval [mSec]
#define  DIST_SENSOR_TOV      200      // [mSec]

#define  REALIGN_TURN       PI/2       // [rad] to trun CCW when detectinh a wall perpendicular to follow-track

#define  Q_KEY              81
#define  D_KEY              68
#define  S_KEY              83
#define  G_KEY              71

typedef struct distanceVector_t
{
	float fAngle;
	float fDistance;
};

typedef enum {
	          STATE_HALT = 0,  // stop
              STATE_SEEK,      // look for wall object to follow
              STATE_FOLLOW,    // follow wall
	          STATE_ALIGN,     // align to a wall object
              STATE_TURN       // realign to perpendicular wall
             } State_Type;

/* -----------------------------------------
   funciton prototypes
----------------------------------------- */

void halt(void);
void seek(void);
void follow(void);
void align(void);
void turn(void);

void ErrorExit(LPSTR);
int  KeyEventProc(KEY_EVENT_RECORD*, int);
int  steeringCommandPID(int, int, float, float, float, float*, float*, float*);

/* -----------------------------------------
   globals
----------------------------------------- */

FILE*   hTraceFile;
HANDLE  hStdin;
char    textBuffer[TXT_BUF] = {0};    // text buffer for traces

short int  nControlTask     = Q_EMPTY;
short int  nDistSensorTask  = Q_EMPTY;
short int  nSonarTask       = Q_EMPTY;
short int  nNav             = Q_EMPTY;

int   nQuit                 = FALSE;
void  (*stateTable[])(void) = { halt, seek, follow, align, turn }; // state function
char* spStates[]            = {"halt", "seek", "follow", "align", "turn" };
int   currentState;
int   prevState;

float fCp                   = _Cp;
float fCi                   = _Ci;
float fCd                   = _Cd;

/* -----------------------------------------
   main()
----------------------------------------- */
int
main(void)
{
 printf("BUILD: wallfollow.c %s, %s\n", __DATE__, __TIME__);
 printf("\t-------------------------\n");
 printf("\t 'Q' - quit follow state \n");
 printf("\t-------------------------\n");

 // initialize remote connection to AMR
 if ( !SMTERC_initRemoteConnection(PORT, CBR_19200, DEFAULT_BITS, DEFAULT_PARITY, DEFAULT_STOP) )
    {
     ErrorExit("SMTERC_initRemoteConnection() failed.");
    }

 // get the standard input handle
 hStdin = GetStdHandle(STD_INPUT_HANDLE);
 if ( hStdin == INVALID_HANDLE_VALUE )
    {
	 ErrorExit("GetStdHandle() failed.");
    }

 // get control task's ID
 nControlTask = SMTERC_getTidByName(TASK_NAME_CONTROL);
 if ( nControlTask == 0 )
    {
     ErrorExit("SMTERC_getTidByName() task ID not found.");
    }
 printf("INFO: control=%d\n", nControlTask);

 // get distance sensor task's ID
 nDistSensorTask = SMTERC_getTidByName(TASK_NAME_DISTANCE);
 if ( nDistSensorTask == 0 )
    {
     ErrorExit("SMTERC_getTidByName() task ID not found.");
    }
 printf("INFO: distance=%d\n", nDistSensorTask);

 // get SONAR sensor task's ID
 nSonarTask = SMTERC_getTidByName(TASK_NAME_SONAR);
 if ( nSonarTask == 0 )
    {
     ErrorExit("SMTERC_getTidByName() task ID not found.");
    }
 printf("INFO: sonar=%d\n", nSonarTask);

 // get nav task's ID
 nNav = SMTERC_getTidByName(TASK_NAME_NAV);
 if ( nNav == 0 )
    {
     ErrorExit("SMTERC_getTidByName() task ID not found.");
    }
 printf("INFO: nav=%d\n", nNav);

 // get PID constants
 printf("PID: enter PID constants <Cp=%g Ci=%g Cd=%g>: ", fCp, fCi, fCd);
 scanf("%g %g %g", &fCp, &fCi, &fCd);

 // open file to capture PID trace
 sprintf(textBuffer, "%s %g %g %g.csv", FILE_NAME, fCp, fCi, fCd);
 if ( (hTraceFile = fopen(textBuffer,"w")) == NULL )
    {
 	 ErrorExit("CreateFile() could not open trace file.");
    }

 fprintf(hTraceFile, "T,Err,Steering,P,I,D,T-convert\n");

 // set motors' center speed
 SMTERC_putMsg(nControlTask, MV_SPEED_CMD, CENTER_SPEED, DW_DONT_CARE);

 // initialize state machine
 currentState = STATE_SEEK;
 prevState    = ~STATE_SEEK;

 while ( !nQuit )
    {
     if ( prevState != currentState )
        {
         printf("STATE: %s(%d)\n", spStates[currentState], currentState);
         prevState = currentState;
        }
	 	
	 SMTERC_flushMsgQ();
     stateTable[currentState]();
    }
 
 SMTERC_deleteRemoteConnection();
 fclose(hTraceFile);

 return 0;
}

/* =========================================
   halt()

   enter this state if follow mode and seek
   mode cannot identify a wall object
========================================= */
void
halt(void)
{
	SMTERC_putMsg(nControlTask, MV_STOP_CMD, W_DONT_CARE, DW_DONT_CARE);      // stop motor
	SMTERC_putMsg(nControlTask, MS_CONTROL_ALGOR, MS_PID_CTRL, DW_DONT_CARE); // reset to PID control
	SMTERC_putMsg(nDistSensorTask, SN_DIST_MODE, SN_STOW, DW_DONT_CARE);      // stop GP2D
    SMTERC_putMsg(nSonarTask, SONAR_OFF, W_DONT_CARE, DW_DONT_CARE);          // turn off SONAR detector
	printf("\texiting\n");
	nQuit = TRUE;
}

/* =========================================
   seek()

   look for a wall object
========================================= */
void
seek(void)
{
    int   nCmd      = 0;
    WORD  wPayload  = 0;
    DWORD dwPayload = 0;

    float fDistance = 0;

	// (1) move forward while scanning for objects
	// (2) stop when object is identified
	// (3) transition to STATE_ALLIGN

	// setup GP2D to measure distance
	SMTERC_putMsg(nDistSensorTask, SN_DIST_MODE, SN_POINT, (DWORD) 0);
	Sleep(500);

	// override default control algorithm
	SMTERC_putMsg(nControlTask, MS_CONTROL_ALGOR, MS_PID_CTRL, DW_DONT_CARE);

	// start moving forward
	SMTERC_putMsg(nControlTask, MV_FORWARD_CMD, W_DONT_CARE, DW_DONT_CARE);

	while ( currentState == STATE_SEEK )
	{
		SMTERC_putMsg(nDistSensorTask, SN_DIST_MODE, SN_MEASURE, DW_DONT_CARE);

		nCmd = SMTERC_waitMsg(SN_DISTANCE, DIST_SENSOR_TOV, &wPayload, &dwPayload);
		switch ( nCmd )
		{
		case Q_EMPTY:
			printf("\tdistance sensor timed out\n");
			currentState = STATE_HALT;
			break;

		case SN_DISTANCE:
			fDistance = Qmn2FLOAT((QmnFP_t) wPayload);
			printf("\tdistance %g[cm]      \r", fDistance);
			
			if ( fDistance == 0 )
				continue;

			if ( fDistance < DETECTION_DISTANCE )
			{
				SMTERC_putMsg(nControlTask, MV_STOP_CMD, W_DONT_CARE, DW_DONT_CARE);
				printf("\n");
				if ( SMTERC_waitMsg(MV_LINEAR_STAT, 5000, &wPayload, &dwPayload) == Q_EMPTY )
				{
					printf("\tlinear move stat time-out\n");
					currentState = STATE_HALT;
				}
				else
					currentState = STATE_ALIGN;
			}
			break;

		default:
			printf("\tdiscarded message: %d %d %lu\n", nCmd, wPayload, dwPayload);
		}
	}
}

/* =========================================
   align()

   align to a wall object

   try to align to a wall object.
   if this is *not* a wall object then transition to STATE_HALT
   (try: backup, turn to random direction and seek again with STATE_SEEK)
   if this is a wall object, align and transition to STATE_FOLLOW

========================================= */
void
align(void)
{
	int   nCmd      = 0;
    WORD  wPayload  = 0;
    DWORD dwPayload = 0;

	float  fAngle;

	struct distanceVector_t vectorMeasured = {0, 0};
	struct distanceVector_t vectorCenter   = {99, 99};

	// initialize sensor direction
	SMTERC_putMsg(nDistSensorTask, SN_DIST_MODE, SN_POINT, (DWORD) 0);
	Sleep(500);

	// scan wall
	fAngle = -(PI/2);
	do
	{
		SMTERC_putMsg(nDistSensorTask, SN_DIST_MODE, SN_POINT, (DWORD) FLOAT2Qmn(fAngle)); // position sensor
		Sleep(500);

		SMTERC_putMsg(nDistSensorTask, SN_DIST_MODE, SN_MEASURE, DW_DONT_CARE);            // take measurment
		nCmd = SMTERC_waitMsg(SN_DISTANCE, 500, &wPayload, &dwPayload);
		switch ( nCmd )
		{
		case Q_EMPTY:                                                                      // sensor times-out
			printf("\tdistance sensor timed out\n");
			currentState = STATE_HALT;
			break;

		case SN_DISTANCE:                                                                  // get distance
			vectorMeasured.fDistance = Qmn2FLOAT((QmnFP_t) wPayload);                      // get measurment
			vectorMeasured.fAngle    = fAngle;

			if ( vectorMeasured.fDistance == 0 )                                           // if '0' then re-read sensor
				break;

			if ( vectorMeasured.fDistance < vectorCenter.fDistance )                       // save if this is a new minimum distance
				vectorCenter = vectorMeasured;

			printf("\tM(%.3f, %.3f) C(%.3f, %.3f)\n", vectorMeasured.fAngle, vectorMeasured.fDistance, vectorCenter.fAngle, vectorCenter.fDistance);
			fAngle += (PI/ALIGNMENT_RES);                                              // move direction to next angle
			break;

		default:
			printf("\tbad message returned %d\n", nCmd);
			currentState = STATE_HALT;
		}
	}
	while ( (fAngle <= (PI/2)) && (currentState == STATE_ALIGN) );

	// exit here if ther was a problem during wall scanning
	if ( currentState == STATE_HALT )
		return;

	// position platform
	fAngle = -1*(PI/2) + vectorCenter.fAngle;
	printf("\taligning to %.3f[rad]\n", fAngle);
	SMTERC_putMsg(nControlTask, MV_PTURN_CMD, FLOAT2Qmn(fAngle), DW_DONT_CARE);
	if ( SMTERC_waitMsg(MV_TURN_STAT, 5000, &wPayload, &dwPayload) == Q_EMPTY )
    {
        printf("\tturn stat time-out\n");
        currentState = STATE_HALT;
    }
    else
    	currentState = STATE_FOLLOW;
}

/* =========================================
   follow()

   follow a wall object
========================================= */
void
follow(void)
{
	int     nCmd      = 0;
    WORD    wPayload  = 0;
    DWORD   dwPayload = 0;

	DWORD   dwDelay;
	DWORD   dwNumRead;
	INPUT_RECORD irInBuf[BUFFER];

	int     nTrackingDistance;
	int     nSteeringK        = 0;
	int     nProcessError     = 0;
	float   fP                = 0.0;
	float   fI                = 0.0;
	float   fD                = 0.0;

	int     nCharsToPrint;
	DWORD   i;

	DWORD t, t2;

	// reset PID
	nSteeringK = steeringCommandPID(1, 0, 0, 0, 0, 0, 0, 0);

	// point sensor at the wall to the right 
	SMTERC_putMsg(nDistSensorTask, SN_DIST_MODE, SN_POINT, (DWORD) FLOAT2Qmn(TRACK_ANGLE));
	Sleep(500);

	// turn on SONAR in collision detector mode
    printf("\tSONAR range trigger %d [cm]\n", SONAR_RANGE_TRIGGGER);
	SMTERC_putMsg(nSonarTask, SONAR_ON, (WORD) SONAR_RANGE_TRIGGGER, (DWORD) DETECT_INTERVAL);

	// change control to open loop so that steering commands are used
	SMTERC_putMsg(nControlTask, MS_CONTROL_ALGOR, MS_OPEN_LOOP, DW_DONT_CARE);

	// calculate tracking distance given stop distance and tracking angle
	nTrackingDistance = (int) ((float) (DETECTION_DISTANCE + 10) / cosf(PI/2 - TRACK_ANGLE)) - 10;
	printf("\ttracking distance %d\n", nTrackingDistance);

	// start motors
	SMTERC_putMsg(nControlTask, MV_FORWARD_CMD, W_DONT_CARE, DW_DONT_CARE);

	// tracking loop
	do
	{
		t = GetTickCount();

		SMTERC_putMsg(nDistSensorTask, SN_DIST_MODE, SN_MEASURE, DW_DONT_CARE);

		nCmd = SMTERC_waitMsg(__ANY__, DIST_SENSOR_TOV, &wPayload, &dwPayload);
		
		t2 = GetTickCount() - t;

		switch ( nCmd )
		{
		case Q_EMPTY:
			printf("\tdistance sensor timed out\n");
			currentState = STATE_HALT;
			break;

		case SN_DISTANCE:
			// calculate steering with PID and send to platform
			nProcessError = nTrackingDistance - Qmn2INT((QmnFP_t) wPayload);
			nSteeringK = steeringCommandPID(0, nProcessError, fCp, fCi, fCd, &fP, &fI, &fD);
			SMTERC_putMsg(nControlTask, MV_LIN_STEER, (WORD) nSteeringK, DW_DONT_CARE);
			break;

		case SN_OBSTACLE:
            // facing perpendicular wall, motors already stopped
			if ( wPayload == SN_MID_SW_ON )
			{
				SMTERC_putMsg(nSonarTask, SONAR_OFF, W_DONT_CARE, DW_DONT_CARE); // turn off SONAR detector
				printf("\tSONAR triggered\n");
				currentState = STATE_TURN;
			}
			break;

		default:
			printf("\tdiscarded message: %d %d %lu\n", nCmd, wPayload, dwPayload);
		}

		// print trace to csv file
		nCharsToPrint = _snprintf(textBuffer, TXT_BUF, "%lu,%d,%d,%g,%g,%g,%d\n", GetTickCount(), nProcessError, nSteeringK, fP, fI, fD, t2);
		fprintf(hTraceFile, textBuffer);

		t = GetTickCount() - t;
		dwDelay = ( t < LOOP_TIME ) ? (DWORD) (LOOP_TIME - t) : 1;

		// insert delay and use this time to get keyboard input
		if ( WaitForSingleObject(hStdin, dwDelay) == WAIT_OBJECT_0 )
		{
			ReadConsoleInput(hStdin, irInBuf, BUFFER, &dwNumRead);
			for ( i = 0; i < dwNumRead; i++)
			{
				if ( irInBuf[i].EventType == KEY_EVENT )
				{
					currentState = KeyEventProc(&irInBuf[i].Event.KeyEvent, currentState);
				}
			}
		}
	}
	while ( currentState == STATE_FOLLOW );
}

/* =========================================
   turn()

   turn CCW if wall detected by SONAR sensor
   to realign to perpendicular wall
========================================= */
void
turn(void)
{
    int   nCmd      = 0;
    WORD  wPayload  = 0;
    DWORD dwPayload = 0;

    Sleep(1000);
    SMTERC_flushMsgQ();
	SMTERC_putMsg(nControlTask, MS_CONTROL_ALGOR, MS_PID_CTRL, DW_DONT_CARE);          // reset to PID control
    SMTERC_putMsg(nControlTask, MV_PTURN_CMD, FLOAT2Qmn(-REALIGN_TURN), DW_DONT_CARE); // turn CCW to perpendicular wall
    if ( SMTERC_waitMsg(MV_TURN_STAT, 5000, &wPayload, &dwPayload) == Q_EMPTY )
    {
        printf("\tp-turn move stat time-out\n");
        currentState = STATE_HALT;
    }
    else
    	currentState = STATE_FOLLOW;
}
/* -----------------------------------------
   steeringCommandPID()

   calculate steering command with PID
   algorithm.
   source: http://lorien.ncl.ac.uk/ming/digicont/digimath/dpid1.htm
   return steering command in the range of
   +/- STEER_LIMIT
----------------------------------------- */
int
steeringCommandPID(int    nReset,
                   int    nProcessError,
                   float  fCp,
			       float  fCi,
				   float  fCd,
				   float* pfP,
				   float* pfI,
				   float* pfD)
{
	 static float fErrSum = 0;
	 static float fErrk_1 = 0;

	 float        fUk;
	 float        fErrk;
	 float        fIntegLim;
	 int          nUk;

	 // reset intergrator
	 if ( nReset )
	 {
		 fErrSum = 0;
		 fErrk_1 = 0;
		 return 0;
	 }

     //*******************************************
	 // PID algorithm
	 //
	 // U(k) = Cp x Err(k) + 
	 //        Ci x [ Err(k) + ErrSum ] + 
	 //        Cd x [ Err(k) - Err(k-1) ]
	 //*******************************************
     
     fErrk = (float) -nProcessError;

	 // calculate limit to prevent "wind up"
	 fIntegLim = (STEER_LIMIT / fCi) * INTEG_LIMIT;

     // deadband filter
	 if ( abs(fErrk) <= DEAD_BAND )
	 {
		 fErrk = 0.0;
	 }

	 // clear integrator if error changes sign?

	 // sum error for integrator
	 fErrSum += fErrk;

	 // integrator limit
	 if ( fErrSum > fIntegLim ) fErrSum = fIntegLim;
	 if ( fErrSum < -fIntegLim ) fErrSum = -fIntegLim;

	 // also keep for tracing
	 *pfP = fCp * fErrk;
	 *pfI = fCi * fErrSum;
	 *pfD = fCd * (fErrk - fErrk_1);

	 // calculate PID control output
     fUk = *pfP + *pfI + *pfD;

	 fErrk_1 = fErrk;
     
     nUk = floor(fUk + 0.5);

     // **************************************************
     // *** DO NOT run line tracking in reverse travel ***
     // **************************************************

	 // keep within steering limits
	 if ( nUk < -STEER_LIMIT ) nUk = -STEER_LIMIT;
	 if ( nUk > STEER_LIMIT )  nUk = STEER_LIMIT;

     return nUk;
}

/* -----------------------------------------
   KeyEventProc()

   handle keyboard events.
   
   return:
     MODE_QUIT         -1
     MODE_NONE          0
     MODE_LINE_TRACK    1

----------------------------------------- */
int
KeyEventProc(KEY_EVENT_RECORD* pKer,
             int               nCurrentMode)
{
 int nModeReturn;
 
 nModeReturn = nCurrentMode;

 if ( pKer->bKeyDown )
 {
	 switch ( pKer->wVirtualScanCode )
	 {
		  case Q_KEY:     // flag program quit
			   nModeReturn = STATE_HALT;
               break;

          case D_KEY:     // send X_PRINT_DATA_LOG to initiate log dump
               break;

          case S_KEY:     // stop
		       break;

          case G_KEY:    // start moveing
               break;

          default:
               printf("\tunhandled key %d\n", pKer->wVirtualScanCode);
	 }
 }

 return nModeReturn;
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
