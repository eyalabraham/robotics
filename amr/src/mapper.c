/* ***************************************************************************

  MAPPER.C

  goal: roam area and look for define object (web cam), pick object (gripper),
  return object to starting point.

  seeker.c (v1) - original idea: read vector commands from file and play them back
                  test accuracy of platform and t_ctrlex.c for dead-reckoning

				  changed to:
				  creat 3 states: one that turns platform until sonar shows distance
				  to object less than certain threshold(1), then transition to drive forward
				  until sonar shows object at less than threshold(2), then turn again etc.
				  halt state that is reached from either a stop command or a 360 deg
				  scan with no sonar measurment higher than threshold(1).
				  where threshold(1) > threshold(2)
				  collect movement vectors, save locally for output and send via tcp to plotting program.

  Octobert 21 2012 - Created

*************************************************************************** */

#ifndef WIN32_LEAN_AND_MEAN    // see Note at http://msdn.microsoft.com/en-us/library/windows/desktop/ms737629%28v=vs.85%29.aspx
 #define WIN32_LEAN_AND_MEAN
#endif

#define  _CRT_SECURE_NO_WARNINGS // suppress compiler errors for unsafe scsnf()

#include <windows.h>
#include <WinSock2.h>
#include <Ws2tcpip.h>
#include <stdio.h>
#include <math.h>

#include "fxp.h"
#include "rc_smte.h"
#include "messages.h"
#include "names.h"

#pragma comment(lib, "Ws2_32.lib")

/* -----------------------------------------
   definitions
----------------------------------------- */

#define  G_KEY             71                          // 'go' key to start
#define  H_KEY             72                          // command go 'Home'
#define  P_KEY             80                          // 'pause' movement
#define  Q_KEY             81                          // quit

#define  PORT              "\\\\.\\COM21"              // AMR comm port

#define  BUFFER            128                         // console input buffer
#define  KEY_SCAN_TOV      50                          // time delay to read keyboard [mSec]

#define  FORWARD_SPEED     MV_SPEED_3                  // motor center speed for forward move
#define  TURN_SPEED        MV_SPEED_MIN                // motor speed for scan turn

#define  SONAR_OBSTACLE    50                          // SONAR range obstacle trigger in [cm]
#define  SONAR_RANGE       100                         // SONAR range detection for clear path [cm]
#define  DETECT_INTERVAL   200                         // SONAR detection interval [mSec]
#define  DIST_SENSOR_TOV   200                         // SONAR wait TOV [mSec]
#define  DETECTION_COUNT   2                           // measurments > SONAR_RANGE to determine clear path
#define  MOVE_TOV          500                         // TOV for reporting platform movement stats

#define  MAX_CHARS         256                         // max length of file specifier
#define  FILE_DIR          "c:\\myDocs\\~ downloads\\" // input and output file directory
#define  VECTOR_FILE       "mapper.txt"                // file to save vectors
#define  MAX_VECTORS       2                           // 2 vectors for calculations, everything will go to a file

#define  __PI__            ((float) 3.14)
#define  __2PI__           ((float)(2 * __PI__))

#define DEFAULT_IP         "localhost" // use "HOMEDESKTOP" or "IBM-7C1A90F4EC1" or IP address
#define DEFAULT_PORT       "49152"

/* -----------------------------------------
   types
----------------------------------------- */

typedef struct vector_t
{
	float fAngle;
	float fLength;
    float fAngleSum;
    float fDx;
    float fDy;
    float fX;
    float fY;
    float fOriginAngle;
    float fOriginLength;
    float fRealignAngle;
};

typedef enum {
	          STATE_HALT = 0,  // stop
			  STATE_PAUSE,
              STATE_MOVE,      // move forward
              STATE_SCAN       // turn and scan for clear path
             } State_Type;

/* -----------------------------------------
   function prototypes
----------------------------------------- */

void  CreateVector(float, float, struct vector_t*, struct vector_t*);
void  GetHomeCommand(struct vector_t*, float*, float*);
int   GetQuadrant(struct vector_t*);
void  ClearVector(struct vector_t*);
void  CopyVector(struct vector_t*, struct vector_t*);
void  PrintSendVector(float, float);
int   KeyEvent(DWORD);

void halt(void);      // state functions
void pause(void);
void move(void);
void scan(void);

/* -----------------------------------------
   globals
----------------------------------------- */

HANDLE  hStdin;                                      // for keyboard input

short int  nControlTask     = Q_EMPTY;
short int  nDistSensorTask  = Q_EMPTY;
short int  nSonarTask       = Q_EMPTY;
short int  nNavTask         = Q_EMPTY;

int    nQuit                 = FALSE;
void   (*stateTable[])(void) = { halt, pause, move, scan }; // state functions
char*  spStates[]            = {"halt", "pause", "move", "scan" };
int    currentState;
int    prevState;
int    pausedState;

FILE*  hVectorsFile;
char   text[MAX_CHARS] = {0};
int    nVector = 0;
float  fDistance;
float  fAngle;
struct vector_t vectors[MAX_VECTORS];

WSADATA wsaData;
struct  addrinfo *result = NULL, *ptr = NULL, hints;
SOCKET  UdpSocket = INVALID_SOCKET;
int     iResult;

/* -----------------------------------------
   main()
----------------------------------------- */
int main(void)
{
    int    nMainExit = 0;

	printf("BUILD: mapper.c %s, %s\n", __DATE__, __TIME__);
	printf("\t-------------------------\n");
	printf("\t 'g' - go                \n");
	printf("\t 'p' - pause             \n");
	printf("\t 'q' - quit              \n");
	printf("\t                         \n");

	// get the standard input handle
	hStdin = GetStdHandle(STD_INPUT_HANDLE);
	if ( hStdin == INVALID_HANDLE_VALUE )
	{
		printf("GetStdHandle() failed.");
		nMainExit = -1;
		goto ERR_EXIT_STDIN;
	}

	// initialize Winsock
    iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if ( iResult != 0)
    {
        printf("WSAStartup() failed with error: %d\n", iResult);
        nMainExit = -1;
		goto ERR_EXIT_WSAStartup;
    }

    ZeroMemory(&hints, sizeof (hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    hints.ai_flags = AI_PASSIVE;

    // Resolve the local address and port to be used by the server
    iResult = getaddrinfo(DEFAULT_IP, DEFAULT_PORT, &hints, &result);
    if (iResult != 0)
    {
        printf("getaddrinfo() failed with error: %d\n", iResult);
		nMainExit = -1;
		goto ERR_EXIT_GETADDRINFO;
    }

    // Create a SOCKET for the server
    UdpSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (UdpSocket == INVALID_SOCKET)
    {
        printf("socket() falied with error: %ld\n", WSAGetLastError());
        nMainExit = -1;
		goto ERR_EXIT_SOCKET;
    }

	// initialize remote connection to AMR
	if ( !SMTERC_initRemoteConnection(PORT, DEFAULT_BAUD, DEFAULT_BITS, DEFAULT_PARITY, DEFAULT_STOP) )
	{
		printf("SMTERC_initRemoteConnection() failed.");
		nMainExit = -1;
		goto ERR_EXIT_SMTERC;
	}

	// get control task's ID
	nControlTask = SMTERC_getTidByName(TASK_NAME_CONTROL);
	if ( nControlTask == 0 )
	{
		printf("SMTERC_getTidByName(%s) task ID not found.", TASK_NAME_CONTROL);
		nMainExit = -1;
		goto ERR_EXIT_GETTASK;
	}
	
	printf("INFO: control=%d\n", nControlTask);
	
	// get sonar task's ID
	nSonarTask = SMTERC_getTidByName(TASK_NAME_SONAR);
	if ( nSonarTask == 0 )
	{
		printf("SMTERC_getTidByName(%s) task ID not found.", TASK_NAME_SONAR);
		nMainExit = -1;
		goto ERR_EXIT_GETTASK;
	}
	
	printf("INFO: control=%d\n", nSonarTask);

	// get nav task's ID
	nNavTask = SMTERC_getTidByName(TASK_NAME_NAV);
	if ( nNavTask == 0 )
	{
		printf("SMTERC_getTidByName(%s) task ID not found.", TASK_NAME_NAV);
		nMainExit = -1;
		goto ERR_EXIT_GETTASK;
	}
	
	printf("INFO: control=%d\n", nSonarTask);

	// set control algorithm
	SMTERC_putMsg(nControlTask, MS_CONTROL_ALGOR, MS_PID_CTRL, DW_DONT_CARE);

    // open  file to save motion vectors
    sprintf(text, "%s%s", FILE_DIR, VECTOR_FILE);
    if ( (hVectorsFile = fopen(text, "a")) == NULL )
    {
		printf("fopen(%s) could not open vectors file.", VECTOR_FILE);
		nMainExit = -1;
		goto ERR_EXIT_GETTASK;
	}

	fprintf(hVectorsFile, "\n");
	fprintf(hVectorsFile, ";==================================\n");
	fprintf(hVectorsFile, "; %s %s\n", __DATE__, __TIME__);
	fprintf(hVectorsFile, ";==================================\n");

	// initialize vector array
	ClearVector(vectors);

	// initialize state machine
	currentState = STATE_PAUSE;
	prevState    = STATE_MOVE;

	while ( !nQuit )
	{
		if ( prevState != currentState )
		{
			printf("STATE: %s(%d)\n", spStates[currentState], currentState);
			pausedState = prevState;
            prevState = currentState;
			SMTERC_flushMsgQ();
        }
        stateTable[currentState]();
    }

	// close all open handles and exit
    fclose(hVectorsFile);

ERR_EXIT_GETTASK:
	SMTERC_deleteRemoteConnection();

ERR_EXIT_SMTERC:
	closesocket(UdpSocket);

ERR_EXIT_SOCKET:
    freeaddrinfo(result);

ERR_EXIT_GETADDRINFO:
	WSACleanup();

ERR_EXIT_WSAStartup:
	// no need to explicitly close 'hStdin'

ERR_EXIT_STDIN:
	return nMainExit;
}

/* -----------------------------------------
   halt()

   halt state, stop all motors and exit
----------------------------------------- */
void halt(void)
{
	SMTERC_putMsg(nControlTask, MV_STOP_CMD, W_DONT_CARE, DW_DONT_CARE);      // stop motor
	//SMTERC_putMsg(nDistSensorTask, SN_DIST_MODE, SN_STOW, DW_DONT_CARE);      // stop GP2D
    SMTERC_putMsg(nSonarTask, SONAR_OFF, W_DONT_CARE, DW_DONT_CARE);          // turn off SONAR detector
	printf("\texiting\n");
	nQuit = TRUE;
}

/* -----------------------------------------
   pause()

   pause state, stop all motors and wait
----------------------------------------- */
void pause(void)
{
	SMTERC_putMsg(nControlTask, MV_STOP_CMD, W_DONT_CARE, DW_DONT_CARE);      // stop motor
	//SMTERC_putMsg(nDistSensorTask, SN_DIST_MODE, SN_STOW, DW_DONT_CARE);      // stop GP2D
    SMTERC_putMsg(nSonarTask, SONAR_OFF, W_DONT_CARE, DW_DONT_CARE);          // turn off SONAR detector

	do
	{
		currentState = KeyEvent(INFINITE);
	} while ( currentState == STATE_PAUSE );

	// if halt requested then exit here
	if ( currentState == STATE_HALT )
		return;

	// regardless of which state was requested while in pause
	// resume the state that was paused
	currentState = pausedState;
	prevState = STATE_HALT;
	printf("\tresuming %s(%d)\n", spStates[currentState], currentState);
}

/* -----------------------------------------
   move()

   move forward until obstacle
----------------------------------------- */
void move(void)
{
	int    nCmd       = 0;
	WORD   wPayload;
	DWORD  dwPayload;

	float  fDistance  = 0.0;

	char   scanning[] = {"-\\|/"};
	int    i = 0;

	// turn on SONAR in collision detector mode
    printf("\tSONAR range trigger %d [cm]\n", SONAR_OBSTACLE);
	SMTERC_putMsg(nSonarTask, SONAR_ON, (WORD) SONAR_OBSTACLE, (DWORD) DETECT_INTERVAL);

	// set speed and start motors
	SMTERC_putMsg(nControlTask, MV_SPEED_CMD, FORWARD_SPEED, DW_DONT_CARE);
	SMTERC_putMsg(nControlTask, MV_FORWARD_CMD, W_DONT_CARE, DW_DONT_CARE);

	// wait for obstacle to be reported
	do
	{
		nCmd = SMTERC_waitMsg(__ANY__, DIST_SENSOR_TOV, &wPayload, &dwPayload);
		switch ( nCmd )
		{
		case Q_EMPTY:
			// no obstacle, so print something and wait again
			i++;
			if ( i == strlen(scanning) ) i = 0;
			printf("\r\tmoving %c", scanning[i]);
			break;

		case SN_OBSTACLE:
			// obstacle detected
			if ( wPayload == SONAR_DETECTED )
			{
				printf("\r\tSONAR triggered\n");
				currentState = STATE_SCAN;
				continue;
			}
			break;

		case SN_ERROR:
			// wheel sensor error reported
			printf("\r\tsensor error %d. aborting.\n", wPayload);
			currentState = STATE_HALT;
			continue;
			break;

		case MV_LINEAR_STAT:
			// collect periodic move stats
			fDistance += Qmn2FLOAT((QmnFP_t) wPayload);
			break;

		default:
			printf("\r\tdiscarded message: %d %d %lu\n", nCmd, wPayload, dwPayload);
		}

		currentState = KeyEvent(KEY_SCAN_TOV);

	} while ( currentState == STATE_MOVE );

	//if  ( currentState == STATE_HALT )
	//	return;

	SMTERC_putMsg(nControlTask, MV_STOP_CMD, W_DONT_CARE, DW_DONT_CARE); // stop motors
	SMTERC_putMsg(nSonarTask, SONAR_OFF, W_DONT_CARE, DW_DONT_CARE);     // turn off SONAR detector

	if ( SMTERC_waitMsg(MV_LINEAR_STAT, MOVE_TOV, &wPayload, &dwPayload) == Q_EMPTY )
    {
        printf("\tmove stat time-out\n");
        currentState = STATE_HALT;
    }
    else
	{
		fDistance += Qmn2FLOAT((QmnFP_t) wPayload);
		printf("\ttraveled %+.2f[cm]\n", fDistance);
		PrintSendVector(0.0, fDistance);
	}
}

/* -----------------------------------------
   scan()

   turn and scan to find new direction
   for forward move
----------------------------------------- */
void scan(void)
{
	int    nCmd           = 0;
	WORD   wPayload;
	DWORD  dwPayload;
	static short nTurnDirection = MV_LEFT_CMD;

	float  fAngle         = 0.0;

	int    nDetections    = 0;

	// alternate turn direction
	if ( nTurnDirection == MV_RIGHT_CMD )
		nTurnDirection = MV_LEFT_CMD;
	else
		nTurnDirection = MV_RIGHT_CMD;

	// set speed and start motors to turn robot
	SMTERC_putMsg(nControlTask, MV_SPEED_CMD, TURN_SPEED, DW_DONT_CARE);
	SMTERC_putMsg(nControlTask, nTurnDirection, W_DONT_CARE, DW_DONT_CARE);

	do
	{
		SMTERC_putMsg(nSonarTask, SONAR_READ, (WORD) nNavTask, DW_DONT_CARE);
		nCmd = SMTERC_waitMsg(__ANY__, (2 * DIST_SENSOR_TOV), &wPayload, &dwPayload);
		switch ( nCmd )
		{
		case Q_EMPTY:
			// sonar didn't return value, something is wrong
			printf("\tSONAR read time-out. aborting.\n");
			currentState = STATE_HALT;
			continue;
			break;

		case SONAR_DISTANCE:
			// got distance back, check if this is a clear path
			// by matching 3 consecutive measurments
			if ( wPayload >= SONAR_RANGE )
			{
				nDetections++;
				if ( nDetections == DETECTION_COUNT )
				{
					printf("\tclear path found (range=%d)\n", (int) wPayload);
					currentState = STATE_MOVE;
					continue;
				}
			}
			else
			{
				nDetections = 0;
			}
			break;

		case MV_TURN_STAT:
			// as we get turn stats, check to see if we completed 360deg
			// without finding a path. abort if this is the case.
			fAngle += Qmn2FLOAT((QmnFP_t) wPayload);
			if ( fAngle > __2PI__ )
			{
				printf("\tcompleted full circle scan without detecting path. aborting.\n");
				currentState = STATE_HALT;
				continue;
			}
			break;

		case SN_ERROR:
			// wheel sensor error reported
			printf("\tsensor error %d. aborting.\n", wPayload);
			currentState = STATE_HALT;
			continue;
			break;

		default:
			printf("\tdiscarded message: %d %d %lu\n", nCmd, wPayload, dwPayload);
		}

		currentState = KeyEvent(KEY_SCAN_TOV);

	} while ( currentState == STATE_SCAN );

	//if  ( currentState == STATE_HALT )
	//	return;

	SMTERC_putMsg(nControlTask, MV_STOP_CMD, W_DONT_CARE, DW_DONT_CARE); // stop motors
	SMTERC_putMsg(nSonarTask, SONAR_OFF, W_DONT_CARE, DW_DONT_CARE);     // turn off SONAR detector

	if ( SMTERC_waitMsg(MV_TURN_STAT, MOVE_TOV, &wPayload, &dwPayload) == Q_EMPTY )
    {
        printf("\tp-turn move stat time-out\n");
        currentState = STATE_HALT;
    }
    else
	{
		fAngle += Qmn2FLOAT((QmnFP_t) wPayload);
		printf("\tturned %+.2f[rad]\n", fAngle);
		PrintSendVector(fAngle, 0.0);
	}
}

/* -----------------------------------------
   CreateVector()

   get angle and distance travelled and 
   calculate vector data points
----------------------------------------- */
void CreateVector(float fAngle, float fDistance, struct vector_t* pVectorN, struct vector_t* pVectorN_1)
{
    /* 
           coordinate system 

                     Y ^
                       |
     cartesian (-x,+y) | cartesian (+x,+y)
     polar     (r, -t) | polar     (r, -t)
     quad #3           | quad #4
    -------------------o>>>>----------------> X
     cartesian (-x,-y) | cartesian (+x,-y)
     polar     (r, +t) | polar     (r, +t)
     quad #2           | quad #1
                       |

     (1) starting point assumes facing down the positive direction of 'X' axis
     (2) CW turn is '+', CCW turn in '-'

    */

    float fOriginAngle1;
    float fOriginAngle2;

    // store distance traveled, direction-angle and the sum of directions
    pVectorN->fAngle    = fAngle;
    pVectorN->fLength   = fDistance;
    pVectorN->fAngleSum = pVectorN_1->fAngleSum - fAngle;

    // calculate cartesian coordinates
    pVectorN->fDx       = pVectorN->fLength * cosf(pVectorN->fAngleSum);
    pVectorN->fDy       = pVectorN->fLength * sinf(pVectorN->fAngleSum);
    pVectorN->fX        = pVectorN_1->fX + pVectorN->fDx;
    pVectorN->fY        = pVectorN_1->fY + pVectorN->fDy;

    // calculate polar coordinates
    if ( GetQuadrant(pVectorN) > 0 )
    {
        pVectorN->fOriginLength = sqrtf((pVectorN->fX * pVectorN->fX) + (pVectorN->fY * pVectorN->fY));
        fOriginAngle1           = -1 * atan2f(pVectorN->fY, pVectorN->fX);
        fOriginAngle2           = -1 * atan2f(pVectorN->fY, pVectorN->fX);
        if ( fabsf(fOriginAngle1) < fabsf(fOriginAngle2) )
            pVectorN->fOriginAngle = fOriginAngle1;
        else
            pVectorN->fOriginAngle = fOriginAngle2;
    }
    else
    {
        pVectorN->fOriginLength = 0;
        pVectorN->fOriginAngle  = 0;
    }

    // calculate turn angle to realign facing start position
    pVectorN->fRealignAngle = fmodf(pVectorN->fAngleSum, __2PI__);
}

/* -----------------------------------------
   GetHomeCommand()

   calculate turn and distance command to get
   to 'home'/origin based on the location vector set
----------------------------------------- */
void GetHomeCommand(struct vector_t* pVector, float* pfTurn, float* pfDistance)
{
    *pfDistance = pVector->fOriginLength;
    if ( GetQuadrant(pVector) > 0 )
    {
        *pfTurn = pVector->fRealignAngle - __PI__ + pVector->fOriginAngle;
        *pfTurn = fmodf(*pfTurn, __2PI__);
    }
    else
        *pfTurn = pVector->fRealignAngle;
}

/* -----------------------------------------
   GetQuadrant()

   calculate the quadrant and return quadrant
   number; origin returned as 0.
----------------------------------------- */
int GetQuadrant(struct vector_t* pVector)
{
    // if both X and Y are less than FP Qmn resolution, assume we are at origin
    if (fabsf(pVector->fX) < QmnRESOLUTION && fabsf(pVector->fY) < QmnRESOLUTION )
        return 0;

    if ( pVector->fX > 0 )
        if ( pVector->fY > 0)
            return 4;           // x > 0 and y > 0
        else
            return 1;           // x > 0 and y < 0
    else
        if ( pVector->fY > 0 )
            return 3;           // x < 0 and y > 0
        else
            return 2;           // x < 0 and y < 0
}

/* -----------------------------------------
   ClearVector()

   clear values of a vector
----------------------------------------- */
void ClearVector(struct vector_t* vector)
{

	vector->fAngle        = 0;
    vector->fLength       = 0;
    vector->fAngleSum     = 0;
    vector->fDx           = 0;
    vector->fDy           = 0;
    vector->fX            = 0;
    vector->fY            = 0;
    vector->fOriginAngle  = 0;
    vector->fOriginLength = 0;
    vector->fRealignAngle = 0;
}

/* -----------------------------------------
   CopyVector()

   copy source vector to destination
----------------------------------------- */
void CopyVector(struct vector_t* dest,
                struct vector_t* source)
{
	memcpy(dest, source, sizeof(struct vector_t));
}

/* -----------------------------------------
   PrintSendVector()

   calculate, print and transmit vector
----------------------------------------- */
void  PrintSendVector(float fAngle,
	                  float fDistance)
{
	char text[BUFFER] = {0};

	CreateVector(fAngle, fDistance, &vectors[1], &vectors[0]);
	sprintf(text, "vector,%+.2f,%+.2f,%+.2f,%+.2f", vectors[0].fX, vectors[0].fY, vectors[1].fX, vectors[1].fY);
	fprintf(hVectorsFile, "%s\n", text);
	iResult = sendto(UdpSocket, text, strlen(text), 0, result->ai_addr, (int) result->ai_addrlen);
	if (iResult == SOCKET_ERROR)
	{
		printf("\tsendto() failed with error: %d\n", WSAGetLastError());
	}
	CopyVector(&vectors[0], &vectors[1]);
}

/* -----------------------------------------
   KeyEvent()

   handle keyboard events.
   
   return: target state

----------------------------------------- */
int KeyEvent(DWORD  dwDelay)
{
	int               nModeReturn = 0;
	DWORD             i;
	DWORD             dwNumRead;
	INPUT_RECORD      irInBuf[BUFFER];

	nModeReturn = currentState;

	if ( WaitForSingleObject(hStdin, dwDelay) == WAIT_OBJECT_0 )
	{
		ReadConsoleInput(hStdin, irInBuf, BUFFER, &dwNumRead);
		for ( i = 0; i < dwNumRead; i++)
		{
			if ( irInBuf[i].EventType == KEY_EVENT && irInBuf[i].Event.KeyEvent.bKeyDown )
			{
				switch ( irInBuf[i].Event.KeyEvent.wVirtualScanCode )
				{
				case G_KEY: // start moveing
					nModeReturn = STATE_MOVE;
					break;

				case H_KEY: // stop and go home
					break;

				case P_KEY: // pause
					nModeReturn = STATE_PAUSE;
					break;

				case Q_KEY: // program quit
					nModeReturn = STATE_HALT;
					break;
				
				default:
					printf("\n\t** unhandled key %d **\n", irInBuf[i].Event.KeyEvent.wVirtualScanCode);
				}
			}
		}
	}
	
	return nModeReturn;
}

