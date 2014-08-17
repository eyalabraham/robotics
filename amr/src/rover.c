/* ***************************************************************************

  ROV.C

  Console app for keybord control of AMR robot; control uses arrow keys:
  UP    - forward
  DOWN  - backward
  LEFT  - left turn
  RIGHT - right turn
  's'   - scan with GP2D
  'q'   - quit app.
  
  The applicaton uses OpenCV to display webcam picture with sensor information
  overlayed on the video.
  Application will overlay the following sensor information:
  - Up/Down/Left/Right pointing arrow to indicate direction of travel
  - Travelled distance for forward and back
  - Degree turn for left and right turns
  - No movement will display 'NO POWER'
  - SONAR ping distance in [cm] updating every 1 sec
  - GP2D distance scan of 180 deg front of robot
  - Travel sleed

  August 4, 2014 - Created

*************************************************************************** */

#define  _CRT_SECURE_NO_WARNINGS // suppress compiler errors for unsafe scsnf()

//#undef	 SMTE_RC
#define  SMTE_RC

#include <windows.h>
#include <stdio.h>
#include <math.h>

#include "cv.h"
#include "highgui.h"

#include "fxp.h"
#include "rc_smte.h"
#include "messages.h"
#include "names.h"

/* -----------------------------------------
   local definitions
----------------------------------------- */

#define  PORT      "\\\\.\\COM21"  // SMTE serial port name

#define  KEY_INPUT_WAIT 30         // mSec to wait for keyboard input
#define  LEFT_KEY  2424832         // keyboard scan codes
#define  UP_KEY    2490368
#define  RIGHT_KEY 2555904
#define  DOWN_KEY  2621440
#define  S_KEY     115
#define  Q_KEY     113

#define  SLEW_RATE 100             // mSec interval to increase motor speed
#define  MAX_SPEED MV_SPEED_4      // max running speed

#define  SONAR_PING_INTERVAL 1000  // mSec ping interval

#define  STRING_BUF 20			   // string buffer for frame text

#define  CAM_FRAME_WIDTH  480

#define  RAD_TO_DEG       57.2957795
#define  PI               3.1416
#define  SCAN_RES         48       // scanning steps/resolution
#define  START_PIX        0        // determine to match GP2D scan range
#define  END_PIX          CAM_FRAME_WIDTH
#define  PROXIMITY_LIMIT  20       // proximity line in [cm]
typedef struct distanceVector_t    // GP2D measurements
{
	float fAngle;
	float fDistance;
};

/* -----------------------------------------
   funciton prototypes
----------------------------------------- */

int  SendCommand(int);
int  SlewMotor(int);

/* -----------------------------------------
   globals
----------------------------------------- */

// smte_rc
short int  nControlTask     = Q_EMPTY;
short int  nDistSensorTask  = Q_EMPTY;
short int  nSonarTask       = Q_EMPTY;
short int  nNavTask         = Q_EMPTY;

/* -----------------------------------------
   main()
----------------------------------------- */
int
main(void)
{
	int     i;
	int		nMainExit = 0;
	int		nKey = 0;

	int		nGp2dScanInProg = 0;
	float	fAngle		    = 0;
	int     nScanStep       = 0;
	struct  distanceVector_t vectorMeasured[SCAN_RES];
	int     nWaitForMeasurement = 0;
	int		nActiveCommand  = MV_STOP_CMD;
	int		nSonarDistance  = 0;
	float   fIrDistance     = 0;
	int     nDrawProximity  = 0;
	float	fPlatformMove   = 0;
	int		nObstacle       = 0;
	int     nSensorErr      = 0;

	DWORD	dwLastPingTime  = 0;

	int		nCmdInfo;
	WORD    wPayload;
	DWORD   dwPayload;

	int     nStartPix = START_PIX;
	int     nEndPix   = END_PIX;
	int     nSegLen   = 1;
	int     nX, nY, nProxLim;

	float   fCp = 0;
	float   fCi = 0;
	float   fCd = 0;

	IplImage*  frame;
	CvCapture* capture;
	CvFont     font;

	char	sTextMove[STRING_BUF+1] = {0};
	char	sTextSonar[STRING_BUF+1] = {0};

#ifdef SMTE_RC

	// initialize remote connection to AMR
	if ( !SMTERC_initRemoteConnection(PORT, DEFAULT_BAUD, DEFAULT_BITS, DEFAULT_PARITY, DEFAULT_STOP) )
	{
		printf("SMTERC_initRemoteConnection() failed.\n");
		nMainExit = -1;
		goto ERR_EXIT_SMTERC;
	}
	
	printf("Remote SMTE connection established.\n");

	// get control task's ID
	nControlTask = SMTERC_getTidByName(TASK_NAME_CONTROL);
	if ( nControlTask == 0 )
	{
		printf("SMTERC_getTidByName(%s) task ID not found.\n", TASK_NAME_CONTROL);
		nMainExit = -1;
		goto ERR_EXIT_GETTASK;
	}
	
	printf(" control=%d\n", nControlTask);
	
	// get sonar task's ID
	nSonarTask = SMTERC_getTidByName(TASK_NAME_SONAR);
	if ( nSonarTask == 0 )
	{
		printf("SMTERC_getTidByName(%s) task ID not found.\n", TASK_NAME_SONAR);
		nMainExit = -1;
		goto ERR_EXIT_GETTASK;
	}
	
	printf(" sonar=%d\n", nSonarTask);

	// get GP2D sensor task's ID
	nDistSensorTask = SMTERC_getTidByName(TASK_NAME_DISTANCE);
	if ( nDistSensorTask == 0 )
	{
		printf("SMTERC_getTidByName(%s) task ID not found.\n", TASK_NAME_DISTANCE);
		nMainExit = -1;
		goto ERR_EXIT_GETTASK;
	}
	
	// initialize sensor direction
	SMTERC_putMsg(nDistSensorTask, SN_DIST_MODE, SN_POINT, (DWORD) FLOAT2Qmn(-PI/2));

	printf(" distance=%d\n", nDistSensorTask);

	// get nav task's ID
	nNavTask = SMTERC_getTidByName(TASK_NAME_NAV);
	if ( nNavTask == 0 )
	{
		printf("SMTERC_getTidByName(%s) task ID not found.\n", TASK_NAME_NAV);
		nMainExit = -1;
		goto ERR_EXIT_GETTASK;
	}

	printf(" nav=%d\n", nNavTask);

	// set control algorithm
	/*
	// get PID constants and use to overwrite defaults
	printf("enter PID constants as Q10.5 fixed point <Cp Ci Cd>: ");
	scanf("%g %g %g", &fCp, &fCi, &fCd);
	PACK_DW(dwPayload, (WORD) FLOAT2Qmn(fCi), (WORD) FLOAT2Qmn(fCd));
	SMTERC_putMsg(nControlTask, MS_PID, (WORD) FLOAT2Qmn(fCp), dwPayload);

	printf("PID constants: Cp=0x%x Ci=0x%x Cd=0x%x\n", (WORD) FLOAT2Qmn(fCp), (WORD) FLOAT2Qmn(fCi), (WORD) FLOAT2Qmn(fCd));
	*/
	// override default control algorithm
	SMTERC_putMsg(nControlTask,
               MS_CONTROL_ALGOR,
               MS_PID_CTRL,       // MS_PID_CTRL or MS_OPEN_LOOP with steering
               DW_DONT_CARE);

	// set default motor speed
	SMTERC_putMsg(nControlTask, MV_SPEED_CMD, MV_SPEED_MIN, DW_DONT_CARE);

#else
    printf("No SMTE_RC connection.\n");
#endif // SMTE_RC

	// initialize the camera interface for capturing
    capture = cvCaptureFromCAM(0);
    if ( !capture )
	{
		printf("cvCaptureFromCAM() failed.\n");
		nMainExit = -1;
		goto ERR_EXIT_CAM;
	}

	// show display banner
	printf("Build: rover.c %s, %s\n", __DATE__, __TIME__);
	printf("\t-------------------------\n");
	printf("\t UP    arrow - forward   \n");
	printf("\t DOWN  arrow - back      \n");
	printf("\t LEFT  arrow - left      \n");
	printf("\t RIGHT arrow - right     \n");
	printf("\t 's'         - scan      \n");
	printf("\t 'q'         - quit      \n");
	printf("\t                         \n");

    // initialize webcam and dislay windows
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, CAM_FRAME_WIDTH);
	cvNamedWindow("cam", CV_WINDOW_AUTOSIZE);
    cvInitFont(&font, CV_FONT_HERSHEY_DUPLEX, 0.75, 0.75, 0.0, 1, 8);

	frame = cvQueryFrame(capture);
	nStartPix = 0;
	nEndPix = frame->width;
	nSegLen = frame->width / SCAN_RES;
	nProxLim = (frame->height / 2) - PROXIMITY_LIMIT;

	while ( !nMainExit )
	{
		/* ------------------------------
		   process sensors
		   ------------------------------ */

		// SONAR ping every SONAR_PING_INTERVAL
		if ( SONAR_PING_INTERVAL <= (GetTickCount() - dwLastPingTime))
		{
			SMTERC_putMsg(nSonarTask, SONAR_READ, (WORD) nNavTask, DW_DONT_CARE);
			dwLastPingTime = GetTickCount();
		}

		// GP2D scanning
		if ( nGp2dScanInProg && !nWaitForMeasurement )
		{
			SMTERC_putMsg(nDistSensorTask, SN_DIST_MODE, SN_POINT, (DWORD) FLOAT2Qmn(fAngle)); // position sensor
			Sleep(100);
			SMTERC_putMsg(nDistSensorTask, SN_DIST_MODE, SN_MEASURE, DW_DONT_CARE);            // take measurement
			nWaitForMeasurement = 1;
			fAngle += (PI/SCAN_RES);                                                           // increment angle
			if (fAngle > (PI/2))  // stop scanning is reached end
			{
				SMTERC_putMsg(nDistSensorTask, SN_DIST_MODE, SN_POINT, (DWORD) FLOAT2Qmn(-PI/2));
				nGp2dScanInProg = 0;
			}
		}

		// process AMR responses
		switch ( SMTERC_getMsg(&nCmdInfo, &wPayload, &dwPayload) )
		{
		case Q_EMPTY: // do nothing
			break;

		case SONAR_DISTANCE: // sonar distance response
			nSonarDistance = (int) wPayload;
			break;

		case MV_LINEAR_STAT: // accumulate travel distance
		case MV_TURN_STAT:
			fPlatformMove += Qmn2FLOAT((QmnFP_t) wPayload);
			break;

		case SN_OBSTACLE: // physical obstacle detected abort
			SlewMotor(MV_STOP_CMD);
			nActiveCommand = MV_STOP_CMD;
			nObstacle = 1;
			break;

		case SN_DISTANCE: // GP2D distance measurments
			vectorMeasured[nScanStep].fDistance = Qmn2FLOAT((QmnFP_t) wPayload);
			vectorMeasured[nScanStep].fAngle    = fAngle;
			//printf("%d %.2f %.2f\n", nScanStep, vectorMeasured[nScanStep].fAngle, vectorMeasured[nScanStep].fDistance);
			nScanStep++;
			if ( nScanStep >= SCAN_RES )
				nScanStep = SCAN_RES - 1;
			nWaitForMeasurement = 0;
			break;

		case SN_ERROR:
			nSensorErr = 1;
		default:      // unexpected response
			break;
		}

		/* ------------------------------
		   process video frame
		   ------------------------------ */

		// capture a frame
		frame = cvQueryFrame(capture);
		if ( !frame )
		{
			printf("cvQueryFrame() failed.");
			nMainExit = -1;
			break;
		}

		// overlay sensor data on image

		// draw center line
		cvLine(frame, cvPoint(frame->width / 2, 0), cvPoint(frame->width / 2, frame->height), cvScalar(255, 255, 255, 0), 1, 8, 0);

		//  nObstacle: hit physical obstacle
		if ( nObstacle )
			cvPutText(frame, "OBSTACLE", cvPoint(2, (frame->height / 2) - 40), &font, cvScalar(255, 0, 0, 0));

		//  nActiveCommand: movement direction symbol
		//  fPlatformMove: floating point for turn in [rad] or distance in [cm]
		switch ( nActiveCommand )
		{
		case MV_FORWARD_CMD:
			sprintf(sTextMove,"%.1f[m] Fwd", fPlatformMove / 100);
			break;

		case MV_BACK_CMD:
			sprintf(sTextMove,"%.1f[m] Rev", fabs(fPlatformMove) / 100);
			break;

		case MV_RIGHT_CMD:
			sprintf(sTextMove,"%.1f[deg] R", fPlatformMove * RAD_TO_DEG);
			break;

		case MV_LEFT_CMD:
			sprintf(sTextMove,"%.1f[deg] L", fabs(fPlatformMove) * RAD_TO_DEG);
			break;

		case MV_STOP_CMD:
			break;

		default:
			sprintf(sTextMove, "ERROR");
		}

		if ( !nSensorErr )
			cvPutText(frame, sTextMove, cvPoint(2, frame->height-40), &font, cvScalar(255, 255, 255, 0));
		else
			cvPutText(frame, "NO POWER", cvPoint(2, frame->height-40), &font, cvScalar(0, 0, 255, 0));

		//  nSonarDistance: SONAR ping distance in [cm]
		sprintf(sTextSonar,"%d[cm]",nSonarDistance);
		cvPutText(frame, sTextSonar, cvPoint(2, frame->height-10), &font, cvScalar(255, 255, 255, 0));

		//  fIrDistance: plot GP2D distance measurment
		if ( nGp2dScanInProg )
		{
			cvPutText(frame, "SCANNING", cvPoint(2, frame->height-70), &font, cvScalar(255, 255, 255, 0));
		}

		if ( nDrawProximity )
		{
			for (i = 0; i < nScanStep; i++)
			{
				nX = nStartPix + (i * nSegLen);
				nY = (frame->height / 2) - (int) vectorMeasured[i].fDistance;
				cvLine(frame, cvPoint(nX,nY), cvPoint(nX+nSegLen, nY), cvScalar(0, 255, 0, 0), 2, 8, 0);
				cvLine(frame, cvPoint(0, nProxLim), cvPoint(frame->width, nProxLim), cvScalar(0, 0, 255, 0), 1, 8, 0);
			}
		}

		// show image
		cvShowImage("cam", frame);

		/* ------------------------------
		   process user input
		   ------------------------------ */

		// pause for user input
		nKey = cvWaitKey(KEY_INPUT_WAIT);
		if ( nKey == -1 )
			continue;

		if ( nKey == Q_KEY )
		{
			SlewMotor(MV_STOP_CMD);
			nActiveCommand = MV_STOP_CMD;
			nMainExit = -1;
			continue;
		}

		if ( nActiveCommand == MV_STOP_CMD && !nGp2dScanInProg )
		{
			fPlatformMove = 0;
			nDrawProximity = 0;
			switch ( nKey )
			{
				case UP_KEY: // move forward
					SlewMotor(MV_FORWARD_CMD);
					nActiveCommand = MV_FORWARD_CMD;
					nSensorErr = 0;
					break;

				case DOWN_KEY: // move back
					SlewMotor(MV_BACK_CMD);
					nActiveCommand = MV_BACK_CMD;
					nSensorErr = 0;
					break;

				case RIGHT_KEY: // turn right
					SlewMotor(MV_RIGHT_CMD);
					nActiveCommand = MV_RIGHT_CMD;
					nSensorErr = 0;
					break;

				case LEFT_KEY: // turn left
					SlewMotor(MV_LEFT_CMD);
					nActiveCommand = MV_LEFT_CMD;
					nSensorErr = 0;
					break;

				case S_KEY: // GP2D scan
					fAngle = -(PI/2);
					nScanStep = 0;
					nGp2dScanInProg = 1;
					nDrawProximity = 1;
					nWaitForMeasurement = 0;
					break;

				default: // unhandled key
					printf("key %d/'%c' has no function\n", nKey, nKey);
					break;
			}
		}
		else
		{
			SlewMotor(MV_STOP_CMD);
			nActiveCommand = MV_STOP_CMD;
		}
	}

	// relase resources and exit
	cvDestroyAllWindows();
    cvReleaseCapture(&capture);

	SMTERC_putMsg(nDistSensorTask, SN_DIST_MODE, SN_POINT, (DWORD) 0);
	SMTERC_putMsg(nControlTask, MV_STOP_CMD, W_DONT_CARE, DW_DONT_CARE);      // stop motor, just in case...
	SMTERC_putMsg(nControlTask, MS_CONTROL_ALGOR, MS_PID_CTRL, DW_DONT_CARE); // reset to PID control
	SMTERC_putMsg(nDistSensorTask, SN_DIST_MODE, SN_STOW, DW_DONT_CARE);      // stop GP2D
    SMTERC_putMsg(nSonarTask, SONAR_OFF, W_DONT_CARE, DW_DONT_CARE);          // turn off SONAR detector

ERR_EXIT_CAM:

ERR_EXIT_GETTASK:
	SMTERC_flushMsgQ();
	SMTERC_deleteRemoteConnection();

ERR_EXIT_SMTERC:
	// no need to explicitly close 'hStdin'

	printf("exiting\n");
	return nMainExit;
}

/* -----------------------------------------
   SendCommand()

   send command to AMR control task

   Return:
    TRUE of success FALSE on failure
----------------------------------------- */
int
SendCommand(int nCommand)
{
 //printf("SendCommand(%d)\n", bCommand);

 return SMTERC_putMsg(nControlTask,
                      (short int) nCommand,
                      W_DONT_CARE,
                      DW_DONT_CARE);
}

/* -----------------------------------------
   SlewMotor()

   slew motor speed
----------------------------------------- */
int
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
 while ( SLEW_RATE <= (GetTickCount() - dwTimeDelta))
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