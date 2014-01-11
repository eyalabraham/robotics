/* ***************************************************************************

  OBJTRACK.C

  object tracker

  'Q' - quit
  'G' - get object. start looking for object and return it to the 'home'/starting location
  'P' - pause the run
  'H' - stop & return home. stop seeking and return to 'home'/starting location
  'C' - calibrate object. calibrate camera on object to find and retrieve

  July 12 2012 - Created

*************************************************************************** */

#define  _CRT_SECURE_NO_WARNINGS // suppress compiler errors for unsafe scsnf()

#include <windows.h>
#include <stdio.h>

#include "cv.h"
#include "highgui.h"

#include "rc_smte.h"
#include "messages.h"
#include "names.h"
#include "fxp.h"
#include "platform.h"

/* -----------------------------------------
   funciton prototypes
----------------------------------------- */

void seek(void);          // top level state functions
void pause(void);
void home(void);
void calibrate(void);
void halt(void);

void seekWander(void);    // STATE_SEEK sub-state functions
void seekTrack(void);
void seekGrab(void);
void seekDone(void);

void mouseClick(int, int, int, int, void*);   // mouse click handler: grab pixel HSV value from video frame
int  steeringCommandPID(int, int, float, float, float, float*, float*, float*);

void ErrorExit(LPSTR);
int  KeyEventProc(int);

/* -----------------------------------------
   local definitions
----------------------------------------- */

typedef struct pixelData_T
{
    IplImage*  frame;
    int x;
    int y;
    double H;
    double S;
    double V;
};

typedef enum                // Top level states
{
    STATE_SEEK = 0,         // start seeking object and bring home
    STATE_PAUSE,            // pause execution of run
	STATE_HOME,             // stop seeking and go home
    STATE_CALIBRATE,        // calibrate to object
    STATE_HALT              // stop and exit   
} State_Type_TopLevel;

typedef enum                // STATE_SEEK sub-states
{
    STATE_SEEK_WANDER = 0,  // wander looking for object 'lock'
    STATE_SEEK_TRACK,       // object 'lock'ed track/drive to object
    STATE_SEEK_GRAB,        // grab object
    STATE_SEEK_DONE         // done seeking, go to STATE_HOME
} State_Type_SubSeek;

#define  PORT               "\\\\.\\COM21"      // AMR communication port
#define  KEY_READ_DELAY     5

#define  _Cp                0.2                 // PID constants
#define  _Ci                0.01
#define  _Cd                0.2
#define  LOOP_TIME          100                 // object track PID loop time [mSec]
#define  INTEG_LIMIT        0.6                 // 60% of full scale for integrator wind-up limit
#define  STEER_LIMIT        8                   // +/- STEER_LIMIT limit on PID output, max. of 'MAX_SPREAD' in t_ctrlex.c
#define  DEAD_BAND          0.0

#define  MAX_DELTA          50                  // max threashold-trackbar value
#define  DEFAULT_DELTA      MAX_DELTA / 2       // threshold's +/- delta

#define  ERROR_SCALE_FACTOR 8                   // scale factor for 'x' axis position error loosely based on ROI_BOXES from linetrack.c
                                                // that gives an error range of +/- 16
                                          
#define  GRABBER_LOC        10                  // grabber offset from bottom of frame in pixels
#define  CENTER_SPEED       MV_SPEED_1          // motor center speed

#define  REGION_SIZE        5                   // average square region pixel size for mouseClick() color averaging
#define  REGION_LOW         -REGION_SIZE/2
#define  REGION_HIGH        (REGION_SIZE/2) + 1 

#define  G_KEY              103
#define  P_KEY              112
#define  H_KEY              104
#define  C_KEY              99
#define  Q_KEY              113

#define  MOTOR_START_TOV    500

#define  _cvPutText(_TEXT_) cvPutText(frame, _TEXT_, cvPoint(2, frame->height-4), &font, cvScalar(0, 0, 0, 0))

/* -----------------------------------------
   globals
----------------------------------------- */

int        nControlTask  = Q_EMPTY;  // task IDs

int        nQuit         = FALSE;    // flags
int        nCalibrated   = FALSE;

IplImage*  frame;
CvCapture* capture;
CvFont     font;

int        nDelta        = DEFAULT_DELTA;
struct     pixelData_T pixelData;

void       (*topStateTable[])(void) = { seek, pause, home, calibrate, halt }; // state function
char*      spTopStates[]            = {"seek", "pause", "home", "calibrate", "halt" };
int        currentTopState;
int        prevTopState;

void       (*seekSubStateTable[])(void) = { seekWander, seekTrack, seekGrab, seekDone }; // state function
char*      spSeekSubStates[]            = { "seek-Wander", "seek-Track", "seek-Grab", "seek-Done" };
int        currentSeekSubState;
int        prevSeekSubState;

float      fCp            = _Cp;
float      fCi            = _Ci;
float      fCd            = _Cd;
float      fP             = 0.0;
float      fI             = 0.0;
float      fD             = 0.0;

/* -----------------------------------------
   main()
----------------------------------------- */
int
main(VOID)
{
    printf("Built: %s, %s\n", __DATE__, __TIME__);

    // initialize remote connection to AMR
    if ( !SMTERC_initRemoteConnection(PORT, DEFAULT_BAUD, DEFAULT_BITS, DEFAULT_PARITY, DEFAULT_STOP) )
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

    printf("INFO: control=%d\n", nControlTask);

    // set motor's center speed
    SMTERC_putMsg(nControlTask, MV_SPEED_CMD, CENTER_SPEED, DW_DONT_CARE);

    // override default control algorithm
    SMTERC_putMsg(nControlTask, MS_CONTROL_ALGOR, MS_OPEN_LOOP, DW_DONT_CARE);

    // get PID constants
    printf("PID: Cp=0.2, Ci=0.01, Cd=0.2\n");
    printf("INFO: enter PID constants <Cp Ci Cd>: ");
    scanf("%g %g %g", &fCp, &fCi, &fCd);

    // initialize the camera interface for capturing
    capture = cvCaptureFromCAM(0);
    if ( !capture )
        ErrorExit("cvCaptureFromCAM() failed.");

    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 320);

    // initialize dislay windows
    cvNamedWindow("cam", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("threshold", CV_WINDOW_AUTOSIZE);
    cvSetMouseCallback("cam", mouseClick, &pixelData);
    cvCreateTrackbar("delta", "threshold", &nDelta, MAX_DELTA, NULL);
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0.0, 1, 8);

    printf("\n");
    printf(" object tracker          \n");
    printf("-------------------------\n");
    printf(" 'G' - get object        \n");
    printf(" 'P' - pause run         \n");
    printf(" 'H' - stop & return home\n");
    printf(" 'C' - calibrate object  \n");
    printf(" 'Q' - quit              \n");
    printf("\n");

    // initialize state machine
    currentTopState = STATE_PAUSE;
    prevTopState    = STATE_HALT;

    while ( !nQuit )
    {
        if ( prevTopState != currentTopState )
        {
            printf("STATE: %s(%d)\n", spTopStates[currentTopState], currentTopState);
            prevTopState = currentTopState;
        }

        SMTERC_flushMsgQ();
        topStateTable[currentTopState]();
    }
    
    cvDestroyAllWindows();
    cvReleaseCapture(&capture);

    return 0;
}

/* =========================================
   halt()

   enter this state to halt and clean up
   system before exiting program
========================================= */
void halt(void)
{
	SMTERC_putMsg(nControlTask, MV_STOP_CMD, W_DONT_CARE, DW_DONT_CARE);      // stop motor
	SMTERC_putMsg(nControlTask, MS_CONTROL_ALGOR, MS_PID_CTRL, DW_DONT_CARE); // reset to PID control

	printf("\texiting\n");
	nQuit = TRUE;
}

/* =========================================
   seek()

   look for object and bring it back to home
========================================= */
void seek(void)
{
    if ( !nCalibrated )
    {
        printf("\tseek(): not calibrated.\n");
        currentTopState = STATE_PAUSE;
        return;
    }

    // initialize state machine
    currentSeekSubState = STATE_SEEK_WANDER;
    prevSeekSubState    = STATE_SEEK_DONE;

    while ( currentTopState == STATE_SEEK )
    {
        if ( prevSeekSubState != currentSeekSubState )
        {
            printf("\tSTATE: %s(%d)\n", spSeekSubStates[currentSeekSubState], currentSeekSubState);
            prevSeekSubState = currentSeekSubState;
        }

        seekSubStateTable[currentSeekSubState]();
    }

    printf("\n");
}

/* =========================================
   pause()

   pause execution
========================================= */
void pause(void)
{
    int        nKey;
    IplImage*  frame;

    switch ( prevTopState )
    {
    case STATE_SEEK:
    case STATE_HOME:
        SMTERC_putMsg(nControlTask, MV_STOP_CMD, W_DONT_CARE, DW_DONT_CARE); // stop motor
        break;

    case STATE_PAUSE: // nothing to do only show web cam as we already paused on initial entry to this state
    case STATE_CALIBRATE:
    case STATE_HALT:
        break;

    default:
        printf("\n\tpause(): unrecognized value in 'prevState' %d\n", prevTopState);
    }
    
    do
    {
        frame = cvQueryFrame(capture);
        if ( !frame )
            ErrorExit("cvQueryFrame() failed.");

        _cvPutText("PAUSE");
        cvShowImage("cam", frame);
        nKey = cvWaitKey(30);
        if ( nKey > 0 )
            currentTopState = KeyEventProc(nKey);
    } while (currentTopState == STATE_PAUSE);
}

/* =========================================
   home()

   stop seek and go home by playing back
   movement vectors in reverse
========================================= */
void home(void)
{
    currentTopState = STATE_PAUSE;
}

/* =========================================
   calibrate()

   calibrate cam to object
========================================= */
void calibrate(void)
{
    int        nKey;
    double     t;
    
    CvMoments* moments;
    double     moment10;
    double     moment01;
    double     area;

    CvPoint    ptObject;
    CvPoint    ptLineTop;
    CvPoint    ptLineBottom;

    int        posX = 0;
    int        posY = 0;

    IplImage*  frame;
    IplImage*  imgHSV = 0;
    IplImage*  imgThreshed;

    nCalibrated = FALSE;

    // Loop to read and handle input events
    do
    {
        nKey = cvWaitKey(KEY_READ_DELAY);
        
        if ( nKey > 0 )
            currentTopState = KeyEventProc(nKey);
        
        t = (double) cvGetTickCount();

        // get a frame from the camera pointed to by 'capture'
        frame = cvQueryFrame(capture);
        if ( !frame )
            ErrorExit("cvQueryFrame() failed.");

        // one-time initialization on first frame
        if ( !imgHSV )
        {
            imgHSV = cvCreateImage(cvGetSize(frame), 8, 3);
            imgThreshed = cvCreateImage(cvGetSize(frame), 8, 1);
            moments = (CvMoments*) malloc(sizeof(CvMoments));

			pixelData.x = 0;
            pixelData.y = 0;
            pixelData.H = 0.0;
            pixelData.S = 0.0;
            pixelData.V = 0.0;

            ptLineTop.x = frame->width / 2;
            ptLineTop.y = 0;
            ptLineBottom.x = ptLineTop.x;
            ptLineBottom.y = frame->height;
        }

        // convert to HSV and calculate threshold
        cvCvtColor(frame, imgHSV, CV_BGR2HSV);

        pixelData.frame = imgHSV;

        cvInRangeS(imgHSV, cvScalar(pixelData.H - nDelta, pixelData.S - nDelta, pixelData.V - nDelta, 0),
			               cvScalar(pixelData.H + nDelta, pixelData.S + nDelta, pixelData.V + nDelta, 0), imgThreshed);

        // Calculate the moments to estimate the position of the ball
        cvMoments(imgThreshed, moments, 1);
 
        // The actual moment values
        moment10 = cvGetSpatialMoment(moments, 1, 0);
        moment01 = cvGetSpatialMoment(moments, 0, 1);
        area = cvGetCentralMoment(moments, 0, 0);

        posX = (int) moment10/area;
        posY = (int) moment01/area;

		if ( posX < 0 ) posX = 0;
		if ( posY < 0 ) posY = 0;

        ptObject.x = posX;
        ptObject.y = posY;

        // add locator graphics to display frame
        _cvPutText("CALIBRATE");
        cvLine(frame,ptLineTop, ptLineBottom, cvScalar(0, 0, 0, 0), 1, 8, 0);
        if ( posX > 0 && posY > 0)
            cvCircle(frame, ptObject, 10, cvScalar(0, 0, 0, 0), 1, 8, 0);

        // show results
        cvShowImage("cam", frame);
        cvShowImage("threshold", imgThreshed);

        // display processing time
        t = (double) cvGetTickCount() - t;
        t = t/((double) cvGetTickFrequency()*1000.);
        printf("\tpos([%d,%d]), HSV(%.1f,%.1f,%.1f), t(%.1f[mSec])         \r", posX, posY, pixelData.H, pixelData.S, pixelData.V, t);

    } while ( currentTopState == STATE_CALIBRATE );

    free(moments);
    cvReleaseImage(&imgHSV);
    cvReleaseImage(&imgThreshed);
    
    printf("\n");
}

/* =========================================
   seekWander()

   wander and look for object
========================================= */
void seekWander(void)
{
	int   nCmd      = 0;
	WORD  wPayload  = 0;
	DWORD dwPayload = 0;

    SMTERC_putMsg(nControlTask, MV_FORWARD_CMD, W_DONT_CARE, DW_DONT_CARE);

	nCmd = SMTERC_waitMsg(__ANY__, MOTOR_START_TOV, &wPayload, &dwPayload);
	switch ( nCmd )
	{
		case Q_EMPTY:
			currentSeekSubState = STATE_SEEK_TRACK;
			break;

		case SN_ERROR:
			printf("\t\tencoder/motor error. aborting.");
			currentTopState = STATE_HALT;
			break;

		default:;
	}
}

/* =========================================
   seekTrack()

   object 'lock'ed track/drive to object
========================================= */
void seekTrack(void)
{
    int        nKey;
    int        nDelay;
    double     t;
    
    CvMoments* moments;
    double     moment10;
    double     moment01;
    double     area;

    CvPoint    ptObject;
    CvPoint    ptLineTop;
    CvPoint    ptLineBottom;

    int        posX = 0;
    int        posY = 0;
    int        nCenterError = 0;
    int        nSteeringK = 0;

    IplImage*  imgHSV = 0;
    IplImage*  imgThreshed;

    // Loop to read and handle input events
    do
    {
        t = (double) cvGetTickCount();

        // get a frame from the camera pointed to by 'capture'
        frame = cvQueryFrame(capture);
        if ( !frame )
            ErrorExit("cvQueryFrame() failed.");

        // one-time initialization on first frame
        if ( !imgHSV )
        {
            imgHSV = cvCreateImage(cvGetSize(frame), 8, 3);
            imgThreshed = cvCreateImage(cvGetSize(frame), 8, 1);
            moments = (CvMoments*) malloc(sizeof(CvMoments));

            ptLineTop.x = frame->width / 2;
            ptLineTop.y = 0;
            ptLineBottom.x = ptLineTop.x;
            ptLineBottom.y = frame->height;
        }

        // convert to HSV and calculate threshold
        cvCvtColor(frame, imgHSV, CV_BGR2HSV);

        pixelData.frame = imgHSV;

        cvInRangeS(imgHSV, cvScalar(pixelData.H - nDelta, pixelData.S - nDelta, pixelData.V - nDelta, 0),
			               cvScalar(pixelData.H + nDelta, pixelData.S + nDelta, pixelData.V + nDelta, 0), imgThreshed);

        // Calculate the moments to estimate the position of the ball
        cvMoments(imgThreshed, moments, 1);
 
        // The actual moment values
        moment10 = cvGetSpatialMoment(moments, 1, 0);
        moment01 = cvGetSpatialMoment(moments, 0, 1);
        area = cvGetCentralMoment(moments, 0, 0);

        posX = (int) moment10/area;
        posY = (int) moment01/area;

		if ( posX < 0 ) posX = 0;
		if ( posY < 0 ) posY = 0;

		// if lost track then halt the system
		if ( posX == 0 && posY == 0 )
		{
			printf("\n\t\tlost tracking\n");
			currentTopState = STATE_HALT;
			continue;
		}

        ptObject.x = posX;
        ptObject.y = posY;

        // calculate steering with PID and send to platform
		nCenterError = (ptLineTop.x - posX) / ERROR_SCALE_FACTOR;
        nSteeringK = steeringCommandPID(0, nCenterError, fCp, fCi, fCd, &fP, &fI, &fD);
		SMTERC_putMsg(nControlTask, MV_LIN_STEER, (WORD) nSteeringK, DW_DONT_CARE);

        // check if object is in grabber position
        if ( posY > (frame->height - GRABBER_LOC))
        {
            SMTERC_putMsg(nControlTask, MV_STOP_CMD, W_DONT_CARE, DW_DONT_CARE); // stop motor
            currentSeekSubState = STATE_SEEK_GRAB;
        }

        // add locator graphics to display frame and show results
        _cvPutText("TRACK");
        cvLine(frame,ptLineTop, ptLineBottom, cvScalar(0, 0, 0, 0), 1, 8, 0);
        if ( posX > 0 && posY > 0 )
            cvCircle(frame, ptObject, 10, cvScalar(0, 0, 0, 0), 1, 8, 0);
        cvShowImage("cam", frame);
        cvShowImage("threshold", imgThreshed);

        // display processing statistics
        t = (double) cvGetTickCount() - t;
        t = t/((double) cvGetTickFrequency()*1000.);
        printf("\t\tpos=(%d,%d) e=%d str=%d t=%.1f    \r", posX, posY, nCenterError, nSteeringK, t);

        if ( t >= LOOP_TIME )
            nDelay = 1;
        else
            nDelay = (int) (LOOP_TIME - t);

        nKey = cvWaitKey(nDelay);
        if ( nKey > 0 )
            currentTopState = KeyEventProc(nKey);
    } while ( currentTopState == STATE_SEEK && currentSeekSubState == STATE_SEEK_TRACK );
    
    free(moments);
    cvReleaseImage(&imgHSV);
    cvReleaseImage(&imgThreshed);

    printf("\n");
}

/* =========================================
   seekGrab()

   move to grab object
========================================= */
void seekGrab(void)
{
    currentSeekSubState = STATE_SEEK_DONE;
}

/* =========================================
   seekDone()

   complete seek state, clean up and go to home
========================================= */
void seekDone(void)
{
    currentTopState = STATE_HOME;
}

/* -----------------------------------------
   mouseClick()

   mouse click callback function
----------------------------------------- */
void mouseClick(int event, int x, int y, int flags, void* param)
{
    uchar* ptr;
	int    nAvgH = 0;
	int    nAvgS = 0;
	int    nAvgV = 0;
	int    nAveragingFactor = 0;
	int    i;
	int    j;

    if ( currentTopState != STATE_CALIBRATE )
        return;

    if ( event == CV_EVENT_LBUTTONDOWN )
    {
        ((struct pixelData_T*) param)->x = x; // get mouse click coordinates
        ((struct pixelData_T*) param)->y = y;

		// run a summing calculation for H, S and V aruond the click coordinates
		for ( i = REGION_LOW; i < REGION_HIGH; i++ )
			for ( j = REGION_LOW; j < REGION_HIGH; j++)
			{
				// avoid pixels that are out of range
				if ( (y + j) < (((struct pixelData_T*) param)->frame)->height && (y + j) > 0 &&
					 (x + i) < (((struct pixelData_T*) param)->frame)->width && (x + i) > 0 )
				{
					ptr = cvPtr2D(((struct pixelData_T*) param)->frame, y + j, x + i, NULL);
					nAvgH += ptr[0];
					nAvgS += ptr[1];
					nAvgV += ptr[2];
					nAveragingFactor++;
				}
			}

		// calculate the average
		if ( nAveragingFactor > 0 )
		{
			nAvgH /= nAveragingFactor;
			nAvgS /= nAveragingFactor;
			nAvgV /= nAveragingFactor;
		}

        ((struct pixelData_T*) param)->H = nAvgH;
        ((struct pixelData_T*) param)->S = nAvgS;
        ((struct pixelData_T*) param)->V = nAvgV;

        nCalibrated = TRUE;
    }
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
   ErrorExit()

   print error string to stderr and exit
----------------------------------------- */
void ErrorExit(LPSTR lpszMessage)
{
    fprintf(stderr, "\nErrorExit(): %s\n", lpszMessage);
    ExitProcess(1);
}

/* -----------------------------------------
   KeyEventProc()

   handle keyboard events.
   
   return:
     FALSE 'q' key not pressed,
     TRUE 'q' key pressed
----------------------------------------- */
int KeyEventProc(int nKey)
{
    int nReturn = currentTopState;
    
    switch ( nKey )
    {
    case Q_KEY:  // flag program quit
        nReturn = STATE_HALT;
        break;
    case G_KEY:  // go get object
        nReturn = STATE_SEEK;
        break;
    case P_KEY:  // pause object seek
        nReturn = STATE_PAUSE;
        break;
    case H_KEY:  // stop and go home
        nReturn = STATE_HOME;
        break;
    case C_KEY:  // calibrate to object
        nReturn = STATE_CALIBRATE;
        break;
    default:
        printf("\n\tKeyEventProc(): unhandled key code %d\n", nKey);
    }
    
    return nReturn;
}