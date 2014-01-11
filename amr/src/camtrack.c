/* ***************************************************************************

  CAMTRACK.C

  line tracking using webcam.
  the console app will also have keybord control of AMR robot; control uses arrow keys:
  'g'   - start line tracking
  's'   - all stop
  'q'   - quit app.
  'd'   - intiate log dump on AMR

  July 20 2010 - Created

*************************************************************************** */

#define  _CRT_SECURE_NO_WARNINGS // suppress compiler errors for unsafe scsnf()

#include <windows.h>
#include <stdio.h>
#include <math.h>

#include "cv.h"
#include "highgui.h"

/* -----------------------------------------
   definitions
----------------------------------------- */

#define  TXT_BUF            80

#define  BUFFER             128       // console input buffer

#define  PORT               "\\\\.\\COM21"

#define  LOOP_TIME          100       // overall loop time in mili-sec (image processing segment is ~30mSec)

#define  LEFT_KEY           2424832   // keyboard scan codes
#define  UP_KEY             2490368
#define  RIGHT_KEY          2555904
#define  DOWN_KEY           2621440
#define  Q_KEY              81
#define  D_KEY              68
#define  S_KEY              83
#define  G_KEY              71

#define  PIC_SIZE           320       // pic size in pixels

#define  ROI_HEIGHT         5         // pixels
#define  ROI_BANDS          10        // max possible ROI bands
#define  DEFAULT_BAND       5
#define  ROI_BOXES          40        // boxes dividing the ROI across pic width
#define  COMP_FACTOR        1
#define  MIN_BOX_PARAM      15

#define  LINE_THRESHOLD     116       // blue line threshold level <-- adjust when calibrating

#define  _Cp                0.2       // PID constants
#define  _Ci                0.01
#define  _Cd                0.2
#define  DEAD_BAND          0.0
#define  INTEG_LIMIT        0.6       // 60% of full scale
#define  STEER_LIMIT        8         // +/- STEER_LIMIT limit on PID output, max. of 'MAX_SPREAD' in t_ctrlex.c
#define  CENTER_SPEED       N_SPEED_2 // motor center speed

#define  MODE_QUIT         -1
#define  MODE_NONE          0
#define  MODE_LINE_TRACK    1
#define  MODE_SEEK_LINE     2

#define  MIN(A,B)           (((float) A < (float) B) ? (float) A : (float) B)

/* -----------------------------------------
   types
----------------------------------------- */


/* -----------------------------------------
   funciton prototypes
----------------------------------------- */

void  ErrorExit(LPSTR);
int   KeyEventProc(KEY_EVENT_RECORD*, int);
int   trackingError(CvMat*, int*, int, int, char*);
int   calcWeight(CvMat*, int, int);
int   steeringCommandPID(int, int, float, float, float, float*, float*, float*);
int   getBandY(CvMat*, int);
int   getBatteryCap(void);
void  drawTrackerSegment(CvMat*, int, int);

/* -----------------------------------------
   globals
----------------------------------------- */

BOOL         fQuit = FALSE;

/* -----------------------------------------
   main()
----------------------------------------- */
int
main(VOID)
{
 HANDLE     hStdin;
 DWORD      cNumRead,
            dwSaveOldMode;

 int        i;

 INPUT_RECORD irInBuf[BUFFER];

 IplImage*  frame;
 CvCapture* capture;
 CvMat*     gray           = 0;
 CvMat*     blue           = 0;

 CvPoint    ptLineTop;
 CvPoint    ptLineBot;
 CvPoint    ptBandLeft;
 CvPoint    ptBandRight;

 DWORD      t              = 0;
 DWORD      dwDelay        = 0;

 int        nROIBand       = DEFAULT_BAND;

 int        nProcessError  = 0;
 int        nSteeringK     = 0;
 float      fCp            = _Cp;
 float      fCi            = _Ci;
 float      fCd            = _Cd;
 float      fP             = 0.0;
 float      fI             = 0.0;
 float      fD             = 0.0;

 int        nThreshold     = LINE_THRESHOLD;

 int        nMode          = MODE_NONE;

 int        nROIBoxes[ROI_BOXES];

 char       textTracker[ROI_BOXES + 1] = {0};    // text visualization of tracker e.g. [....****....]
 char       textBuffer[TXT_BUF]        = {0};    // text buffer for traces
 int        nCharsToPrint;
 
 printf("Built: %s, %s\n", __DATE__, __TIME__);

 // get PID/Fuzzy constants
 //printf("enter PID constants <Cp Ci Cd Threshold>: ");
 //scanf("%g %g %g %d", &fCp, &fCi, &fCd, &nThreshold);

 // initialize the camera interface for capturing
 capture = cvCaptureFromCAM(0);
 if ( !capture )
    ErrorExit("cvCaptureFromCAM() failed.");

 cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, PIC_SIZE);

 cvNamedWindow("capture", CV_WINDOW_AUTOSIZE);
 cvNamedWindow("tracking", CV_WINDOW_AUTOSIZE);

 cvCreateTrackbar("threshold", "tracking", &nThreshold, 255, NULL);
 cvSetTrackbarPos("threshold", "tracking", nThreshold);

 // Get the standard input handle.
 hStdin = GetStdHandle(STD_INPUT_HANDLE);
 if ( hStdin == INVALID_HANDLE_VALUE )
    ErrorExit("GetStdHandle() failed.");

 // Save the current input mode, to be restored on exit.
 if ( !GetConsoleMode(hStdin, &dwSaveOldMode) )
    ErrorExit("GetConsoleMode() failed.");

 printf("\n");
 printf("line track               \n");
 printf("---------------          \n");
 printf(" 'G'   - go line tracking\n");
 printf(" 'S'   - all stop        \n");
 printf(" 'Q'   - quit            \n");
 printf(" 'D'   - intiate log dump\n");
 printf("\n");

 // Loop to process cam input and PID
 do
    {
     // webcam image capture and process
     t = GetTickCount();

     if ( nMode == MODE_NONE )
     {
		 // reset process variables and PID integrator
         nSteeringK = steeringCommandPID(1, 0, 0, 0, 0, 0, 0, 0); 
     }

     // get a frame from the camera pointed to by 'capture'
     frame = cvQueryFrame(capture);
     if ( !frame )
        ErrorExit("cvQueryFrame() failed.");
     
     // one-time initialization on first frame
     if ( !gray )
     {
         gray = cvCreateMat(frame->height, frame->width, CV_8UC1);
		 blue = cvCreateMat(frame->height, frame->width, CV_8UC1);
		 printf("image size: %d x %d\nthreshold: %d\n\n", gray->cols, gray->rows, nThreshold);

		 ptLineTop.x = gray->cols / 2;
         ptLineTop.y = 0;
         ptLineBot.x = ptLineTop.x;
         ptLineBot.y = gray->rows;
     }

	 // extract blue channel - webcam must be set to color capture, and line tape must be blue
	 // then filter with a fixed level threashold
	 // threshold level of 150 was experamentally determined (camtrack.c)
	 // and shold be adjusted based on lighting conditions
	 cvSplit(frame, NULL, NULL, blue, NULL);
	 cvThreshold(blue, gray, nThreshold, 255, CV_THRESH_BINARY);

     // calculate image weight
     for (i = 0; i < ROI_BOXES; i++)
     {
		 nROIBoxes[i] = calcWeight(gray, i, nROIBand);
	 }
	 
     // calculate process error (in +/- number of boxes from center line)
	 nProcessError = trackingError(gray, nROIBoxes, 0, nROIBand, textTracker);

	 switch ( nMode )
	 {
	 case MODE_LINE_TRACK:
		 // calculate steering with PID and send to platform
		 nSteeringK = steeringCommandPID(0, nProcessError, fCp, fCi, fCd, &fP, &fI, &fD);

		 // print trace to csv file
		 nCharsToPrint = _snprintf(textBuffer, TXT_BUF, "%lu,%d,%d,%g,%g,%g,[%s]\n", GetTickCount(), nProcessError, nSteeringK, fP, fI, fD, textTracker);
		 break;

	 case MODE_SEEK_LINE:
		 break;

	 default:;
	 }

     // print some processing stats
     printf("mode: %2d [%s] err: %d st: %d      \r", nMode,
		                                             textTracker,
											         nProcessError,
													 nSteeringK);

	 // draw center line and scan level line
	 cvLine(gray, ptLineTop, ptLineBot, CV_RGB(0, 0, 128), 1, 8, 0);

	 ptBandLeft.x = 0;
	 ptBandLeft.y = getBandY(gray, nROIBand);
	 ptBandRight.x = gray->cols;
	 ptBandRight.y = ptBandLeft.y;

	 cvLine(gray, ptBandLeft, ptBandRight, CV_RGB(0, 0, 128), 1, 8, 0);

	 cvShowImage("capture", frame);
	 cvShowImage("tracking", gray);

	 cvWaitKey(1);

	 // Wait for keyboard events
     t = GetTickCount() - t;
	 if ( t >= LOOP_TIME )
		 dwDelay = 1;
	 else
		 dwDelay = (DWORD) (LOOP_TIME - t);

	 switch ( WaitForSingleObject(hStdin, dwDelay) )
	 {
	     case WAIT_OBJECT_0:
		      ReadConsoleInput(hStdin, irInBuf, BUFFER, &cNumRead);
  		  	  for ( i = 0; i < (int) cNumRead; i++)
				  {
					  if ( irInBuf[i].EventType == KEY_EVENT )
					  {
						  nMode = KeyEventProc(&irInBuf[i].Event.KeyEvent, nMode);
					  }
			      }
		      break;

	     default:;
	 }

    }
 while ( nMode != MODE_QUIT );

 printf("\nexiting.\n");
 
 // close all open handles
 cvReleaseCapture(&capture);
 cvDestroyAllWindows();

 return 0;
}

/* -----------------------------------------
   ErrorExit()

   print error string to stderr and exit
----------------------------------------- */
void
ErrorExit(LPSTR lpszMessage)
{
 fprintf(stderr, "%s\n", lpszMessage);
 ExitProcess(1);
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

	 // calculate PID control output
     fUk = (fCp * fErrk) +
		   (fCi * fErrSum) +
		   (fCd * (fErrk - fErrk_1));

	 //-------------- for tracing -----------------------

	 *pfP = fCp * fErrk;
	 *pfI = fCi * fErrSum;
	 *pfD = fCd * (fErrk - fErrk_1);

	 //--------------------------------------------------

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
   drawTrackerSegment()

   draw tracker lines under the
   designated box.
----------------------------------------- */
void
drawTrackerSegment(CvMat* img,
                   int    nBoxNum,
				   int    nBand)
{
    int      nBoxWidth;
    CvPoint  ptLeft;
    CvPoint  ptRight;

    nBoxWidth = img->cols / ROI_BOXES;
    ptLeft.x = nBoxWidth * nBoxNum;
    ptLeft.y = getBandY(img, nBand);

    ptRight.x = ptLeft.x + nBoxWidth;
    ptRight.y = ptLeft.y;

    cvLine(img, ptLeft, ptRight, CV_RGB(0, 0, 255), 2, 8, 0 );
}

/* -----------------------------------------
   trackingError()

   calculate tracker error as the +/-
   difference in box count between center line
   and center of the tracked segment

   return tracker error.
----------------------------------------- */
int
trackingError(CvMat* img,
              int*   nROIBoxes,
              int    nAverageBox,
			  int    nBand,
              char*  textTracker)
{
    int i;
    int nBoxes        = 0;
    int nSumBoxNumber = 0;

    textTracker[0] = 0;
    
    for (i = 0; i < ROI_BOXES; i++)
    {
        if ( nROIBoxes[i] <= nAverageBox )
        {
            nSumBoxNumber += i;
            nBoxes++;
            strcat(textTracker, "*");
			drawTrackerSegment(img, i, nBand);
        }
        else
        {
            strcat(textTracker, " ");
        }
    }

    if ( nBoxes == 0 )
        return 0;
    else
        return ((ROI_BOXES / 2) - (nSumBoxNumber / nBoxes));
}
/* -----------------------------------------
   calcWeight()

   calculates the region of interest's black/white weight.
   summarizes gray scale pixel value and divides
   by pixel count

   return the image weight.
----------------------------------------- */
int
calcWeight(CvMat* img,
           int    nBoxNum,
           int    nBand)
{
    int      x;
    int      y;
    int      nBoxWidth;

    int      nWeight  = 0;
    int      nSamples = 0;
    CvScalar pixel;
    CvPoint  ptTopLeft;

    nBoxWidth = img->cols / ROI_BOXES;
    ptTopLeft.x = nBoxWidth * nBoxNum;
    ptTopLeft.y = getBandY(img, nBand);

    for (x = 0; x < nBoxWidth; x++)
        for (y = 0; y < ROI_HEIGHT; y++)
        {
            // calculate 
            pixel = cvGet2D(img, ptTopLeft.y + y, ptTopLeft.x + x);
            nWeight += (int) pixel.val[0];
            nSamples++;
        }

    return (nWeight / nSamples);
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
	 switch ( pKer->wVirtualKeyCode )
	 {
		  case Q_KEY:     // flag program quit
               nModeReturn = MODE_QUIT;
               break;
          case D_KEY:     // send X_PRINT_DATA_LOG to initiate log dump
               //printf("Log dump initiated.\n");
               nModeReturn = MODE_NONE;
               break;
          case S_KEY:     // stop
               nModeReturn = MODE_NONE;
		       break;
          case G_KEY:    // start moveing
               if ( nCurrentMode != MODE_LINE_TRACK )
			   {
                   nModeReturn = MODE_LINE_TRACK;
			   }
               break;
          default:
               //printf("\nunhandled key %d\n", nKey);
               ;
	 }
 }

 return nModeReturn;
}
/* -----------------------------------------
   getBandY()

   calculate band's Y row coordinate
   and return that value
----------------------------------------- */
int
getBandY(CvMat* img,
		 int    nBand)
{
	int   y;

	y = nBand * (img->rows / ROI_BANDS);

	if ( (y + ROI_HEIGHT) > img->rows )
		y = img->rows - ROI_HEIGHT;

	return y;
}
/* -----------------------------------------
   getBatteryCap()

   return battery capacity in %
----------------------------------------- */
int
getBatteryCap(void)
{
	SYSTEM_POWER_STATUS batState;

	GetSystemPowerStatus(&batState);

	return (int) batState.BatteryLifePercent;
}