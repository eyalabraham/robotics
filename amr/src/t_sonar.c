/* ***************************************************************************

  T_SONAR.C

  SONAR control task header file.

  April 18 2012 - Created

*************************************************************************** */

#include  "smte.h"
#include  "names.h"
#include  "messages.h"

#include  "t_sonar.h"
#include  "sonar.h"

/* -----------------------------------------
   definitions
----------------------------------------- */

#define   SAMPLES_PER_INTERVAL  3    // samples per interval to determine positive detection
#define   MIN_DETECTION_INT     165  // minimum detection interval in mSec

#define   DISTANCE_SAMPLES      3    // average readout using 3 samples, ~50 mSec apart
#define   SONAR_SAMPLE_INTERVAL 60   // measurment produced after DISTANCE_SAMPLES x SONAR_SAMPLE_INTERVAL [mSec]

/* -----------------------------------------
   external variables
----------------------------------------- */

/* -----------------------------------------
   task function code
   
   activate a report obstacle or distance
   from object using SONAR module

----------------------------------------- */

void
t_sonar(void)
{
 int    nMsg;
 WORD   wPayload;
 DWORD  dwPayload;

 int    i;
 
 int    nControlTask = 0;

 int    nDetectorOn          = FALSE;
 WORD   wDetectionSampleTime = 0;

 int    nDetectorTrigger   = 255;
 int    nDetectionValue    = 0;

 int    nSampleN   = 0,
        nSampleN_1 = 0,
        nSampleN_2 = 0;
 int    nEnable    = TRUE;

 // attach to control task
 nControlTask = getTidByName(TASK_NAME_CONTROL);
 if ( nControlTask == 0 )
    {
     putDebugMsg(DB_TRACE, DB_BAD_TNAME, __LINE__); // @@ bad task name for TASK_NAME_CONTROL
     return;
    }

 SonarOff();
 
 while (1)
    {
     //print("sonar\r\n");

     // wait for message
     nMsg = waitMsg(__ANY__, wDetectionSampleTime, &wPayload, &dwPayload);

     // parse message
     switch ( nMsg )
        {
         case Q_EMPTY:
              // use SONAR as obstacle detector to get a measurment and send out
              if ( nDetectorOn )  // this may be redundant but good as a guard
              {
                 nDetectionValue = SonarGetRange(1);

                 nSampleN_2 = nSampleN_1;
                 nSampleN_1 = nSampleN;
                 if ( nDetectionValue < nDetectorTrigger )
                    nSampleN = 1;
                 else
                    nSampleN = 0;

                 if (  nEnable && ((nSampleN + nSampleN_1 + nSampleN_2) == SAMPLES_PER_INTERVAL) )
                 {
                    putMsg(nControlTask, SONAR_OBSTALE, SONAR_DETECTED, DW_DONT_CARE);
                    nSampleN = 0;     // force 3 consecutive detection before another notification
                    nEnable  = FALSE; // only one notification will be sent. to reset, turn detector OFF then ON
                 }
              }
              break;

         case SONAR_ON:
              // turns SONAR on for operation as obstacle detector
              if ( !nDetectorOn )
              {
                 SonarOn();
                 nDetectorOn = TRUE;
                 nDetectorTrigger = (int) wPayload;               // trigger level
                 wDetectionSampleTime = (WORD) dwPayload;         // setup detection interval
                 if ( wDetectionSampleTime < MIN_DETECTION_INT )
                    wDetectionSampleTime = MIN_DETECTION_INT;
                 wDetectionSampleTime /= SAMPLES_PER_INTERVAL;
              }
              break;

         case SONAR_OFF:
              // turn off the SONAR
              SonarOff();
              nDetectorOn          = FALSE;
              wDetectionSampleTime = 0; // wake up task again only if there is a message
              nDetectionValue      = 0;
              nSampleN             = 0;
              nSampleN_1           = 0;
              nSampleN_2           = 0;
              nEnable              = TRUE;
              break;

         case SONAR_READ:
              // collect and report sensor data
              nDetectionValue = 0;
              nSampleN   = 0;      // use for temp sample
              nSampleN_1 = 0;      // use for max. sample
              nSampleN_2 = 255;    // use for min. sample

              SonarOn();
              for ( i = 0; i < DISTANCE_SAMPLES; i++)
              {
                 nSampleN = SonarGetRange(1);
                 if ( nSampleN > nSampleN_1 )
                    nSampleN_1 = nSampleN;
                 if ( nSampleN < nSampleN_2 )
                    nSampleN_2 = nSampleN;
                 nDetectionValue += nSampleN;
                 suspend(SONAR_SAMPLE_INTERVAL);
              }

              if ( !nDetectorOn ) // check if we were in SONAR_ON obstacle detection mode
              {                   // if yes, leave sonar on
                 SonarOff();      // if not, turn it off
              }

              nDetectionValue /= DISTANCE_SAMPLES;

              PACK_DW(dwPayload, nSampleN_1, nSampleN_2);
              putMsg((int) wPayload, SONAR_DISTANCE, (WORD) nDetectionValue, dwPayload);

              nDetectionValue = 0;
              nSampleN   = 0;
              nSampleN_1 = 0;
              nSampleN_2 = 0;
              break;

         default:
              // signal bad message
              putDebugMsg(DB_TRACE, DB_BAD_MSG, __LINE__); // @@ bad mode message in t_sonar()
        } // switch on bMsg
    } // endless loop
}


