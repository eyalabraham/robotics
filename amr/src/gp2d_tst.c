/* ***************************************************************************

  GP2D_TST.C

  GP2D02 driver test with simple DOS executable.

  March 27 2002 - Created

*************************************************************************** */

#define  __FLOAT__

#include <stdio.h>
#include <dos.h>
#include <stdlib.h>
#include <conio.h>
#include <assert.h>

#include "sonar.h"
#include "gp2d02.h"
#include "servo.h"
#include "hc08drv.h"
#include "fxp.h"


#define  MENU "\n[M]annually initiate distance measurment\n[S]ervo scan and distance measurment\nS[O]NAR collision detector\n[c]alibration data for SONAR\n[Q]uit\n\n?"

#define  CALIBRATION_SAMPLES 10 // IR calibration samples
void
main(void)
{
 int   nStart;
 int   nEnd;
 int   nStep;
 int   nMeasurements;
 int   nPosition;
 int   i;
 int   nData;
 int   nAvg;
 int   nMax;
 int   nMin;
 WORD  wInitVal = 0;

 printf("build %s %s\n", __DATE__, __TIME__);

 wInitVal = hc08Init();
 printf("hc08Init() 0x%x\n", wInitVal);

 //printf("[M]annually initiate distance measurment\n");
 //printf("[S]ervo scan and distance measurment\n");
 //printf("[I]R collision detector\n");
 //printf("[Q]uit\n");
 //printf("\n?");

 while ( 1 )
 {
     printf(MENU);
     switch ( getche() )
     {
        case 'm':
        case 'M':
             // mannually initiate a distance measurment
             printf("\nhit ENTER to trigger measurment, 'q' to escape test\n");
             i = 0;
             while ( i != 'q' )
             {
                i = getche();
                if ( gp2d02Trigger() )
                {
                   nData = gp2d02GetDistance();
                   printf("readout=%d,  %g [cm]      \r", nData, Qmn2FLOAT(gp2d02DataToDistance(nData)));
                }
                else
                {
                   printf("trigger failed    \r");
                }
             }
             printf("\n");
             break;

        case 's':
        case 'S':
             // generate servo scan and GO2d distance measurement
             printf("\nservo start position (35<P<125): ");
             scanf("%d", &nStart);
             printf("servo end position (P>%d): ", nStart);
             scanf("%d", &nEnd);
             printf("servo position step: ");
             scanf("%d", &nStep);
             printf("measurement average: ");
             scanf("%d", &nMeasurements);

             // print table:
             // <position>;<data-1>;<dist-1>;....<data-N>;<dist-N>;<average-data>;<average-dist>

             for ( nPosition = nStart; nPosition < nEnd; nPosition += nStep)
             {
                 nData = 0;
                 nAvg  = 0;

                 servoSetCount(nPosition);
                 printf("%d,", nPosition);

                 for ( i = 0; i < nMeasurements; i++)
                 {
                     if ( gp2d02Trigger() )
                     {
                        nData = gp2d02GetDistance();
                        printf("%d,", nData);
                        printf("%d,", gp2d02DataToDistance(nData));
                        nAvg  += nData;
                     }
                     else
                     {
                        printf("\ntrigger failed");
                     }
                 }

                 nAvg /= nMeasurements;
                 printf("%d,", nAvg);
                 printf("%d\n", gp2d02DataToDistance(nAvg));
             }
             servoSetCount(MID_POS);
             break;

        case 'o':
        case 'O':
             // collision detector
             SonarOn();
             printf("\nSONAR on\n");
             for ( i = 0; i < 60; i++ )
             {
                printf("(%d) detector value %d     \r", (60 - i), SonarGetRange(TRUE));
                sleep(1);
             }
             SonarOff();
             printf("\nSONAR off\n");
             break;

        case 'c':
        case 'C':
              nAvg  = 0;
              nData = 0;      // use for temp sample
              nMax  = 0;      // use for max. sample
              nMin  = 255;    // use for min. sample
              printf("\n");

              SonarOn();
              for ( i = 0; i < CALIBRATION_SAMPLES; i++)
              {
                 nData = SonarGetRange(TRUE);
                 if ( nData > nMax )
                    nMax = nData;
                 if ( nData < nMin )
                    nMin = nData;
                 nAvg += nData;
                 printf("sample %d  \r", i);
                 sleep(1);
              }
              SonarOff();
              nAvg /= CALIBRATION_SAMPLES;

              printf("calibration data: avg=%d  [%d,%d]\n", nAvg, nMax, nMin);

              nAvg  = 0;
              nData = 0;
              nMax  = 0;
              nMin  = 0;
              break;

        case 'q':
        case 'Q':
             exit(EXIT_SUCCESS);
             break;
     }
 }
}
