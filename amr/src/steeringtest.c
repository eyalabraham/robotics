#include <stdlib.h>
#include <stdio.h>

#define MAX_SPREAD  8


int   nDefSpeedIndx = 2;

/* -----------------------------------------
   calculateMotorBias()

   get steering command and return left and right
   morots' speed bias.
   the function will maximize steering dynamic range
----------------------------------------- */
void
calculateMotorBias(int  nSteeringCmd,
                   int* npRightBias,
                   int* npLeftBias)
{
 int  nTemp;
 int  nOverUnderSpeed;

 // when watching the platform move away and traveling forward:
 //   a positive spread (nUk > 0) means platform will be commanded CW, veer right
 //   a negative spread (nUk < 0) means platform will be commanded CCW, veer left

 // the following code will maximize steering dynamic range

 *npLeftBias = abs(nSteeringCmd) / 2;

 if ( nDefSpeedIndx >= (MAX_SPREAD / 2))
    {
     nOverUnderSpeed = MAX_SPREAD - (nDefSpeedIndx + *npLeftBias);
     if ( nOverUnderSpeed < 0 )
        *npLeftBias += nOverUnderSpeed;
     *npRightBias = *npLeftBias - abs(nSteeringCmd);

     // don't stop motors
     if ( (nDefSpeedIndx + *npRightBias) == 0 )
        *npRightBias += 1;

     if ( nSteeringCmd < 0 )
        {
         nTemp = *npRightBias;
         *npRightBias = *npLeftBias;
         *npLeftBias = nTemp;
        }
    }
 else
    {
     nOverUnderSpeed = (nDefSpeedIndx - *npLeftBias) - 1;
     *npLeftBias = -(*npLeftBias);
     if ( nOverUnderSpeed < 0 )
        *npLeftBias = *npLeftBias - nOverUnderSpeed;
     *npRightBias = *npLeftBias + abs(nSteeringCmd);

     // don't drive beyond MAX_SPREAD
     if ( (nDefSpeedIndx + *npRightBias) > MAX_SPREAD )
        *npRightBias -= 1;

     if ( nSteeringCmd >= 0 )
        {
         nTemp = *npRightBias;
         *npRightBias = *npLeftBias;
         *npLeftBias = nTemp;
        }
    }
}

void
main(void)
{
 // use steering command and turn directly into motor speed spread
 // this is a very simple and easy way to do it...

 int   nDeltaSpreadR;
 int   nDeltaSpreadL;

 int   nSteering;

 // when watching the platform move away and traveling forward:
 //   a positive spread (nUk > 0) means platform will be commanded CW, veer right
 //   a negative spread (nUk < 0) means platform will be commanded CCW, veer left

 // limit steering command to max. spread
 // if ( abs(nSteering) > MAX_SPREAD )
 //   nSteering = SIGN(nSteering) * MAX_SPREAD;

 printf("Steer\tSpeed\tL\tR\n");

 for(nSteering = -MAX_SPREAD; nSteering <= MAX_SPREAD; nSteering++)
    {
     calculateMotorBias(nSteering, &nDeltaSpreadR, &nDeltaSpreadL);
     printf("%d\t%d\t%d\t%d\n", nSteering, nDefSpeedIndx, nDeltaSpreadL, nDeltaSpreadR);
    }
}
