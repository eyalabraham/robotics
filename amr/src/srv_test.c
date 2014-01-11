/* ***************************************************************************

  SERVO_TEST.C

  Servo driver test.

  March 16 2002 - Created

*************************************************************************** */

#include <stdio.h>
#include <assert.h>

#include "servo.h"
#include "hc08drv.h"

void
main(void)
{
 int  nCount;
 int  nDone    = 0;
 WORD wInitVal = 0;

 printf("build %s %s\n", __DATE__, __TIME__);

 wInitVal = hc08Init();
 printf("servoInit() 0x%x;\n", wInitVal);

 while ( !nDone )
    {
     printf("enter angle count 35<P<125 ('-1' to quit): ");
     scanf("%d", &nCount);

     if ( nCount < 0 )
        nDone = 1;
     else
        {
         if ( !servoSetCount((WORD) nCount) )
            printf("error or count out of range!\n");
        }
    }
}
