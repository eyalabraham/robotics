/* ***************************************************************************

  sonar.h

  SONAR detector header file.

  April 18 2012 - Created

*************************************************************************** */

#ifndef __SONAR_H__
#define __SONAR_H__

/* -----------------------------------------
   function prototypes
----------------------------------------- */

/* ---------------------------------------------------------
   SonarOn()

   turn sonar on
--------------------------------------------------------- */
void SonarOn(void);

/* ---------------------------------------------------------
   SonarOff()

   turn sonar off
--------------------------------------------------------- */
void SonarOff(void);

/* ---------------------------------------------------------
   SonarGetRange()

   read sonar and leave on or off after reading

   return:   '0'    - on failure
           non-zero - distance on success
--------------------------------------------------------- */
int SonarGetRange(int);

#endif  /* __SONAR_H__ */
