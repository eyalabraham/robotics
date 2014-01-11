/* ***************************************************************************

  AMREX.C

  Main application module.

  Oct. 23 2011 - Created
  11 20 2011   - modified from amr.c
  Jan. 2 2012  - modified to use HC08

*************************************************************************** */

#include  <stdio.h>
#include  <stdlib.h>
#include  <assert.h>
#include  <string.h>
#include  <dos.h>

#include  "fxp.h"
#include  "smte.h"
#include  "names.h"

#include  "hc08drv.h"

#include  "t_wdog.h"
#include  "t_sense.h"
#include  "t_task_stub.h"
#include  "t_ctrl.h"
#include  "t_dist.h"
#include  "t_dsply.h"
#include  "t_sonar.h"

#pragma intrinsic strcmp        /* compile strcmp() as inline               */
#pragma intrinsic strcat        /* compile strcat() as inline               */

/* -----------------------------------------
   globals
----------------------------------------- */

#define   FILE_NAME_LEN      12
#define   DEFAULT_COMMAND    "command.lst"

int     nDebugLevel          = __TRC_LVL1__; // debug level
int     nAlgorithm           = 0;            // t_ctrlex() -- open loop
int     nSenseSampleInterval = 100;          // t_sense(), t_ctrlex()
int     nDeadBand            = 0;            // t_ctrlex()
QmnFP_t qCp                  = 16;           // t_ctrlex() internal PID constants
QmnFP_t qCi                  = 0;            // t_ctrlex()
QmnFP_t qCd                  = 6;            // t_ctrlex()

volatile BYTE bUtil          = 0x00;         // t_wdog(), t_ctrlex(), t_dsply()

/* -----------------------------------------
   parseCommandFile()

   parse command file for initialization

   Return:
    1 if success or 0 of failed.
----------------------------------------- */
int
parseCommandFile(char* szFile)
{
 FILE *cmdFile;

 BYTE    bEndCmd      = 0;
 int     nRetOk       = 1;
 WORD    wLine        = 0;
 char    szStr[81]    = {0};
 char    szIdent[21]  = {0};
 int     nParam;

 /* system initialization code */

 if ((cmdFile = fopen(szFile, "rt")) == NULL)
    {
     printf("ERR: parseCommandFile(): Cannot open command file '%s'\n", szFile);
     return 0;
    }

 printf("INFO: parsing command file '%s'\n", szFile);

 while ( !feof(cmdFile) && !bEndCmd && nRetOk )
    {
     szStr[0] = '\0';
     szIdent[0] = '\0';
     fgets(szStr, 80, cmdFile);
     wLine++;
     switch ( szStr[0] )
        {
         case '!':
              sscanf(szStr, "%*1s %20s %d", szIdent, &nParam);
              printf("  %s = %d\n", szIdent, nParam);
              if ( strcmp(szIdent, "debugLevel") == 0 )
                 nDebugLevel = nParam;
              else if ( strcmp(szIdent, "sampleInterval") == 0 )
                 nSenseSampleInterval = nParam;
              else if ( strcmp(szIdent, "constProp") == 0 )
                 qCp = (QmnFP_t) nParam;
              else if ( strcmp(szIdent, "constInteg") == 0 )
                 qCi = (QmnFP_t) nParam;
              else if ( strcmp(szIdent, "constDiff") == 0 )
                 qCd = (QmnFP_t) nParam;
              else if ( strcmp(szIdent, "deadBand") == 0 )
                 nDeadBand = nParam;
              else if ( strcmp(szIdent, "maxSpread") == 0 )
                 ; // do nothing, this is for backward compatibility
              else if ( strcmp(szIdent, "algorithm") == 0 )
                 nAlgorithm = nParam;
              else
                 {
                  printf("ERR: parseCommandFile(): unknown parameter line %i\n", wLine);
                  nRetOk = 0;
                 }
              break;
         case '*':
              bEndCmd = 1;
              nRetOk  = 1;
              break;
        }
    }

 fclose(cmdFile);

 return nRetOk;
}

/* -----------------------------------------
   startup code
----------------------------------------- */

int
main(int argc, char* argv[])
{
 char szCommandFile[FILE_NAME_LEN + 1] = {DEFAULT_COMMAND};
 int  i;
 
 // parse command line parameters:
 // harness [-v] [-f <command_file>]
 //   '-v' print build and verion information
 //   '-f' provide alternate command file, default is 'command.lst'
 for ( i = 1; i < argc; i++ )
 {
     if ( strcmp(argv[i], "-v") == 0 )
     {
        printf("BUILD: amrex.exe %s %s\n", __DATE__, __TIME__);
        return 0;
     }
     else if ( strcmp(argv[i], "-f") == 0 )
     {
         if (i + 1 < argc) // check that a file name was supplied
         {
             i++;
             strncpy(szCommandFile, argv[i], FILE_NAME_LEN);
         }
     }
     else
     {
        printf("FAIL: main(), bad command line option '%s'\n", argv[i]);
        return 1;
     }
 }
 
 // build date
 printf("BUILD: amrex.exe %s %s\n", __DATE__, __TIME__);

#ifdef __V25__
 printf("INFO: V25 object module.\n");
#endif

 // system initialization from command file
 assert(parseCommandFile(szCommandFile));

 // hardware initialization

 assert(conCommInit());   // macro service receive on serial console
 printf("INFO: Console macro service ok\n");

 assert(auxCommInit());   // PS/2 device on aux. comm setup
 printf("INFO: PS2 macro service ok\n");

 assert(hc08Init());      // connection test with HC08
 printf("INFO: HC08 status: 0x%x\n", hc08Init());

 /* tasks:           task        stack ticks    msg  name                    idle state  */
 assert(registerTask(t_wdog,       64,    1,     0,  TASK_NAME_WATCHDOG));   // reschedule()  1

 assert(registerTask(t_sonar,     128,    5,     2,  TASK_NAME_SONAR));      // block()       2

 assert(registerTask(t_sense,     512,   10,     0,  TASK_NAME_SENSOR));     // reschedule()  5

 assert(registerTask(t_control,   512,   10,    10,  TASK_NAME_CONTROL));    // block()       5

 assert(registerTask(t_dist,      256,    8,     2,  TASK_NAME_DISTANCE));   // block()       2

 assert(registerTask(t_nav_stub,  512,   10,    10,  TASK_NAME_TASK_STUB));  // reschedule()  5

 assert(registerTask(t_dsply,     128,    1,     2,  TASK_NAME_DISPLAY));    // block()       1

 // ***  add 5 mili sec for smte utility task  ***

 /* start scheduler */
 startScheduler(nDebugLevel, __TIMER_OFF__, 1);

 return 0;
}
