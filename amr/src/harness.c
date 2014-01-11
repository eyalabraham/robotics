/* ***************************************************************************

  harness.C

  Feb   2007

*************************************************************************** */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include  "smte.h"
#include  "t_ctrl.h"
#include  "t_task_stub.h"
#include  "fxp.h"

#define   STDIN_STRING     80
#define   FILE_NAME_LEN    11
#define   DEFAULT_COMMAND  "command.lst"

int       nNewMessage = 0;
int       nQPayload;                   /* message type identifier          */
WORD      wQPayload;                   /* word payload                     */
DWORD     dwQPayload;                  /* double word payload              */

char      sInput[STDIN_STRING + 1] ={0};

int                     nStartupDelay;

int  nDebugLevel          = __TRC_LVL0__;   /* general                        */
int  nAlgorithm           = 0;              /* t_control()                    */
int  nAccelerationDelay   = 50;             /* t_motor()                      */
int  nSenseSampleInterval = 250;            /* t_sense()                      */

BYTE    bUtil;                    // defined in amrex.c
int     nSenseSampleInterval;
int     nAlgorithm;
int     nDeadBand;
QmnFP_t qCp;
QmnFP_t qCi;
QmnFP_t qCd;

/* -----------------------------------------
   harness function prototypes
----------------------------------------- */


/* -----------------------------------------
*/

void
print(char* szStr)                       /* print string to 'stdout'         */
{
 printf("\tprint() [%s]\n", szStr);
}

static BYTE
nibtohex(BYTE  bNibble)
{
 return ( (bNibble > 9) ? (bNibble - 0x0a + 'a') : (bNibble + '0'));
}

char*                                    /* convert number to hex string     */
tohex(DWORD   dwNum,                     /* number to convert                */
      char*   szHex,                     /* pointer to string buffer         */
      int     nSize)                     /* string buffer size               */
{
 int   i;

 if ( nSize > 8 )
    return 0;

 szHex[nSize] = '\0';

 for ( i = nSize ; i > 0; i--)
    {
     szHex[i - 1] = nibtohex((BYTE) dwNum & 0x0f);
     dwNum >>= 4;
    }

 return szHex;
}

/* -----------------------------------------
*/
void
setCSflag(void)
{
 printf("\t >>> Enter Critical Section.\n");
}

/* -----------------------------------------
*/
void
clearCSflag(void)
{
 printf("\t <<< Exit Critical Section.\n");
}

/* -----------------------------------------
*/
#pragma  argsused
int
registerTask(void   (* func)(void),      /* pointer to task                  */
             WORD   wStackSize,          /* task stack size in WORD count
                                            '0' = default (512 words)        */
             WORD   wTicks,              /* task tick count
                                            '0' = default (5)                */
             int    nMsgQsize,           /* message Q size
                                            '0' = default (1 message)        */
             char*  szTaskName)          /* task name string                 */
{
 printf("\tregisterTask(%s)\n", szTaskName);
 return 1;
}

/* -----------------------------------------
*/
#pragma  argsused
void
startScheduler(enum tag_traceLevel traceLvl,  /* trace sevices level        */
               WORD           wTimerFlag, /* '1' to register timer sevices   */
               WORD           wPerTick)   /* mili-sec per timer tick
                                            '0' = DEF_MSEC_PER_TICK (5)      */
{
 printf("\tstartScheduler()\n");
}

/* -----------------------------------------
*/
int
getTidByName(char* szTaskName)           /* pointer to task name             */
{
 int  nId;

 printf("\tgetTidByName(%s)\n", szTaskName);
 fgets(sInput, STDIN_STRING, stdin);
 sscanf(sInput, "%i", &nId);
 printf("\t\ttask ID=%d:\n", nId);

 return nId;
}

/* -----------------------------------------
*/
#pragma  argsused
char*
getNameByTid(int     nTid,               /* task ID to search for            */
             char*   szTaskName,         /* task name return buffer          */
             int     nNameLen)           /* task name buffer size            */
{
 printf("\tgetNameByTid(%i)\n", nTid);
 return NULL;
}

/* -----------------------------------------
*/
#pragma  argsused
void
suspend(WORD  wTime)                     /* suspened time in mili-seconds    */
{
 printf("\tsuspend(%i)\n", wTime);
}

/* -----------------------------------------
*/
int
release(int nTid)                       /* task ID to release               */
{
 printf("\trelease(%i)\n", nTid);
 return nTid;
}

/* -----------------------------------------
*/
#pragma  argsused
int
putMsg(int   nTid,                       /* destination task                 */
       int   nPayload,                   /* message type identifier          */
       WORD  wPayload,                   /* word payload                     */
       DWORD dwPayload)                  /* double word payload              */
{
 printf("\tputMsg(%i : %u %u %lu)\n", nTid, nPayload, wPayload, dwPayload);

 nNewMessage = nTid;              /* flag message in Q                */
 nQPayload   = nPayload;          /* message type identifier          */
 wQPayload   = wPayload;          /* word payload                     */
 dwQPayload  = dwPayload;         /* double word payload              */

 return 1;
}

/* -----------------------------------------
*/
#pragma  argsused
int                                      /* returns Q_EMPTY if no messages,
                                            or bPayload if got message       */
getMsg(int*    pnPayload,                /* message type                     */
       WORD*   pwPayload,                /* word payload                     */
       DWORD*  pdwPayload)               /* double word payload              */
{
 printf("\tgetMsg()\n");

 if (nNewMessage == 0)
    return Q_EMPTY;
 else
    {
     pnPayload   = &nQPayload;      /* get message information */
     pwPayload   = &wQPayload;
     pdwPayload  = &dwQPayload;

     nNewMessage = 0;               /* clear meaasge flag      */
     return nQPayload;
    }
}

/* -----------------------------------------
*/
#pragma  argsused
int                                      /* returns Q_EMPTY if timed out,
                                            or bPayload if got message       */
waitMsg(int     nMsg,                    /* message to wait for or '__ANY__' */
        WORD    wTimeOut,                /* wait time out value, 0 = forever */
        WORD*   pwPayload,               /* word payload                     */
        DWORD*  pdwPayload)              /* double word payload              */

{
 nQPayload  = 0;
 wQPayload  = 0;
 dwQPayload = 0L;

 printf("\twaitMsg()\n");
 fgets(sInput, STDIN_STRING, stdin);
 sscanf(sInput, "%i %u %lx", &nQPayload, &wQPayload, &dwQPayload);
 printf("\t\tmessage (bPayload=%d wPayload=%d dwPayload=0x%lx):\n", nQPayload, wQPayload, dwQPayload);

 *pwPayload   = wQPayload;      /* get message information */
 *pdwPayload  = dwQPayload;

 nNewMessage = 0;               /* clear meaasge flag      */

 if ( nQPayload == -1 )         // exit the harness
 {
     printf("\tEXIT\n");
     exit(1);
 }

 return nQPayload;
}

/* -----------------------------------------
*/
void                                     /* flush all pending messages       */
flushMsgQ(void)                          /* of calling task                  */
{
 printf("\tflushMsgQ()\n");
 nNewMessage = 0;               /* clear meaasge flag      */
}

/* -----------------------------------------
*/
#pragma  argsused
int
putDebugMsg(int    nPayload,             /* message type identifier          */
            WORD   wPayload,             /* word payload                     */
            DWORD  dwPayload)            /* double word payload              */
{
 printf("\tputDebugMsg(%hu %u %lu)\n", nPayload, wPayload, dwPayload);
 return 0;
}

/* -----------------------------------------
*/
int                                      /* trace data                       */
putDataLog(WORD wTraceData)              /* byte/word                        */
{
 printf("\tputDataLog(%u)\n", wTraceData);
 return 0;
}

/* -----------------------------------------
*/
void                                     /* dump trace data                  */
dumpDataLog(const int nGroupDelimiter)   /* data count row delimiter         */
{
 printf("\tdumpDataLog(%i)\n", nGroupDelimiter);
}

/* -----------------------------------------
*/
DWORD
getGlobalTicks(void)                     /* get global ticks                 */
{
 printf("\tgetGlobalTicks()\n");
 return 0xffffffffL;
}

/* -----------------------------------------
*/
WORD
getMsecPerTick(void)                     /* get tick interval                */
{
 printf("\tgetMsecPerTick()\n");
 return 1;
}

/* -----------------------------------------
*/
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
*/
int
main(int argc, char* argv[])
{
 char szCommandFile[FILE_NAME_LEN + 1] = {DEFAULT_COMMAND};
 int  i;
 
 // parse command line parameters:
 // harness [-v] [-f <command_file>]
 //   '-v' print build and verion information
 //   '-f' provide alternate command file, default is 'command.lst'

 // something quick and dirty to parse command line parameters...
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
 
 if ( parseCommandFile(szCommandFile) )
    t_control();
    
 return 0;
}
