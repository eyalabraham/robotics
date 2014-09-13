/* ***************************************************************************

  T_CTRLEX.C

  Control task source code.

  Jan.  6 2001 - created
  Jan. 14 2007 - added PID
  June  8 2010 - added new continuous move commands
                 changes fixed point conversion
  Nov.    2011 - moved in motor control, eliminated comm with motor tasks
  May     2012 - bug fixes

*************************************************************************** */

#include  <values.h>
#include  <conio.h>

#ifdef __HARNESS__
 #include <stdio.h>
#endif

#include  "smte.h"
#include  "messages.h"
#include  "names.h"
#include  "fxp.h"
#include  "platform.h"
#include  "ctrlio.h"

#include  "t_ctrl.h"

/* -----------------------------------------
   global definitions
----------------------------------------- */

#define  SENSOR_TOV      3       // N x nSenseSampleInterval for mSec time-out of wheel sensor data

#define  L_MOTOR         0
#define  R_MOTOR         1

#define  DIR_UNDEF       0
#define  DIR_FWD         1
#define  DIR_REV        -1

#define  INTEG_LIMIT     0x0014  // PID integrator limit 62.5% of MAX_SPREAD in Qmn
                                 // S000|0000|000.1|0100 -> 0x0014 -> 0.625

#define  MAX_SPD_INDX    MV_SPEED_MAX
#define  MAX_SPREAD      MV_SPEED_MAX
#define  DEF_SPD_INDX    MV_SPEED_4       // default index of center level of motor speed

#define  TRAVEL_DIST_REPORT_THRESH 0x0392 // max value of click counter to prevent Qmn overflow when converting to [cm]
                                          // this value will report traveled distance every ~400[cm]
#define  TRAVEL_DIST_REPORT_INTERVAL 1000 // mSec between travel distance report

#define  RT_MOTOR_CTRL_AND_MASK 0xf0
#define  LT_MOTOR_CTRL_AND_MASK 0x0f
#define  RT_MOTOR_REVERSE_MASK  0x08
#define  LT_MOTOR_REVERSE_MASK  0x80

/* -----------------------------------------
   macros
----------------------------------------- */

#define  ALL_STOP               setMotorSpeed(0, DIR_FWD, 0, DIR_FWD)

#define  SIGN(A)                (((int) (A) >= 0) ? 1 : -1)

/* -----------------------------------------
   function prototypes
----------------------------------------- */

void wait(void);     // state functions
void move(void);
void report(void);

void sendMotionStats(void);
void setMotorSpeed(int, int, int, int);
int  activateMotors(void);
void calculateMotorBias(int, int*, int*);
void controlModeNone(void);
void controlModePID(int);

/* -----------------------------------------
   types
----------------------------------------- */

// state name types
typedef enum {
              STATE_WAIT = 0,  /* wait for command    */
              STATE_MOVE,      /* motors running      */
              STATE_REPORT     /* report travel       */
             } State_Type;

// motor state and data
typedef struct motorState_tag
               {
                int  nClicksToGo;      // clicks to travel
                int  nClicksTraveled;  // clicks already traveled
                WORD wMeasureInterval; // clock ticks since last measurement
                int  nDeltaTravel;     // clicks since last measurement
                int  nSpeedBias;       // offset from default motor speed
                int  nDirection;       // motor direction
               };

/* -----------------------------------------
   task globals
----------------------------------------- */

static int        nSenseTask   = 0;
static int        nNavTask     = 0;
static State_Type controlState = STATE_WAIT;

int          nDumpFull         = 0;

int          nSteering         = 0;
int          nDefSpeedIndx     = DEF_SPD_INDX;

int          nActiveCmd        = 0;
int          nAvoidDistance    = 999;
WORD         wClicksTOV;

struct motorState_tag MotorState[2] = {{0, 0, 0, 0, 0, DIR_UNDEF},
                                       {0, 0, 0, 0, 0, DIR_UNDEF}};

const BYTE motorCtrlOrMask[MAX_SPD_INDX + 2] =  // motor speed control bits
                     {
                      0x07, //  0 stop
                      0x07, //  1 min
                      0x06, //  2
                      0x05, //  3
                      0x04, //  4
                      0x03, //  5
                      0x02, //  6
                      0x01, //  7
                      0x00, //  8 max
                      0x00  //  fill byte
                     };

/* -----------------------------------------
   externals
----------------------------------------- */

extern BYTE    bUtil;                    // defined in amrex.c
extern int     nSenseSampleInterval;
extern int     nAlgorithm;
extern int     nDeadBand;
extern QmnFP_t qCp;
extern QmnFP_t qCi;
extern QmnFP_t qCd;

/* -----------------------------------------
   miscellaneous functions
----------------------------------------- */

/* -----------------------------------------
   sendMotionStats()

   calculates distance and arc data
   travelled, and sends info to 'nav' task.
----------------------------------------- */
void
sendMotionStats(void)
{
 int     nTravel = 0;
 QmnFP_t qTravel = 0;

 switch ( nActiveCmd )
    {
     case MV_FORWARD_CMD:
     case MV_BACK_CMD:
     case MV_LINEAR_CMD:
          // calculate platform center travel in [cm] and send to 'nav'
          nTravel = (MotorState[L_MOTOR].nClicksTraveled + MotorState[R_MOTOR].nClicksTraveled) / 2;
          qTravel = INT2Qmn(nTravel);
          qTravel = MUL_Qmn(qTravel, CM_PER_CLICK_FP);
          putMsg(nNavTask, MV_LINEAR_STAT, (WORD) qTravel, DW_DONT_CARE);
          break;

     case MV_RIGHT_CMD:
     case MV_LEFT_CMD:
     case MV_PTURN_CMD:
          // convert clicks to arc in Qm.n [rad] and send to 'nav'
		    // **** check if this calculation is correct ****
          nTravel = (MotorState[L_MOTOR].nClicksTraveled - MotorState[R_MOTOR].nClicksTraveled) / 2;
          qTravel = INT2Qmn(nTravel);
          qTravel = DIV_Qmn(qTravel, BASE_CENTER_CL_FP);
          putMsg(nNavTask, MV_TURN_STAT, (WORD) qTravel, DW_DONT_CARE);
          break;

     default:  // just in case 'nActiveCmd' gets corrupted ...
          putDebugMsg(DB_TRACE, DB_BAD_PARAM, __LINE__); // @@ variable 'nActiveCmd' corrupted in sendMotionStats()
    }
    
 // reset clicks traveled after reporting
 //   for the following four commands it is assumed that
 //   reporting was forced by threshold crossing
 if ( nActiveCmd == MV_FORWARD_CMD || 
      nActiveCmd == MV_BACK_CMD ||
      nActiveCmd == MV_RIGHT_CMD ||
      nActiveCmd == MV_LEFT_CMD )
 {
    MotorState[L_MOTOR].nClicksTraveled = 0;
    MotorState[R_MOTOR].nClicksTraveled = 0;
 }
}

/* -----------------------------------------
   setMotorSpeed()

   this function recieves speed setting and direction
   for left and right motors and sets the
   control bit settings for the motor control
   registers UTIL and MOTOR_CTRL
----------------------------------------- */
void
setMotorSpeed(int nSpeedIndxR,
              int nDirR,
              int nSpeedIndxL,
              int nDirL)
{
 register BYTE  bOrMask;
 static   BYTE  bMotorControlWord = 0x77; // initial motor control = forward

 CRIT_SEC_START;

 // setup motors on/off state
 if ( nSpeedIndxR == 0 )
    bUtil &= (~MASK_LMON); // motor off
 else
    bUtil |= MASK_LMON;    // motor on

 if ( nSpeedIndxL == 0 )
    bUtil &= (~MASK_RMON); // motor off
 else
    bUtil |= MASK_RMON;    // motor on

 // setup lower nibble for right motor
 bOrMask = motorCtrlOrMask[nSpeedIndxR];
 bMotorControlWord &= RT_MOTOR_CTRL_AND_MASK;
 bMotorControlWord |= bOrMask;
 if ( nDirR == DIR_REV )
    bMotorControlWord |= RT_MOTOR_REVERSE_MASK;
 else
    bMotorControlWord &= (~RT_MOTOR_REVERSE_MASK);

 // setup high nibble for left motor
 bOrMask = motorCtrlOrMask[nSpeedIndxL];
 bOrMask  <<= 4;
 bMotorControlWord &= LT_MOTOR_CTRL_AND_MASK;
 bMotorControlWord |= bOrMask;
 if ( nDirL == DIR_REV )
    bMotorControlWord |= LT_MOTOR_REVERSE_MASK;
 else
    bMotorControlWord &= (~LT_MOTOR_REVERSE_MASK);

#ifdef __HARNESS__
 printf("\tbMotorControlWord=0x%x, bUtil=0x%x\n", bMotorControlWord, bUtil);
#else
 outp(MOTOR_CTRL, bMotorControlWord);
 outp(UTIL, bUtil);
 putDebugMsg(DB_TRACE, DB_INFO, (DWORD) (bMotorControlWord * 256 + bUtil)); // @@ debug info for setMotorSpeed() in harness
#endif

 CRIT_SEC_END;
}

/* -----------------------------------------
   activateMotors()

   send activation command to motors
   this is ok for commanding Point-turn and linear travel

   returns:
   0 - both motors commanded off
   1 - at least one motor is still commanded to run
----------------------------------------- */
int
activateMotors(void)
{
 int nClicksToGo;
 int nClicksTraveled;
 int nSpeedIndxR,
     nSpeedIndxL;

 static DWORD dwPrevSendTime = 0;

 // distance calculation of wheels
 nClicksToGo     = abs(MotorState[R_MOTOR].nClicksToGo);

 nClicksTraveled = (abs(MotorState[R_MOTOR].nClicksTraveled) +
                    abs(MotorState[L_MOTOR].nClicksTraveled)) / 2;

 // check completion of travel only for 'limited' move commands
 if ( (nActiveCmd == MV_LINEAR_CMD) || (nActiveCmd == MV_PTURN_CMD) )
 {
    if ( nClicksTraveled >= nClicksToGo )
    {
       ALL_STOP;
       return 0;
    }

    // attempt to minimize distance overshoot by limiting
    // motor speed to minimum speed
    /*
    if ( (nClicksToGo - nClicksTraveled) < WHEEL_CIRCUMFERENCE_CL)
    {
       nDefSpeedIndx = MV_SPEED_MIN;
    }
    */
 }

 // report traveled distance before overflowing wheel click counters
 // this will be true for the 'inlimited' move commands
 /*
 if ( nClicksTraveled > TRAVEL_DIST_REPORT_THRESH )
    sendMotionStats();
 */

 // initialize timer
 if ( dwPrevSendTime == 0 )
 	dwPrevSendTime = getGlobalTicks();

 // report travel distance based on a timer interval
 if ( (getGlobalTicks() - dwPrevSendTime) >= TRAVEL_DIST_REPORT_INTERVAL )
    {
     sendMotionStats();
     dwPrevSendTime = getGlobalTicks();
    }

 // calculate speed indexes for motors
 nSpeedIndxR = nDefSpeedIndx + MotorState[R_MOTOR].nSpeedBias;

 if ( nSpeedIndxR > MAX_SPD_INDX )
    nSpeedIndxR = MAX_SPD_INDX;

 if ( nSpeedIndxR < 0 )
    nSpeedIndxR = 0;

 nSpeedIndxL = nDefSpeedIndx + MotorState[L_MOTOR].nSpeedBias;

 if ( nSpeedIndxL > MAX_SPD_INDX )
    nSpeedIndxL = MAX_SPD_INDX;

 if ( nSpeedIndxL < 0 )
    nSpeedIndxL = 0;

 // send move command to motors
 // *** NOTE: an undefined direction ( DIR_UNDEF = '0') will run motors forward
 setMotorSpeed(nSpeedIndxR,
               MotorState[R_MOTOR].nDirection,
               nSpeedIndxL,
               MotorState[L_MOTOR].nDirection);

 return 1;
}

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

/* -----------------------------------------
   controlModeNone()

   platform motors will run in open loop
----------------------------------------- */
void
controlModeNone(void)
{
 // use steering command and turn directly into motor speed spread
 // this is a very simple and easy way to do it...

 int  nLocalSteeringVar;
 int  nDeltaSpreadR;
 int  nDeltaSpreadL;

 nLocalSteeringVar = nSteering;

 // this will compensate for traveling in reverse when in MV_BACK_CMD
 // in this commands both motors run in same direction, so only need
 // to test one of them for travel direction
 if ( MotorState[R_MOTOR].nDirection == DIR_REV )
    {
     nLocalSteeringVar = -nLocalSteeringVar;
    }

 calculateMotorBias(nLocalSteeringVar, &nDeltaSpreadR, &nDeltaSpreadL);

 MotorState[R_MOTOR].nSpeedBias = nDeltaSpreadR;
 MotorState[L_MOTOR].nSpeedBias = nDeltaSpreadL;
}

/* -----------------------------------------
   controlModePID()

   PID motion control.
   will set motor speed based on deviation
   from straight line travel.
   when nReset is TRUE the internal variables are 0'ed
----------------------------------------- */
void
controlModePID(int nReset)
{
 static QmnFP_t qErrk_1 = 0;
 static QmnFP_t qErrSum = 0;

 int            nDr;
 int            nDl;
 int            nErrk;

 QmnFP_t        qErrk;
 QmnFP_t        qUk;
 QmnFP_t        qIntegLim;

 int            nUk;
 int            nDeltaSpreadR;
 int            nDeltaSpreadL;

 if ( nReset )
 	{
    qErrk_1 = 0;
    qErrSum = 0;
    return;
   }

 /* PID controller

  Arc = B x Theta

                  arc:   platform rotation angle [click]
                  B:     platform width wheel-to-wheel [click]
                  Theta: platform rotation [rad]

  Dl(k) - Dr(k) = B x Theta(k)

                  Dl(k): delta left wheel travel sample [click]
                  Dr(k): delta right wheel travel sample [click]
                  CW turn Theta > 0
                  CCW turn Theta < 0

  platform rotation atsample k
  ----------------------------
      T_k = ( Dl_k - Dr_k ) / B since B is constant and Theta is directly relted to delta clicks
      will only use delta clicks:

  (1) T_k = ( Dl_k - Dr_k )

  angle error relative to commanded rotation
  ------------------------------------------
  (2) Err_k = Tcmd_k - T_k

  control output equation
  -----------------------
  (3) U_k =   Cp x Err_k
            + Ci x ErrSum
            + Cd (Err_k - Err_k_1 )

  using external globals initialized from command.lst:

    extern QmnFP_t qCp;
    extern QmnFP_t qCi;
    extern QmnFP_t qCd;

  (4) scale U_k and fit to motor control spread
  -----------------------------------------------

 */

 nDr = MotorState[R_MOTOR].nDeltaTravel;
 nDl = MotorState[L_MOTOR].nDeltaTravel;

 // PID

 qIntegLim = DIV_Qmn(MAX_SPREAD, qCi);            // integrator "wind up" limit
 qIntegLim = MUL_Qmn(qIntegLim, INTEG_LIMIT);

 nErrk = nDr - nDl;                               // was: nErrk = nSteering - nDl + nDr;

 if ( abs(nErrk) <= nDeadBand ) nErrk = 0;        // signal filter

 qErrk = INT2Qmn(nErrk);                          // convert to Qmn

 qErrSum += qErrk;                                // load integrator

 if ( qErrSum > qIntegLim ) qErrSum = qIntegLim;  // integrator limit
 if ( qErrSum < -qIntegLim ) qErrSum = -qIntegLim;

 qUk = MUL_Qmn(qCp, qErrk) +                      // calculate PID
       MUL_Qmn(qCi, qErrSum) +
       MUL_Qmn(qCd, (qErrk - qErrk_1));

 qErrk_1 = qErrk;                                 // save err for next cycle

 nUk = Qmn2INT(qUk);

 // this will compensate for traveling in reverse when
 // in N_LINEAR_CMD and N_BACK_CMD; in these commands both motors run
 // in same direction, so only need to test one of them for travel direction
 if ( MotorState[R_MOTOR].nDirection == DIR_REV )
    {
     nUk = -nUk;
    }

 calculateMotorBias(nUk, &nDeltaSpreadR, &nDeltaSpreadL);

 MotorState[R_MOTOR].nSpeedBias = nDeltaSpreadR;
 MotorState[L_MOTOR].nSpeedBias = nDeltaSpreadL;

 if ( !nDumpFull ) nDumpFull = putDataLog((WORD) ((nErrk << 8) + nUk)); 				// data log
 if ( !nDumpFull ) nDumpFull = putDataLog((WORD) ((nDeltaSpreadR << 8) + nDeltaSpreadL));
 //if ( !nDumpFull ) nDumpFull = putDataLog((WORD) nDeltaSpreadL);
} // end controlModePID()

/* -----------------------------------------
   state functions
----------------------------------------- */

/* -----------------------------------------
   wait()

   initial state while waiting for commands
   from 'nav' task.
----------------------------------------- */
void
wait(void)
{
 int     nMsg;
 WORD    wPayload;
 DWORD   dwPayload;

 QmnFP_t qTemp;
 int     nTemp;

 // initialize distance and velocity variables
 MotorState[L_MOTOR].nClicksToGo      = 0;
 MotorState[L_MOTOR].nClicksTraveled  = 0;
 MotorState[L_MOTOR].wMeasureInterval = 0;
 MotorState[L_MOTOR].nDeltaTravel     = 0;
 MotorState[L_MOTOR].nSpeedBias       = 0;
 MotorState[L_MOTOR].nDirection       = DIR_UNDEF;

 MotorState[R_MOTOR].nClicksToGo      = 0;
 MotorState[R_MOTOR].nClicksTraveled  = 0;
 MotorState[R_MOTOR].wMeasureInterval = 0;
 MotorState[R_MOTOR].nDeltaTravel     = 0;
 MotorState[R_MOTOR].nSpeedBias       = 0;
 MotorState[R_MOTOR].nDirection       = DIR_UNDEF;

 nSteering                            = 0;

 controlModePID(1);                   // reset PID controller's static variables

 flushMsgQ();

 // wait for message
 nMsg = waitMsg(__ANY__, 0, &wPayload, &dwPayload);

 // parse message
 switch ( nMsg )
    {
     // navigation commands
     case MV_LINEAR_CMD: // move a specified distance forward or back
          qTemp = (QmnFP_t) wPayload;                    // get distance in [cm]
          qTemp = MUL_Qmn(qTemp, CLICKS_PER_CM_FP);      // convert to clicks
          nTemp = Qmn2INT(qTemp);
          MotorState[R_MOTOR].nDirection  = SIGN(nTemp); // wheels go same direction
          MotorState[L_MOTOR].nDirection  = SIGN(nTemp);
          MotorState[R_MOTOR].nClicksToGo = nTemp;       // wheels go same distance
          MotorState[L_MOTOR].nClicksToGo = nTemp;

          // save command
          nActiveCmd = nMsg;

          // change state
          controlState = STATE_MOVE;
          break;

     case MV_PTURN_CMD:  // conver arc into clicks to turn per wheel
          qTemp = (QmnFP_t) wPayload;                    // get turn arc in [rad]
          MotorState[R_MOTOR].nDirection = SIGN(-qTemp); // wheels go in opposite dir
          MotorState[L_MOTOR].nDirection = SIGN(qTemp);
          qTemp = MUL_Qmn(qTemp, BASE_CENTER_CL_FP);     // calculate arc in clicks
          nTemp = Qmn2INT(qTemp);
          MotorState[R_MOTOR].nClicksToGo = -nTemp;      // wheels go same arc distance
          MotorState[L_MOTOR].nClicksToGo = nTemp;

          // save command
          nActiveCmd = nMsg;

          // change state
          controlState = STATE_MOVE;
          break;

	  case MV_FORWARD_CMD:   // move forward
          MotorState[R_MOTOR].nDirection  = DIR_FWD;
			 MotorState[L_MOTOR].nDirection  = DIR_FWD;
          nActiveCmd = nMsg;
          controlState = STATE_MOVE;
			 break;

	  case MV_BACK_CMD:  // move backward
     	    MotorState[R_MOTOR].nDirection  = DIR_REV;
          MotorState[L_MOTOR].nDirection  = DIR_REV;
          nActiveCmd = nMsg;
          controlState = STATE_MOVE;
			 break;

     case MV_LEFT_CMD: // left point turn
          MotorState[R_MOTOR].nDirection  = DIR_FWD;
			 MotorState[L_MOTOR].nDirection  = DIR_REV;
          nActiveCmd = nMsg;
          controlState = STATE_MOVE;
			 break;

     case MV_RIGHT_CMD: // right point turn
          MotorState[R_MOTOR].nDirection  = DIR_REV;
			 MotorState[L_MOTOR].nDirection  = DIR_FWD;
          nActiveCmd = nMsg;
          controlState = STATE_MOVE;
			 break;

     case MS_DATA_LOG: // trigger a trace dump
          dumpDataLog((int) wPayload);
          break;

     case MS_PID:      // override PID constants
          qCp = (QmnFP_t) wPayload;
          qCi = (QmnFP_t) (dwPayload >> 16);
          qCd = (QmnFP_t) dwPayload;
          break;
          
     case MV_SPEED_CMD: // override default speed setting
          nDefSpeedIndx = (int) wPayload;
          if ( nDefSpeedIndx < MV_SPEED_MIN )
             nDefSpeedIndx = MV_SPEED_MIN;
          if ( nDefSpeedIndx > MV_SPEED_MAX )
             nDefSpeedIndx = MV_SPEED_MAX;
          break;

     case MS_CONTROL_ALGOR: // override control loop algorithm
          nAlgorithm = (int) wPayload;
          break;

     case MS_PING: // reply to ping requests
          putMsg((BYTE) wPayload, MS_PING, W_DONT_CARE, DW_DONT_CARE);
          break;

     default:   // default handler
          putDebugMsg(DB_TRACE, DB_BAD_MSG, __LINE__); // @@ bad message received in wait()
    } //switch on bMsg
}

/* -----------------------------------------
   move()

   this state monitors speed and distance data,
   and shuts motors off if an error has been encountered.
   sensor data is monitored for errors,
   and bumper switch closure.

   NOTES:
   1. in case of error (TOV or overrun) last
      value sent is the most valid
      so new data is not sent to nav.
   2. bumper switches that are released
      are only reported.
----------------------------------------- */
void
move(void)
{
 BYTE  bMsg;
 WORD  wPayload;
 DWORD dwPayload;

 WORD  wMeasureInterval;
 int   nLDeltaTravel;
 int   nRDeltaTravel;

 // send move command to activate motors
 if ( activateMotors() == 0 )
    {
     // if both motors are off then exit this state
     controlState = STATE_REPORT;
     return;
    }

 /* wait for message */
 bMsg = waitMsg(__ANY__, wClicksTOV, &wPayload, &dwPayload);

 switch ( bMsg )
    {
     case Q_EMPTY: /* sensor time out */
          /* to-do: this will not work as a sensor TOV error if t_ctrlex() gets
                    peridic steering commands. need to compare current system clock
                    with last 'MotorState[L_MOTOR].wMeasureInterval' or 'wMeasureInterval'
                    against TOV.
          */
          ALL_STOP;
          putMsg(nNavTask, SN_ERROR, SN_ENCODER_TOV, DW_DONT_CARE);
          sendMotionStats();
          controlState = STATE_WAIT;
          break;

     case SN_ERROR: /* S_ENCODER_OVR   */
          /* to-do: does not distinguish between error types in 'wPayload'
          */
          ALL_STOP;
          putMsg(nNavTask, SN_ERROR, SN_ENCODER_OVR, DW_DONT_CARE);
          sendMotionStats();
          flushMsgQ();
          controlState = STATE_WAIT;
          break;

     case SN_BUMPER_SW: /* bupber switches */
          /* to-do: change to a switch-case construct
          */
          if ( wPayload == SN_RIGHT_ON ||
               wPayload == SN_LEFT_ON ||
               wPayload == SN_MID_ON )
             { /* bumper switches pressed, stop motors and notify */
              ALL_STOP;
              putMsg(nNavTask, SN_OBSTACLE, wPayload, DW_DONT_CARE);
              controlState = STATE_REPORT;
             }
          else /* switch released, just notify */
             putMsg(nNavTask, SN_OBSTACLE, wPayload, DW_DONT_CARE);
          break;

	  case SN_CLICKS:
          nLDeltaTravel    = (int) (dwPayload & 0x0000ffffL); /* left wheel clicks   */
          nRDeltaTravel    = (int) (dwPayload >> 16);         /* right wheel clicks  */
          wMeasureInterval = wPayload;                        /* measurment interval */

          MotorState[L_MOTOR].nClicksTraveled += nLDeltaTravel; /* left wheel   */
          MotorState[L_MOTOR].nDeltaTravel     = nLDeltaTravel;
          MotorState[L_MOTOR].wMeasureInterval = wMeasureInterval;

          MotorState[R_MOTOR].nClicksTraveled += nRDeltaTravel; /* right wheel  */
          MotorState[R_MOTOR].nDeltaTravel     = nRDeltaTravel;
          MotorState[R_MOTOR].wMeasureInterval = wMeasureInterval;

          switch ( nActiveCmd )
          {
                // for linear movement always use internal PID controller
           case MV_LINEAR_CMD:
                controlModePID(0);
                break;

                // use internal PID only if selected, note that in this case steering
                // commands will be ignored! (see N_LIN_STEERING handler)
           case MV_FORWARD_CMD:
           case MV_BACK_CMD:
                if ( nAlgorithm == MS_PID_CTRL )
                   controlModePID(0);
                break;

           default:;
          }

          break;

     case MV_LIN_STEER:
          // provide offset for steering only for unlimited move commands
          // and only when PID selection is 'off', otherwise ignore steering
          if (  nAlgorithm == MS_OPEN_LOOP &&
               (nActiveCmd == MV_FORWARD_CMD || nActiveCmd == MV_BACK_CMD) )
             {
              // get steering and limit to max. spread
              nSteering = (int) wPayload;
              if ( abs(nSteering) > MAX_SPREAD )
                 nSteering = SIGN(nSteering) * MAX_SPREAD;
              controlModeNone();
             }
          else
             {
              putDebugMsg(DB_TRACE, DB_BAD_MSG, __LINE__); // @@ MV_LIN_STEER received when mode is MS_PID_CTRL in move()
             }
          break;

     case MV_STOP_CMD: // first abort current move by ALL_STOP
          ALL_STOP;
          controlState = STATE_REPORT;
          break;

     case MV_SPEED_CMD: // override default speed setting
          nDefSpeedIndx = (int) wPayload;
          if ( nDefSpeedIndx < MV_SPEED_MIN )
             nDefSpeedIndx = MV_SPEED_MIN;
          if ( nDefSpeedIndx > MV_SPEED_MAX )
             nDefSpeedIndx = MV_SPEED_MAX;
          break;

     case MS_PING: // respond to ping requests even while moving
          putMsg((BYTE) wPayload, MS_PING, W_DONT_CARE, DW_DONT_CARE);
          break;

     default: // default handler
          putDebugMsg(DB_TRACE, DB_BAD_MSG, __LINE__); // @@ bad message received in move()
    } // switch on bMsg
}

/* -----------------------------------------
   report()

   on entry to this state, motors are *off*.
   state waits for last travel data, and after
   none are received for CLICKS_TOV, it sends
   totals to 'nav' task.
----------------------------------------- */
void
report(void)
{
 BYTE        bMsg;
 WORD        wPayload;
 DWORD       dwPayload;

 // wait for last sensor messages
 bMsg = waitMsg(__ANY__, wClicksTOV, &wPayload, &dwPayload);

 switch ( bMsg )
    {
     case Q_EMPTY: /* no more data */
          sendMotionStats();
          controlState = STATE_WAIT;
          break;

     case SN_CLICKS:
          MotorState[R_MOTOR].nClicksTraveled += (int) (dwPayload >> 16); /* right wheel */
          MotorState[L_MOTOR].nClicksTraveled += (int) dwPayload;         /* left wheel  */
          break;

     case SN_BUMPER_SW: /* discard bumpber switches and distance */
     case SN_DISTANCE:
          break;

     /* default handler */
     default:
          putDebugMsg(DB_TRACE, DB_BAD_MSG, __LINE__); // @@ bad message received in report()
    }
}

/* -----------------------------------------
   task function code
----------------------------------------- */

void
t_control(void)
{
 static State_Type prevState;

 // initialize motor control
 ALL_STOP;

 // attach to nav and sessor tasks
 nSenseTask = getTidByName(TASK_NAME_SENSOR);
 if ( nSenseTask == 0 )
    {
     putDebugMsg(DB_TRACE, DB_BAD_TNAME, __LINE__); // @@ bad task name for TASK_NAME_SENSOR
     return;
    }

 nNavTask = getTidByName(TASK_NAME_NAV);
 if ( nNavTask == 0 )
    {
     putDebugMsg(DB_TRACE, DB_BAD_TNAME, __LINE__); // @@ bad task name for TASK_NAME_NAV
     return;
    }

 // initialize state machine and sensor time-out
 controlState = STATE_WAIT;
 prevState    = ~STATE_WAIT;
 wClicksTOV   = SENSOR_TOV * ((WORD) nSenseSampleInterval);

 // start control loop
 while (1)
    {
     //print("ctrl\r\n");

     if ( prevState != controlState )
        {
         putDebugMsg(DB_TRACE, DB_INFO, (DWORD) controlState); // @@ state change info
         prevState = controlState;
        }

     switch ( controlState )
     {
      case STATE_WAIT:
           wait();
           break;

      case STATE_MOVE:
           move();
           break;

      case STATE_REPORT:
           report();
           break;

      default:
           ALL_STOP;
           controlState = STATE_WAIT;
           putDebugMsg(DB_TRACE, DB_BAD_PARAM, __LINE__); // @@ variable 'controlState' contains unrecognized state
     }
    } // endless loop
}

