/* ***************************************************************************

  T_TASK_STUB.H

  Task header file for a navigation task's stub.
  This task stub forwards messages to and from a NAV task that resides on PC
  through COM1

  April 1 2012 - modified message structure
  May  16 2010 - Created

*************************************************************************** */

#ifndef  __T_TASK_STUB__
#define  __T_TASK_STUB__

/* -----------------------------------------
   task prototypes
----------------------------------------- */

int
conCommInit(void);

void
t_nav_stub(void);

#endif // __T_TASK_STUB__
