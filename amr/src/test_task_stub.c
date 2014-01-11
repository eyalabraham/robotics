/* ***************************************************************************

  TEST_NAV_STUB.C

  This app starts 2 tasks: NAV_STUB and LOOPBACK
  Will use this to test all T_NAV_STUB code paths end to end through CON link
  to PC

  5 16 2010 - Created

*************************************************************************** */

#include  <assert.h>

#include  "smte.h"
#include  "names.h"

#include  "t_task_stub.h"
#include  "t_loopback.h"

void main(void)
{
 assert(conCommInit(conCommIsr));   // macro service receive on serial console

 // tasks:           task        stack  ticks  msg  name
 assert(registerTask(t_nav_stub, 1024,  4,     10,  TASK_NAME_TASK_STUB));
 assert(registerTask(t_loopback, 128,   4,     2,   TASK_NAME_LOOPBACK));

 startScheduler(__TRC_LVL2__, __TIMER_OFF__, 1);
}
