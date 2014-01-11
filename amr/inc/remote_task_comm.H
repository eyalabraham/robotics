/* ***************************************************************************

  REMOTE_TASK_COMM.H

  Header file for a task stub.
  This task stub forwards messages to and from a NAV task that resides on PC
  through COM1.
  This header file is used in t_task_stub.c, comredir.c, and rc_smte.c

  April 2 2012 - modified message structure

*************************************************************************** */

#ifndef  __REMOTE_TASK_COMM__
#define  __REMOTE_TASK_COMM__

#include "internal.h"

/* -----------------------------------------
   message definiitons
----------------------------------------- */

#define TASK_MSG_TOKEN    "!tk"
#define TASK_MESSAGE_LEN  13     // length of char sequence of task msg
                                 // !tk         - 3 chars - (3 byte) ID
                                 // <type>      - 1 chars   (1 byte) - see defs below
                                 // <nMsg>      - 2 chars   (1 byte) -or-  <taskName> 7 chars padded with white space
                                 // <wPayload>  - 2 chars   (2 byte)
                                 // <dwPayload> - 4 chars   (4 byte)
                                 // <checksum>  - 1 chars   (1 byte)

#define MSG_TYPE_STRING   0      // index 6 to end is a string name of a task
                                 // any other number is a task ID

/* -----------------------------------------
   message structure type definition
----------------------------------------- */

typedef union T_messageBlock
{
   // message as string of bytes
   unsigned char msgBytesFormat[TASK_MESSAGE_LEN];

   // message as string of bytes for checksum calculation
   struct T_msgChecksumFormat
   {
    unsigned char msgMsgBytesOnly[TASK_MESSAGE_LEN - 1];
    unsigned char bChecksum;
   } msgChecksumFormat;

   // message as 7 characters + type
   struct T_msgStringFormat
   {
    char           header[3];
    unsigned char  bType;
    char           charString[T_NAME_LENGTH];
    unsigned char  bChecksum;
   } msgStringFormat;

   // message as BYTE-WORD-DWORD format
   struct T_msgMessageFormat
   {
    char               header[3];
    unsigned char      bType;
    short int          nMsg;
    unsigned short int wPayload;
    unsigned long      dwPayload;
    unsigned char      bChecksum;
   } msgMessageFormat;
} messageBlock;

#endif // __REMOTE_TASK_COMM__
