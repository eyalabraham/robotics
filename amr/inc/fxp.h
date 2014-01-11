/* ***************************************************************************

  FXP.H

  MACRO definitions for Fixed Point Q9.6

  Data:
   - size:       16 bit integer
   - range:      +1023.96875 .. -1024.0
   - resolution: 0.03125

  see also: http://en.wikipedia.org/wiki/Q_%28number_format%29

  Jan. 16 2007 - Created
  June 7  2010 - updated to support proper Q9.6 format and precision

*************************************************************************** */

#include <math.h>

#ifndef __FXP_H__
#define __FXP_H__

/* =========================================
   constants
========================================= */

#define Qmn_m                   10 // number of bits used for the two's complement integer portion
#define Qmn_n                   5  // number of bits used for the two's complement fraction

#ifdef __FLOAT__
   #define QmnMAX        ((float) +1023.96875) // 2^(Qmn_m) - 2^(-Qmn_n)
   #define QmnMIN        ((float) -1024)       // -2^(Qmn_m)
   #define QmnRESOLUTION ((float) 0.03125)     // 2^(-Qmn_n)
#endif

#define K        (1 << (Qmn_n-1)) // multiplication rounding constant

/* =========================================
   types
========================================= */

typedef short int QmnFP_t;              // fixed point type

/* =========================================
   macros
========================================= */

// addition
#define ADD_Qmn(a,b) ((a)+(b))

// subtraction
#define SUB_Qmn(a,b) ((a)-(b))

// convert int to a Qm.n equivalent
#define INT2Qmn(a)   ((a)<<(Qmn_n)) 

// convert Qm.n to int ** loosing precision ** 
#define Qmn2INT(a)   ((a)>>(Qmn_n))

/* =========================================
   functions
========================================= */

#ifdef __FLOAT__

// Qm.n to float
float
Qmn2FLOAT(QmnFP_t);

// float to Qm.n
QmnFP_t
FLOAT2Qmn(float);

#endif //__FLOAT__

// multiplication
QmnFP_t
MUL_Qmn(QmnFP_t,
        QmnFP_t);

// division
QmnFP_t
DIV_Qmn(QmnFP_t,
        QmnFP_t);

#endif  // __FXP_H__
