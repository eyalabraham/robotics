/* ***************************************************************************

  FXP.C

  MACRO definitions for Fixed Point Q9.6

  Data:
   - size:       16 bit integer
   - range:      +1023.96875 .. -1024.0
   - resolution: 0.03125

  see also: http://en.wikipedia.org/wiki/Q_%28number_format%29

  Jan. 16 2007 - Created
  June 7  2010 - updated to support proper Q9.6 format and precision

*************************************************************************** */

#include "fxp.h"

/* =========================================
   functions
========================================= */

#ifdef __FLOAT__

#include <float.h>
//#include <values.h>

// ----------------------------------------
//  Qm.n to float
// ----------------------------------------
float
Qmn2FLOAT(QmnFP_t a)
{
 // convert the number to floating point and multiply by 2^-n
 return (float) a / pow( (float) 2, (float) Qmn_n);
}

// ----------------------------------------
//  float to Qm.n
// ----------------------------------------
QmnFP_t
FLOAT2Qmn(float a)
{
 float temp;

 // Multiply the floating point number by 2^n and ...
 temp = a * pow( (float) 2, (float) Qmn_n);

 // ... round to the nearest integer
 if ( a < 0 )
    {
	 return ceil(temp - 0.5);
    }
 else
    {
	 return floor(temp + 0.5);
    }
}

#endif // __FLOAT__

// ----------------------------------------
//  multiplication
// ----------------------------------------
QmnFP_t
MUL_Qmn(QmnFP_t a,
        QmnFP_t b)
{
 signed long int  temp;

 temp = (long int) a * (long int) b;
 // Rounding; mid values are rounded up
 temp += K;
 // Correct by dividing by base and return
 return (QmnFP_t) (temp >> Qmn_n);
}

// ----------------------------------------
//  division
// ----------------------------------------
QmnFP_t
DIV_Qmn(QmnFP_t a,
        QmnFP_t b)
{
 signed long int temp;

 // pre-multiply by the base (Upscale to Qm.10 so that the result will be in Qm.5)
 temp = (long int) a << Qmn_n;
 // The result will be rounded ; mid values are rounded up.
 temp = temp + b / 2;
 return (QmnFP_t) (temp / b);
}
