/* ----------------------------------------------------

   I/O test program


---------------------------------------------------- */

#include <stdlib.h>
#include <conio.h>
#include <stdio.h>
#include <dos.h>
#include <time.h>

#define  MENU_1 "\n [I]/O port\n [S]FR\n [L]eft motor\n [R]ight motor\n [W]atch dog\n [D]isplay\n [Q]uit\n\n ? "

#define  MENU_2 "\n [R]ead\n [W]rite\n\n ? "

#define  MENU_3 "\n\n Address [hex]:"

#define  MENU_4 "\n\n Data [hex]:"

unsigned char bPort300 = 0x00;
unsigned char bPort301 = 0x77;


void
trigger( int nSeconds )
{
   time_t start, end;

   start = time(NULL);
   end   = start;

   while ( difftime(end, start) <= nSeconds )
      {
       bPort300 |= 0x20;
       outp(0x300, bPort300);
       bPort300 &= 0xdf;
       outp(0x300, bPort300);

       end = time(NULL);
      }
}

void
main(void)
{
 enum { IO_PORT, SFR_PORT, L_MOTOR, R_MOTOR, W_DOG, DISPLAY } nType;
 enum { READ, WRITE } nAction;

 int           nVal  = 0;
 int           nPort = 0;
 unsigned char i;

 while ( 1 )
    {
     printf(MENU_1);
     switch ( getche() )
        {
         case 'I':
         case 'i':
              nType = IO_PORT;
              break;
         case 'S':
         case 's':
              nType = SFR_PORT;
              break;
         case 'L':
         case 'l':
              nType = L_MOTOR;
              break;
         case 'R':
         case 'r':
              nType = R_MOTOR;
              break;
         case 'W':
         case 'w':
              nType = W_DOG;
              break;
         case 'D':
         case 'd':
              nType = DISPLAY;
              break;
         case 'Q':
         case 'q':
              exit(EXIT_SUCCESS);
                  break;
        }

     if ( nType == IO_PORT || nType == SFR_PORT )
        {
         printf(MENU_3);
         scanf("%x", &nPort);

         printf(MENU_2);
         switch ( getche() )
            {
             case 'W':
             case 'w':
                  printf(MENU_4);
                  scanf("%x", &nVal);
                  nAction = WRITE;
                  break;
             case 'R':
             case 'r':
                  nAction = READ;
                  break;
            }

         if ( nAction == READ )
            {
             if ( nType == SFR_PORT )
                nVal = *( ((unsigned char far*) MK_FP(0xf000, 0xff00)) + nPort );
             else
                nVal = inp(nPort);

             printf("\n\ninput $%x from port $%x\n", nVal, nPort);
            }
         else
            {
             printf("\n\noutput $%x to port $%x\n", nVal, nPort);
             if ( nType == SFR_PORT )
                *( ((unsigned char far*) MK_FP(0xf000, 0xff00)) + nPort) = nVal;
             else
                outp(nPort, nVal);
             } /* if ( nAction == READ ) */
        } /* handle port direct I/O */
     else
        {
         switch ( nType )
            {
             case DISPLAY:
                  bPort300 = 0x10;
                  for ( i = 0; i < 16; i++ )
                     {
                      bPort300 &= 0xf0;
                      bPort300 |= i;
                      outp(0x300, bPort300);
                      sleep(1);
                     }
                  outp(0x300, 0x00);
                  break;
             case L_MOTOR:
                  bPort301 &= 0x0f;      /* forward max */
                  outp(0x301, bPort301);
                  bPort300 |= 0x40;
                  outp(0x300, bPort300);
                  trigger(4);

                  bPort300 &= 0xbf;      /* reverse max */
                  outp(0x300, bPort300);
                  bPort301 |= 0x80;
                  outp(0x301, bPort301);
                  sleep(1);
                  bPort300 |= 0x40;
                  outp(0x300, bPort300);
                  trigger(4);

                  bPort301 |= 0x70;      /* stop */
                  outp(0x301, bPort301);
                  bPort300 &= 0xbf;
                  outp(0x300, bPort300);
                  break;
             case R_MOTOR:
                  bPort301 &= 0xf0;      /* forward max */
                  outp(0x301, bPort301);
                  bPort300 |= 0x80;
                  outp(0x300, bPort300);
                  trigger(4);

                  bPort300 &= 0x7f;      /* reverse max */
                  outp(0x300, bPort300);
                  bPort301 |= 0x08;
                  outp(0x301, bPort301);
                  sleep(1);
                  bPort300 |= 0x80;
                  outp(0x300, bPort300);
                  trigger(4);

                  bPort301 |= 0x07;      /* stop */
                  outp(0x301, bPort301);
                  bPort300 &= 0x7f;
                  outp(0x300, bPort300);
                  break;
             case W_DOG:
                  trigger(10);
                  break;
            }
         outp(0x300, 0x00);
         outp(0x301, 0x77);
        } /* handle motor and watchdog test */

    } /* while (1) */
}
