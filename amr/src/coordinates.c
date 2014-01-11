/* ***************************************************************************

  coordinates.C

  test coordinate calculations and fixed-point math

*************************************************************************** */
#ifndef WIN32_LEAN_AND_MEAN    // see Note at http://msdn.microsoft.com/en-us/library/windows/desktop/ms737629%28v=vs.85%29.aspx
 #define WIN32_LEAN_AND_MEAN
#endif

#define  _CRT_SECURE_NO_WARNINGS // suppress compiler errors for unsafe scanf()

#include <windows.h>
#include <WinSock2.h>
//#include <Ws2def.h>
#include <Ws2tcpip.h>
#include <stdio.h>
#include <math.h>

#include "fxp.h"

#pragma comment(lib, "Ws2_32.lib")

/* -----------------------------------------
   definitions
----------------------------------------- */

#define  MAX_CHARS         256                       // max length of file specifier
#define  FILE_DIR          "C:\\myDocs\\tempDocs\\"  // input and output file directory
#define  INPUT_FILE        "vectors.txt"             // file with input vectors
#define  OUTPUT_FILE       "track.txt"               // track vectors
#define  MAX_VECTORS       10                        // max number of vectors

#define  __PI__            3.14
#define  __2PI__           (2 * __PI__)

#define DEFAULT_IP         "localhost" // "HOMEDESKTOP"
#define DEFAULT_PORT       "49152"

/* -----------------------------------------
   types
----------------------------------------- */

typedef struct vector_t
{
	float fAngle;
	float fLength;
    float fAngleSum;
    float fDx;
    float fDy;
    float fX;
    float fY;
    float fOriginAngle;
    float fOriginLength;
    float fRealignAngle;
};

/* -----------------------------------------
   function prototypes
----------------------------------------- */

void  CreateVector(float, float, struct vector_t*, struct vector_t*);
void  GetHomeCommand(struct vector_t*, float*, float*);
int   GetQuadrant(struct vector_t*);
void  ClearVector(struct vector_t*);
void  ErrorExit(LPSTR);

/* -----------------------------------------
   globals
----------------------------------------- */

/* -----------------------------------------
   main()
----------------------------------------- */
int
main(void)
{
    FILE*  hFileInVectors;
    char   text[MAX_CHARS] = {0};
    int    nQuit = FALSE;
    int    nVector;
    int    iResult;

    float  fDistance;
    float  fAngle;
    
    struct vector_t vectors[MAX_VECTORS];
    
    WSADATA wsaData;
    struct addrinfo *result = NULL, *ptr = NULL, hints;
    SOCKET UdpSocket = INVALID_SOCKET;

	printf("BUILD: coordinates.c %s, %s\n", __DATE__, __TIME__);
	
	// -------------- CODE -----------
    
    // open input file to get command vectors
    sprintf(text, "%s%s", FILE_DIR, INPUT_FILE);
    if ( (hFileInVectors = fopen(text,"r")) == NULL )
    {
 	    ErrorExit("fopen() could not open vectors file.");
    }

    // initialize Winsock
    iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if ( iResult != 0)
    {
        sprintf(text, "WSAStartup() failed with error: %d\n", iResult);
        fclose(hFileInVectors);
        ErrorExit(text);
    }

    ZeroMemory(&hints, sizeof (hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    hints.ai_flags = AI_PASSIVE;

    // Resolve the local address and port to be used by the server
    iResult = getaddrinfo(DEFAULT_IP, DEFAULT_PORT, &hints, &result);
    if (iResult != 0)
    {
        sprintf(text, "getaddrinfo() failed with error: %d\n", iResult);
        fclose(hFileInVectors);
        WSACleanup();
        ErrorExit(text);
    }

    // Create a SOCKET for the server
    UdpSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (UdpSocket == INVALID_SOCKET)
    {
        sprintf(text, "socket() falied with error: %ld\n", WSAGetLastError());
        fclose(hFileInVectors);
        freeaddrinfo(result);
        WSACleanup();
        ErrorExit(text);
    }

    // Setup the TCP listening socket -- don't need to bind for UDP
    //iResult = bind( UdpSocket, result->ai_addr, (int) result->ai_addrlen);
    //if (iResult == SOCKET_ERROR)
    //{
    //    sprintf(text, "bind() failed with error: %d\n", WSAGetLastError());
    //    freeaddrinfo(result);
    //    closesocket(UdpSocket);
    //    WSACleanup();
    //    ErrorExit(text);
    //}

    ClearVector(&vectors[0]);

    printf("  [#] (  t  , r )    tSum  dX    dY    X     Y    |   R     t   align\n");
    printf(" ---- -----------    ----  ---   ---  ---   ---   |  ---   ---  -----\n");

    nVector = 1;
    while ( !feof(hFileInVectors) && !nQuit && nVector < MAX_VECTORS )
    {
       text[0] = '\0';
       fgets(text, MAX_CHARS, hFileInVectors);
       if ( text[0] != '#' )
       {
          sscanf(text, "%g %g", &fAngle, &fDistance);
          CreateVector(fAngle, fDistance, &vectors[nVector], &vectors[nVector-1]);

          printf("  [%d] (%+.2f,%+.2f) %+.2f %+.2f %+.2f %+.2f %+.2f | %+.2f %+.2f %+.2f\n",
                 nVector, fAngle, fDistance, vectors[nVector].fAngleSum,
                                             vectors[nVector].fDx,
                                             vectors[nVector].fDy,
                                             vectors[nVector].fX,
                                             vectors[nVector].fY,
                                             vectors[nVector].fOriginLength,
                                             vectors[nVector].fOriginAngle,
                                             vectors[nVector].fRealignAngle);

          GetHomeCommand(&vectors[nVector], &fAngle, &fDistance);
          printf("      get home: [%+.2f,%+.2f]\n", fAngle, fDistance);

          sprintf(text, "vector,%+.2f,%+.2f,%+.2f,%+.2f", vectors[nVector-1].fX, vectors[nVector-1].fY, vectors[nVector].fX, vectors[nVector].fY);

          // Echo the buffer to the client
          iResult = sendto(UdpSocket, text, strlen(text), 0, result->ai_addr, (int) result->ai_addrlen);
          if (iResult == SOCKET_ERROR)
          {
              sprintf(text, "sendto() failed with error: %d\n", WSAGetLastError());
              fclose(hFileInVectors);
              closesocket(UdpSocket);
              WSACleanup();
              ErrorExit(text);
          }

          sprintf(text, "obstacle,%+.2f,%+.2f", vectors[nVector].fX, vectors[nVector].fY);

          // Echo the buffer to the client
          iResult = sendto(UdpSocket, text, strlen(text), 0, result->ai_addr, (int) result->ai_addrlen);
          if (iResult == SOCKET_ERROR)
          {
              sprintf(text, "sendto() failed with error: %d\n", WSAGetLastError());
              fclose(hFileInVectors);
              closesocket(UdpSocket);
              WSACleanup();
              ErrorExit(text);
          }

          Sleep(500);
       
          nVector++;
       }
    }
    
    // vector position and error calculation
    

	// close all open handles
    fclose(hFileInVectors);
    freeaddrinfo(result);
    closesocket(UdpSocket);
    WSACleanup();
	return 0;
}

/* -----------------------------------------
   CreateVector()

   get angle and distance travelled and 
   calculate vector data points
----------------------------------------- */
void
CreateVector(float fAngle, float fDistance, struct vector_t* pVectorN, struct vector_t* pVectorN_1)
{
    /* 
           coordinate system 

                     Y ^
                       |
     cartesian (-x,+y) | cartesian (+x,+y)
     polar     (r, -t) | polar     (r, -t)
     quad #3           | quad #4
    -------------------o>>>>----------------> X
     cartesian (-x,-y) | cartesian (+x,-y)
     polar     (r, +t) | polar     (r, +t)
     quad #2           | quad #1
                       |

     (1) starting point assumes facing down the positive direction of 'X' axis
     (2) CW turn is '+', CCW turn in '-'

    */

    float fOriginAngle1;
    float fOriginAngle2;

    // store distance traveled, direction-angle and the sum of directions
    pVectorN->fAngle    = fAngle;
    pVectorN->fLength   = fDistance;
    pVectorN->fAngleSum = pVectorN_1->fAngleSum - fAngle;

    // calculate cartesian coordinates
    pVectorN->fDx       = pVectorN->fLength * cosf(pVectorN->fAngleSum);
    pVectorN->fDy       = pVectorN->fLength * sinf(pVectorN->fAngleSum);
    pVectorN->fX        = pVectorN_1->fX + pVectorN->fDx;
    pVectorN->fY        = pVectorN_1->fY + pVectorN->fDy;

    // calculate polar coordinates
    if ( GetQuadrant(pVectorN) > 0 )
    {
        pVectorN->fOriginLength = sqrt((pVectorN->fX * pVectorN->fX) + (pVectorN->fY * pVectorN->fY));
        fOriginAngle1           = -1 * atan2f(pVectorN->fY, pVectorN->fX);
        fOriginAngle2           = -1 * atan2f(pVectorN->fY, pVectorN->fX);
        if ( fabsf(fOriginAngle1) < fabsf(fOriginAngle2) )
            pVectorN->fOriginAngle = fOriginAngle1;
        else
            pVectorN->fOriginAngle = fOriginAngle2;
    }
    else
    {
        pVectorN->fOriginLength = 0;
        pVectorN->fOriginAngle  = 0;
    }

    // calculate turn angle to realign facing start position
    pVectorN->fRealignAngle = fmodf(pVectorN->fAngleSum, __2PI__);
}

/* -----------------------------------------
   GetHomeCommand()

   calculate turn and distance command to get
   to 'home'/origin based on the location vector set
----------------------------------------- */
void
GetHomeCommand(struct vector_t* pVector, float* pfTurn, float* pfDistance)
{
    *pfDistance = pVector->fOriginLength;
    if ( GetQuadrant(pVector) > 0 )
    {
        *pfTurn = pVector->fRealignAngle - __PI__ + pVector->fOriginAngle;
        *pfTurn = fmodf(*pfTurn, __2PI__);
    }
    else
        *pfTurn = pVector->fRealignAngle;
}

/* -----------------------------------------
   GetQuadrant()

   calculate the quadrant and return quadrant
   number; origin returned as 0.
----------------------------------------- */
int
GetQuadrant(struct vector_t* pVector)
{
    // if both X and Y are less than FP Qmn resolution, assume we are at origin
    if (fabsf(pVector->fX) < QmnRESOLUTION && fabsf(pVector->fY) < QmnRESOLUTION )
        return 0;

    if ( pVector->fX > 0 )
        if ( pVector->fY > 0)
            return 4;           // x > 0 and y > 0
        else
            return 1;           // x > 0 and y < 0
    else
        if ( pVector->fY > 0 )
            return 3;           // x < 0 and y > 0
        else
            return 2;           // x < 0 and y < 0
}

/* -----------------------------------------
   ClearVector()

   clear values of a vector
----------------------------------------- */
void
ClearVector(struct vector_t* vector)
{

	vector->fAngle        = 0;
    vector->fLength       = 0;
    vector->fAngleSum     = 0;
    vector->fDx           = 0;
    vector->fDy           = 0;
    vector->fX            = 0;
    vector->fY            = 0;
    vector->fOriginAngle  = 0;
    vector->fOriginLength = 0;
    vector->fRealignAngle = 0;
}

/* -----------------------------------------
   ErrorExit()

   print error string to stderr and exit
----------------------------------------- */
void
ErrorExit(LPSTR lpszMessage)
{
	fprintf(stderr, "ERR: %s\n", lpszMessage);
	ExitProcess(1);
}
