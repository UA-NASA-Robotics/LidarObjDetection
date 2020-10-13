#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "uart_Handler.h"
#include "Definitions.h"
#include "GroundOBJprocessing.h"
#include "timers.h"
#include "MPU6050.h"

#define REAL double

//unsigned short object_Angles[MAX_OBJECTS * 2];
//unsigned short object_Mag[MAX_OBJECTS * 2];
#define MAX_ANGLE  120  //this is the max angle the code will use to find an object on the ground
#define MIN_ANGLE  60   //this is the min angle the code will use to find an object on the ground
#define THRESHOLD  35
unsigned short MaxMAGground_LookUpTable[180];


REAL ranges[MAX_ANGLE - MIN_ANGLE];
REAL xVal[MAX_ANGLE - MIN_ANGLE];

inline static REAL sqr(REAL x) {
    return x*x;
}

/** \brief (linreg): Least square fit algorithm that finds the slope and intercept of 
 *                   a line with a list of fitting points
 *
 * \param (n): number of data points
 * \param (x,y):  arrays of data
 * \param (b): output intercept
 * \param (m): output slope
 * \param (r): output correlation coefficient (can be NULL if you don't want it)
 * 
 * \return: 1-failed, 0-success
 */
int linreg(int n, const REAL x[], const REAL y[], REAL* m, REAL* b, REAL* r) {
    REAL sumx = 0.0; /* sum of x     */
    REAL sumx2 = 0.0; /* sum of x**2  */
    REAL sumxy = 0.0; /* sum of x * y */
    REAL sumy = 0.0; /* sum of y     */
    REAL sumy2 = 0.0; /* sum of y**2  */
    int i;
    for (i = 0; i < n; i++) {
        sumx += x[i];
        sumx2 += sqr(x[i]);
        sumxy += x[i] * y[i];
        sumy += y[i];
        sumy2 += sqr(y[i]);
    }

    REAL denom = (n * sumx2 - sqr(sumx));
    if (denom == 0) {
        // singular matrix. can't solve the problem.
        *m = 0;
        *b = 0;
        if (r) *r = 0;
        return 1;
    }

    *m = (n * sumxy - sumx * sumy) / denom;
    *b = (sumy * sumx2 - sumx * sumxy) / denom;
    if (r != NULL) {
        *r = (sumxy - sumx * sumy / n) / /* compute correlation coeff */
                sqrt((sumx2 - sqr(sumx) / n) *
                (sumy2 - sqr(sumy) / n));
    }

    return 0;
}
/** \brief (runGroundObjectDetection): Looks of object that are outside a linear plane 
 *
 * \param (_objLoc):  and empty array passed to the function for the function to be able to return
 *                    multiple locations of objects
 * \param (_maxObjCount):  This is the size of the array
 * 
 * \return: The number of objects that are found 
 */
REAL m, b, r;
int x;

int runGroundObjectDetection(point_t *_objLoc, int _maxObjCount) {

    int i;
    unsigned short objCount = 0;
    // convert polar values to Cartesian  
    for (i = MIN_ANGLE; i < MAX_ANGLE; i++) {
        ranges[i - MIN_ANGLE] = (double) getDistanceReading(i) * sin(i * DEGREE_TO_RAD);
        xVal[i - MIN_ANGLE] = (double) i;
        //printf("Range %d: %f \n", i, ranges[i - MIN_ANGLE]);

    }
    linreg(MAX_ANGLE - MIN_ANGLE, xVal, ranges, &m, &b, &r);
    float angle = getPitch();
    float distFromBase =(((double)90.0*m)+b)*cos (fabs(angle));
    //printf("res: %f\tang: %.2f\tdis: %f\n",(((double)90.0*m)+b), angle*RAD_TO_DEGREE, distFromBase);
    double maxVal = 0, maxAng = 0;
    int startAng = 0;
    int objCenter;
    for (x = MIN_ANGLE; x < MAX_ANGLE; x++) {
        if ((fabs(((((double) x) * m) + b)) - ranges[x - MIN_ANGLE]) > THRESHOLD) {
                      //printf("m: %.2f\t b: %.2f\t x: %d \t Range: %.2f \n", m, b, x, ranges[x - MIN_ANGLE]);
                       //printf("Res: %f\n",fabs(((((double)x)*m) + b))-ranges[x - MIN_ANGLE] );
            maxVal = fabs(fabs(((((double) x) * m) + b)) - ranges[x - MIN_ANGLE]);
            maxAng = x;
            /* Finding the edge of the object */
            if (startAng == 0) {
                startAng = (int) maxAng;
                //printf("start; %d\n", startAng);
            }
        } else {
            /* found the other side of the object! */
            if (startAng != 0) {
                objCenter = ((x - 1) + startAng) / 2;
                //printf("end: %d\ncenter: %d\n", x - 1, objCenter);
                //printf("end: %d \n", x - 1);
                startAng = 0;
                if( objCount <_maxObjCount ){
                    //printf("width: %d\n", Calculate_PointDistance((pointD) {(double)x,(((double)x*m) + b)}, (pointD){(double) startAng ,(((double)startAng * m) + b)}) );
                    _objLoc[objCount].y = distFromBase;
                    _objLoc[objCount].x = (((double)abs(objCenter-90)*m)+b)*sin((double)(objCenter-90)*DEGREE_TO_RAD);
                    //printf("xLoc: %f\n", xloc);
                    objCount++;
                }
            }
        }
    }
    return objCount;
}
 point_t obj[4];
int getObjectsCount()
{
    return runGroundObjectDetection(obj, sizeof(obj));
}
short getObject_1()
{
    getObjectsCount();
    return ((obj[0].x << 8) |(obj[0].y));
    
}
void Generate_GroundLookUpTable() {
    int _angle = 0;
    int quad = 0;
    MaxMAGground_LookUpTable[90] = AREA_LENGTH;
    for (quad = 0; quad < 2; quad++) {

        //calculating the max magnitude the is allowed inside the virtual box from angles 91 -180)
        MaxMAGground_LookUpTable[_angle] = MaxMAGground_LookUpTable[_angle - ((_angle - 90)*2)];

    }
    //Setting a max range1573 3000

    for (_angle = 0; _angle < 90; _angle++) {
        if (quad == 0) {
            //will only calculate the magnitudes for the end of the virtual box( max distances would only be calculated for the angle that will fall in between the intersection of the maxwidth and max length)
            if (_angle > (int) (atan(AREA_LENGTH / RIGHTSIDE_DISTANCE) * RAD_TO_DEGREE)) {
                MaxMAGground_LookUpTable[_angle] = (AREA_LENGTH / cos(((90 - _angle) * DEGREE_TO_RAD)));
            }
        } else {
            //just copying the value that have already been calculated over to the second quadrant 
            MaxMAGground_LookUpTable[180 - _angle] = MaxMAGground_LookUpTable[_angle];
        }
    }



}