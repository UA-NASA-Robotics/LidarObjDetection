/* 
 * File:   Definitions.h
 * Author: John
 *
 * Created on January 2, 2020, 2:33 PM
 */

#ifndef DEFINITIONS_H
#define	DEFINITIONS_H

#ifdef	__cplusplus
extern "C" {
#endif
#include "CAN_Handler/GlobalCAN_IDs.h"

#define MY_ADDRESS LIDAR

//*******************************************************************************************************
//---------------------LidarData Prossesing Constants/Booleans--------------------------
//*******************************************************************************************************

#define RUN_REALTIME_LIDAR_SPEED_VERIFICATION true
#define LIDAR_SPEED_UPPER_BOUND 300//220 //280
#define LIDAR_SPEED_LOWER_BOUND 290//180 //240
#define LIDAR_PULSE_WIDTH_INCREMENT_SIZE 20


#define LIDAR_MIN_INDEX_BYTE 0xA0   //these are the indexs byte in the 90, packets, going from 0XA0 to 0xF9 
#define LIDAR_MAX_INDEX_BYTE 0xF9
#define LIDAR_FIRST_180_INDEX_BYTE 0xCD
#define INVALID_DATA_FLAG_STATUS_MASK 0b10000000
#define CLEAN_UPPER_DISTANCE_BYTE_MASK 0b00111111
#define PACKET_INDEX_OFFSET  0xA0


#define LIDAR_MAX_VALUE 6000
//indexes of the array that will hold the right or left edge of the supposed object
#define OBJECT_RIGHT_EDGE 0
#define OBJECT_LEFT_EDGE 1
//this is the minimum width from the left-side to right-side of an object
#define MIN_OBJECT_WIDTH 300
#define ERROR_SECTOR_MAX_RANGE 6
#define ERROR_SECTOR_RAY_MAX_DELTA 300
//the maximum number of object that the code will try to keep track of in a sweep from 0 to 180 degrees
#define MAX_OBJECTS 1
//the difference in magnitudes that will determine a potential object
#define OBJECT_IDENTIFIER_RANGE 350
#define ENABLE_OBJ_GRACE_PERIOD false
//OBJ_END_GRACE_PERIOD: this const is the number of rays that are allowed to be greater then OBJECT_IDENTIFIER_RANGE untill the end of the OBJ is declared
#define OBJ_END_GRACE_PERIOD_VAL 4
//if the difference of magnitude is less then GRACE_P_SIGMA_MAG during the grace Period of object detection then we will continue looking for the end of the OBJ
#define GRACE_P_SIGMA_MAG 50
#define USE_BOUNDARY_LIMITS true
//Averaging the Object data operation values
#define AVERAGE_ITEMS_COUNT 20
#define RUNNING_AVG_OBJ_DATA true
#define REMOVE_DEVIATORS false //not implemented
#define OBJ_AVG_MAX_STD_DEVIATION 4
//Min_Heading_dist: this value is the minimum distance that is allowed when the
//heading is calculated. This eliminates headings that are erradic seen when an object doesn't move
//but a heading is calculated 
#define Min_Heading_dist 50
//====================Arena Dimensions===========================
#define COMPUT_AUTO_DEMENTIONS false
#define MAKE_SAME false
#define LEFTSIDE_DISTANCE   1000
#define RIGHTSIDE_DISTANCE  1000
#define AREA_LENGTH         2000

int LeftSideDistance;
int RightSideDistance;
//#define AREA_WIDTH  (LEFTSIDE_DISTANCE + RIGHTSIDE_DISTANCE)
//================================================================
//(AVERAGE_BAD_VALUES)This boolean when set to true will set a group of bad packets if the group is smaller than ERROR_SECTOR_MAX_RANGE to (the last good ray + the First good ray) / 2
#define AVERAGE_BAD_RAYS            true
#define SET_BAD_RAYS_TO_MAX         false
#define OBJ_DETECTION_USE_WIDTH     false
//if no object is found the constraints set then it will reset the object data to zero
#define NO_OBJ_DATA_RESET           true
//when set to true the data will be sent to debugger
#define SEND_DATA_TO_DEBUGGER       false
#define DEBUGGER_SEND_DISTANCES     false
#define DEBUGGER_SEND_QUALITY_DATA  false
//#define DEBUGGER_SEND_OBJECT_ANGLES false


//*************************************************************************************
//****************OBJ DIAGNOSTICS ALGORITHEMS************************
//*************************************************************************************
//PLEASE: Only set one of these values to true
#define USE_CARTESIAN_CENTER            false
#define USE_SLOPE_COMPUTATOR            false
#define REFLEC_TRUE_VAL                 700
//1400 is a good value for the reflective tape

//-------------------------------------------------------------------------------------
//________OBJ_SIDE_SLOPE_IDENTIFIER_ALGORITHEM___________
#define POINT_TO_POINT_SLOPE_TOLERANCE  0.8
#define SLOPE_DEVIATION_GRACE_PERIOD    4
#define NUM_OF_INITIAL_SLOPES           2
//this is the min number of degrees from the transitionIndex to the end of the OBJ
#define DEGREES_SLOPE_TERMINATION   1
#define SLOPE_OF_NINETY_DEG         50.0
#define NINETY_DEG_RETURN_VAL       90
 
//#define SIDE_SLOPE_TOLERNCE 0

//-------------------------------------------------------------------------------------
//_______parrallel Reflective Algorithem______________________
#define DELTA_REFLECTANCE           100
#define PARALLEL_SLOPE_TOLERENCE    0.5
#define IS_PARALLEL_BIT             0x8000;
//-------------------------------------------------------------------------------------
#define DEGREE_TO_RAD               0.01745329    //value of pi/180
#define RAD_TO_DEGREE               57.2957795    //value of 180/pi

typedef struct 
{
    double x;
    double y;
}pointD;

    //*******************************************************************************************************
    //------------------------------------DEBUGGING LEDs-------------------------------------------
    //*******************************************************************************************************
    /* These are defs for the LEDs on the beacon board (i am not sure why they need to be added here since they
    /* are in the app.h file(something to find out on another date))*/
#define LED1 LATEbits.LATE0
#define LED2 LATEbits.LATE1
#define LED3 LATEbits.LATE2
#define LED4 LATEbits.LATE3
#define LED5 LATEbits.LATE4
#define LED6 LATEbits.LATE5
#define LED7 LATEbits.LATE6
#define LED8 LATEbits.LATE7

#define On 0
#define Off 1

   
    unsigned short LidarSpeed;


#ifdef	__cplusplus
}
#endif

#endif	/* DEFINITIONS_H */

