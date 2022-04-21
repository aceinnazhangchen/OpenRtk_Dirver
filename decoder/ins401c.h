#pragma once
#include <stdint.h>

namespace ins401c_Tool {
    /* Missing in Action structure */
    typedef struct {
        uint32_t is_mia : 1;          ///< Missing in action flag
        uint32_t mia_counter_ms : 31; ///< Missing in action counter
    } dbc_mia_info_t;
    /* CAN message header structure */

    /*
    * Message: INSPVAX from 'Vector__XXX* DLC: 53 byte(s), MID: 1280
    */
    typedef struct {
    //Comment:comment//< B8:16  Min: -4 Max: 4   Unit: g   Destination: Vector__XXX
        float ACC_X;                             
    //Comment:comment//< B24:16  Min: -4 Max: 4   Unit: g   Destination: Vector__XXX
        float ACC_Y;                             
    //Comment:comment//< B40:16  Min: -4 Max: 4   Unit: g   Destination: Vector__XXX
        float ACC_Z;                             
    //Comment:comment//< B56:16  Min: -250 Max: 250   Unit: deg/s   Destination: Vector__XXX
        float GYRO_X;                            
    //Comment:comment//< B72:16  Min: -250 Max: 250   Unit: deg/s   Destination: Vector__XXX
        float GYRO_Y;                            
    //Comment:comment//< B88:16  Min: -250 Max: 250   Unit: deg/s   Destination: Vector__XXX
        float GYRO_Z;                            
    //Comment:comment//< B104:16  Min: -360 Max: 360   Unit: deg   Destination: Vector__XXX
        float INS_PitchAngle;                    
    //Comment:comment//< B120:16  Min: -360 Max: 360   Unit: deg   Destination: Vector__XXX
        float INS_RollAngle;                     
    //Comment:comment//< B136:16  Min: -360 Max: 360   Unit: deg   Destination: Vector__XXX
        float INS_LocatHeight;          
    //Comment:comment//< B168:32  Min: -10000 Max: 10000   Unit: m   Destination: Vector__XXX
        uint32_t IMU_Status;

        float INS_HeadingAngle;                  

    //Comment:comment//< B200:32  Min: 0 Max: 4.29497e9   Unit: ms   Destination:       
    //Comment:comment//< B232:32  Min: -180 Max: 180   Unit: deg   Destination: 
        double INS_Latitude;                     
    //Comment:comment//< B264:32  Min: -180 Max: 180   Unit: deg   Destination: Vector__XXX
        double INS_Longitude;                    
    //Comment:comment//< B280:16  Min: -100 Max: 100   Unit: m/s   Destination: Vector__XXX
        float INS_NorthSpd;                      
    //Comment:comment//< B296:16  Min: -100 Max: 100   Unit: m/s   Destination: Vector__XXX
        float INS_EastSpd;                       
    //Comment:comment//< B312:16  Min: -100 Max: 100   Unit: m/s   Destination: Vector__XXX
        float INS_ToGroundSpd;                   
    //Comment:comment//< B320:8  Min: 0 Max: 255   Unit:    Destination: Vector__XXX
        uint8_t INS_GpsFlag_Pos;                 
    //Comment:comment//< B328:8  Min: 0 Max: 255   Unit:    Destination: Vector__XXX
        uint8_t INS_NumSV;                       
    //Comment:comment//< B336:8  Min: 0 Max: 255   Unit:    Destination: Vector__XXX
        uint8_t INS_GpsFlag_Heading;             
    //Comment:comment//< B344:8  Min: 0 Max: 255   Unit:    Destination: Vector__XXX
        uint8_t INS_Gps_Age;                     
    //Comment:comment//< B352:8  Min: 0 Max: 255   Unit:    Destination: Vector__XXX
        uint8_t INS_Car_Status;                  
    //Comment:comment//< B360:8  Min: 0 Max: 255   Unit:    Destination: Vector__XXX
        uint8_t INS_Status;                      
    //Comment:comment//< B376:16  Min: 0 Max: 655.35   Unit:    Destination: Vector__XXX
        float INS_Std_Lat;                       
    //Comment:comment//< B392:16  Min: 0 Max: 655.35   Unit:    Destination: Vector__XXX
        float INS_Std_Lon;                       
    //Comment:comment//< B408:16  Min: 0 Max: 655.35   Unit:    Destination: Vector__XXX
        float INS_Std_LocatHeight;               
    //Comment:comment//< B424:16  Min: 0 Max: 655.35   Unit:    Destination: Vector__XXX
        float INS_Std_Heading;                 

        uint16_t Week;                   
        uint32_t TimeOfWeek;        
        dbc_mia_info_t mia_info;
    } INSPVAX_t;
    #define MAX_CANFD_MESSAGE_COUNT  1
    typedef struct { 
        uint32_t mid;  //< Message ID of the message
        uint8_t  dlc;  //< Data length of the message
        bool (*func)(uint8_t *pstu, const uint8_t *bytes); //< Data length of the message
    } dbc_msg_hdr_t; 

    /* Extern function needed for dbc_msg_decode() */
    bool dbc_msg_decode(uint32_t mid, uint8_t *from, uint8_t *to);

    /* CAN message decode extern function */
    bool dbc_decode_INSPVAX(uint8_t *pstu, const uint8_t *bytes);

    void set_base_ins401c_file_name(char* file_name);
    int input_ins401c_line(uint8_t* data);
    void write_ins401c_log_file(char* log);
    void write_ins401c_imu_file(char* log);
    void write_ins401c_ins_file(char* log);

}