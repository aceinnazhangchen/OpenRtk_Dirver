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

    /*
    * Message: INS_ACC from 'Vector__XXX* DLC: 6 byte(s), MID: 384
    */
    typedef struct {
    //Comment:comment//< B8:16  Min: -4 Max: 4   Unit: g   Destination: Vector__XXX
        float ACC_X;                             
    //Comment:comment//< B24:16  Min: -4 Max: 4   Unit: g   Destination: Vector__XXX
        float ACC_Y;                             
    //Comment:comment//< B40:16  Min: -4 Max: 4   Unit: g   Destination: Vector__XXX
        float ACC_Z;                             

        dbc_mia_info_t mia_info;
    } INS_ACC_t;

    /*
    * Message: INS_GYRO from 'Vector__XXX* DLC: 6 byte(s), MID: 385
    */
    typedef struct {
    //Comment:comment//< B8:16  Min: -250 Max: 250   Unit: deg/s   Destination: Vector__XXX
        float GYRO_X;                            
    //Comment:comment//< B24:16  Min: -250 Max: 250   Unit: deg/s   Destination: Vector__XXX
        float GYRO_Y;                            
    //Comment:comment//< B40:16  Min: -250 Max: 250   Unit: deg/s   Destination: Vector__XXX
        float GYRO_Z;                            

        dbc_mia_info_t mia_info;
    } INS_GYRO_t;

    /*
    * Message: INS_HeadingPitchRoll from 'Vector__XXX* DLC: 6 byte(s), MID: 386
    */
    typedef struct {
    //Comment:comment//< B8:16  Min: -360 Max: 360   Unit: deg   Destination: Vector__XXX
        float INS_PitchAngle;                    
    //Comment:comment//< B24:16  Min: -360 Max: 360   Unit: deg   Destination: Vector__XXX
        float INS_RollAngle;                     
    //Comment:comment//< B40:16  Min: -360 Max: 360   Unit: deg   Destination: Vector__XXX
        float INS_HeadingAngle;                  

        dbc_mia_info_t mia_info;
    } INS_HeadingPitchRoll_t;

    /*
    * Message: INS_HeightAndIMUStatus from 'Vector__XXX* DLC: 4 byte(s), MID: 387
    */
    typedef struct {
    //Comment:comment//< B24:32  Min: -10000 Max: 10000   Unit: m   Destination: Vector__XXX
        float INS_LocatHeight;                   
    //Comment:comment//< B56:32  Min: 0 Max: 4.29497e9   Unit:    Destination: Vector__XXX
        uint32_t IMU_Status;                     

        dbc_mia_info_t mia_info;
    } INS_HeightAndIMUStatus_t;

    /*
    * Message: INS_LatitudeLongitude from 'Vector__XXX* DLC: 4 byte(s), MID: 388
    */
    typedef struct {
    //Comment:comment//< B24:32  Min: -180 Max: 180   Unit: deg   Destination: Vector__XXX
        double INS_Latitude;                     
    //Comment:comment//< B56:32  Min: -180 Max: 180   Unit: deg   Destination: Vector__XXX
        double INS_Longitude;                    

        dbc_mia_info_t mia_info;
    } INS_LatitudeLongitude_t;

    /*
    * Message: INS_Speed from 'Vector__XXX* DLC: 6 byte(s), MID: 389
    */
    typedef struct {
    //Comment:comment//< B8:16  Min: -100 Max: 100   Unit: m/s   Destination: Vector__XXX
        float INS_NorthSpd;                      
    //Comment:comment//< B24:16  Min: -100 Max: 100   Unit: m/s   Destination: Vector__XXX
        float INS_EastSpd;                       
    //Comment:comment//< B40:16  Min: -100 Max: 100   Unit: m/s   Destination: Vector__XXX
        float INS_ToGroundSpd;                   

        dbc_mia_info_t mia_info;
    } INS_Speed_t;

    /*
    * Message: INS_DataInfo from 'Vector__XXX* DLC: 6 byte(s), MID: 390
    */
    typedef struct {
    //Comment:comment//< B0:8  Min: 0 Max: 255   Unit:    Destination: Vector__XXX
        uint8_t INS_GpsFlag_Pos;                 
    //Comment:comment//< B8:8  Min: 0 Max: 255   Unit:    Destination: Vector__XXX
        uint8_t INS_NumSV;                       
    //Comment:comment//< B16:8  Min: 0 Max: 255   Unit:    Destination: Vector__XXX
        uint8_t INS_GpsFlag_Heading;             
    //Comment:comment//< B24:8  Min: 0 Max: 255   Unit:    Destination: Vector__XXX
        uint8_t INS_Gps_Age;                     
    //Comment:comment//< B32:8  Min: 0 Max: 255   Unit:    Destination: Vector__XXX
        uint8_t INS_Car_Status;                  
    //Comment:comment//< B40:8  Min: 0 Max: 255   Unit:    Destination: Vector__XXX
        uint8_t INS_Status;                      

        dbc_mia_info_t mia_info;
    } INS_DataInfo_t;

    /*
    * Message: INS_Std from 'Vector__XXX* DLC: 8 byte(s), MID: 391
    */
    typedef struct {
    //Comment:comment//< B8:16  Min: 0 Max: 655.35   Unit:    Destination: Vector__XXX
        float INS_Std_Lat;                       
    //Comment:comment//< B24:16  Min: 0 Max: 655.35   Unit:    Destination: Vector__XXX
        float INS_Std_Lon;                       
    //Comment:comment//< B40:16  Min: 0 Max: 655.35   Unit:    Destination: Vector__XXX
        float INS_Std_LocatHeight;               
    //Comment:comment//< B56:16  Min: 0 Max: 655.35   Unit:    Destination: Vector__XXX
        float INS_Std_Heading;                   

        dbc_mia_info_t mia_info;
    } INS_Std_t;

    /*
    * Message: INS_Time from 'Vector__XXX* DLC: 4 byte(s), MID: 392
    */
    typedef struct {
    //Comment:comment//< B8:16  Min: 0 Max: 65535   Unit:    Destination: Vector__XXX
        uint16_t Week;                           
    //Comment:comment//< B24:32  Min: 0 Max: 4.29497e9   Unit: ms   Destination: Vector__XXX
        uint32_t TimeOfWeek;                     

        dbc_mia_info_t mia_info;
    } INS_Time_t;

    typedef struct{
        uint16_t week;
        double time_of_week;
        double acc_x;
        double acc_y;
        double acc_z;
        double gryo_x;
        double gryo_y;
        double gryo_z;
        uint32_t imu_status;
    }can_imu_t;

    typedef struct{
        uint16_t week;
        double time_of_week;
        uint8_t ins_car_status;
        uint8_t ins_position_status;
        double latitude;
        double longitude;
        double height;
        double north_vel;
        double east_vel;
        double up_vel;
        double roll;
        double pitch;
        double heading;
        double latitude_std;
        double longitude_std;
        double height_std;
    }can_ins_t;


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

    bool dbc_decode_INS_ACC(uint8_t *pstu, const uint8_t *bytes);    
    bool dbc_decode_INS_GYRO(uint8_t *pstu, const uint8_t *bytes);
    bool dbc_decode_INS_HeadingPitchRoll(uint8_t *pstu, const uint8_t *bytes);
    bool dbc_decode_INS_HeightAndIMUStatus(uint8_t *pstu, const uint8_t *bytes);
    bool dbc_decode_INS_LatitudeLongitude(uint8_t *pstu, const uint8_t *bytes);
    bool dbc_decode_INS_Speed(uint8_t *pstu, const uint8_t *bytes);
    bool dbc_decode_INS_DataInfo(uint8_t *pstu, const uint8_t *bytes);
    bool dbc_decode_INS_Std(uint8_t *pstu, const uint8_t *bytes);
    bool dbc_decode_INS_Time(uint8_t *pstu, const uint8_t *bytes);    
    void write_ins401c_kml_files();
}