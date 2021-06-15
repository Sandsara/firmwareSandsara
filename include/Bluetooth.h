#pragma once
//#define BLUECOMMENTS
#include <Arduino.h>
#include "config.h"


#define SERVICE_UUID1 "fd31a2be-22e7-11eb-adc1-0242ac120002" //for led config
#define SERVICE_UUID2 "fd31a58e-22e7-11eb-adc1-0242ac120002" //for 
#define SERVICE_UUID3 "fd31a688-22e7-11eb-adc1-0242ac120002"
#define SERVICE_UUID4 "fd31a778-22e7-11eb-adc1-0242ac120002" //for playlist config
#define SERVICE_UUID5 "fd31a840-22e7-11eb-adc1-0242ac120002" //for general config
#define SERVICE_UUID6 "fd31abc4-22e7-11eb-adc1-0242ac120002" //for file config


//====Led configuration Characteristics====
#define CHARACTERISTIC_UUID_LEDSPEED        "1a9a7b7e-2305-11eb-adc1-0242ac120002"
#define CHARACTERISTIC_UUID_CYCLEMODE       "1a9a7dea-2305-11eb-adc1-0242ac120002"
#define CHARACTERISTIC_UUID_DIRECTION       "1a9a8042-2305-11eb-adc1-0242ac120002"
#define CHARACTERISTIC_UUID_BRIGHTNESS      "1a9a8948-2305-11eb-adc1-0242ac120002"
#define CHARACTERISTIC_UUID_SELECTEDPALETTE "1a9a813c-2305-11eb-adc1-0242ac120002"
#define CHARACTERISTIC_UUID_AMOUNTCOLORS    "1a9a820e-2305-11eb-adc1-0242ac120002"
#define CHARACTERISTIC_UUID_POSITIONS       "1a9a82d6-2305-11eb-adc1-0242ac120002"
#define CHARACTERISTIC_UUID_RED             "1a9a83a8-2305-11eb-adc1-0242ac120002"
#define CHARACTERISTIC_UUID_GREEN           "1a9a8466-2305-11eb-adc1-0242ac120002"
#define CHARACTERISTIC_UUID_BLUE            "1a9a852e-2305-11eb-adc1-0242ac120002"
#define CHARACTERISTIC_UUID_UPDATECPALETTE  "1a9a87b8-2305-11eb-adc1-0242ac120002"
#define CHARACTERISTIC_UUID_MSGERRORLEDS    "1a9a8880-2305-11eb-adc1-0242ac120002"
/*
1a9a8a06-2305-11eb-adc1-0242ac120002
1a9a8ac4-2305-11eb-adc1-0242ac120002
1a9a8b8c-2305-11eb-adc1-0242ac120002*/

//====File config====
#define FILE_UUID_RECEIVEFLAG   "fcbff68e-2af1-11eb-adc1-0242ac120002" 
#define FILE_UUID_RECEIVE       "fcbffa44-2af1-11eb-adc1-0242ac120002"
#define FILE_UUID_EXISTS        "fcbffb52-2af1-11eb-adc1-0242ac120002"
#define FILE_UUID_DELETE        "fcbffc24-2af1-11eb-adc1-0242ac120002"
#define FILE_UUID_SENDFLAG      "fcbffdaa-2af1-11eb-adc1-0242ac120002"
#define FILE_UUID_SEND          "fcbffe72-2af1-11eb-adc1-0242ac120002"
#define FILE_UUID_ERRORMSG      "fcbffce2-2af1-11eb-adc1-0242ac120002"
/*
fcc0012e-2af1-11eb-adc1-0242ac120002
fcc0020a-2af1-11eb-adc1-0242ac120002
fcc002c8-2af1-11eb-adc1-0242ac120002
*/

//====Playlist Config====
#define PLAYLIST_UUID_NAME              "9b12a048-2c6e-11eb-adc1-0242ac120002"
#define PLAYLIST_UUID_PATHAMOUNT        "9b12a26e-2c6e-11eb-adc1-0242ac120002"
#define PLAYLIST_UUID_PATHNAME          "9b12a534-2c6e-11eb-adc1-0242ac120002"
#define PLAYLIST_UUID_PATHPOSITION      "9b12a62e-2c6e-11eb-adc1-0242ac120002"
#define PLAYLIST_UUID_ADDPATH           "9b12a7be-2c6e-11eb-adc1-0242ac120002"
#define PLAYLIST_UUID_MODE              "9b12a886-2c6e-11eb-adc1-0242ac120002"
#define PLAYLIST_UUID_PATHPROGRESS      "9b12a944-2c6e-11eb-adc1-0242ac120002"
#define PLAYLIST_UUID_ERRORMSG          "9b12aa02-2c6e-11eb-adc1-0242ac120002"
/*
9b12ac28-2c6e-11eb-adc1-0242ac120002
9b12acfa-2c6e-11eb-adc1-0242ac120002
9b12adb8-2c6e-11eb-adc1-0242ac120002
*/

//====General Config====
#define GENERAL_UUID_VERSION        "7b204278-30c3-11eb-adc1-0242ac120002"
#define GENERAL_UUID_NAME           "7b204548-30c3-11eb-adc1-0242ac120002"
#define GENERAL_UUID_STATUS         "7b204660-30c3-11eb-adc1-0242ac120002"
#define GENERAL_UUID_PAUSE          "7b20473c-30c3-11eb-adc1-0242ac120002"
#define GENERAL_UUID_PLAY           "7b20480e-30c3-11eb-adc1-0242ac120002"
#define GENERAL_UUID_SLEEP          "7b204a3e-30c3-11eb-adc1-0242ac120002"
#define GENERAL_UUID_SPEED          "7b204b10-30c3-11eb-adc1-0242ac120002"
#define GENERAL_UUID_RESTART        "7b204bce-30c3-11eb-adc1-0242ac120002"
#define GENERAL_UUID_FACTORYRESET   "7b204c8c-30c3-11eb-adc1-0242ac120002"
#define GENERAL_UUID_ERRORMSG       "7b204d4a-30c3-11eb-adc1-0242ac120002"
#define GENERAL_UUID_CALIBRATION    "7b204f84-30c3-11eb-adc1-0242ac120002"
/*
7b205056-30c3-11eb-adc1-0242ac120002
7b205114-30c3-11eb-adc1-0242ac120002
7b2051d2-30c3-11eb-adc1-0242ac120002
7b205290-30c3-11eb-adc1-0242ac120002
*/

/**
 * @class Bluetooth
 * @brief Se encarga de gestionar la comunicacion por bluetooth
 * @param timeOutBt es el tiempo, en milisegundos, que se va a esperar para recibir respuesta del dispositivo bluetooth conectado.
 * @param dataBt es donde se va a almacenar la informacion recibida.
 */
class Bluetooth {
    private:
        
    public:
        Bluetooth(); 
        int init(String = "Sandsara");

        static void setPlaylistName(String );
        static void setPathAmount(int);
        static void setPathName(String);
        static void setPathPosition(int);
        static void setPlayMode(int);
        static void setPathProgress(int);

        static void setLedSpeed(int);
        static void setCycleMode(int);
        static void setLedDirection(int);
        static void setBrightness(uint16_t);
        static void setIndexPalette(int);
        static void setRed();
        static void setGreen();
        static void setBlue();
        static void setPositions();
        static void setAmountOfColors();

        static void setVersion(String);
        static void setName(String);
        static void setStatus(int);
        static void setMotorSpeed(int);
        static void setPercentage(int);
        static void setCalibrationStatus(bool);
};