#pragma once

#ifndef HEADER_H
#define HEADER_H
/**
 * VERSION v1Current.v2Current.v3Current
 */
#define v1Current 1
#define v2Current 0
#define v3Current 0

extern char dataS[];

/**
 * Playlist priority
 * the next playlist names will be reproduce if are in the SD card
 */
#define TESTINGPLAYLIST "/SandsaraTesting.playlist"
#define DEFAULTPLAYLIST "/playlist.playlist"
/**
 * Status de sandsara
 * 1 calibrating
 * 2 running
 * 3 pause
 * 4 sleep
 * 5 busy
 */

//Es el valor de la resistencia colocada en los drivers, por ejemplo 0.11f representa 0.11 ohms.
#define R_SENSE             0.11f

//STALL_VALUE y STALL_VALUE2 es la sensibilidad para la deteccion de colision la cual tambien depende de la corriente.
#define STALL_VALUE         2 // [0..255]
#define STALL_VALUE2        2

//MICROSTEPPING es para configurar las distintas configuraciones de de microstepping que tiene el motor
// entre las opciones estan 1,2,4,8,16,32,64.
extern int MICROSTEPPING;

/** 
 * l1 y l2 representan la longitud, en milimetros, de los eslabones 1 y 2, respectivamente.
 * el eslabon 1 es el eslabon que se encuentra acoplado al eje central del mecanismo
 * el eslabon 2 es el eslabon que se encuentra acoplado la eslabon 1.
 */
extern int l1;
extern int l2;

/**
 * DISTANCIA_MAX es el radio maximo, desde el centro, que va a dibujar el robot
 * RESOLUCION_MAX es el numero que representa la DISTANCIA_MAX
 */
extern double MAX_RADIO;
extern double RESOLUCION_MAX;

/**
 * bigPulleySize y littlePulleySize se utilizan para saber la relacion de engranes
 * bigPulleySize es la polea acoplada a los motores.
 * littlePulleySize es la polea acoplada al eslabon 2.
 * la relacion se consigue con haciendo littlePulleySize / bigPulleySize por lo que se recomienda utilizar numeros flotantes
 * por ejemplo si bigPulleySize es de 40 dientes y littlePulleySize 20 dientes
 * se puede hacer bigPulleySize = 4.0 y littlePulleySize = 2.0
 */
extern double bigPulleySize;
extern double littlePulleySize;

/**
 * DIR_PIN, STEP_PIN, DIR_PIN2 y STEP_PIN2 se utilizan en la clase Motors para representar los pines de conexion de los motores
 * DIR_PIN es el pin de direccion del driver del motor 1
 * STEP_PIN es el pin step del driver para el motor 1
 * EN_PIN es el pin enable del driver del motor 1
 * DIR_PIN2 es el pin de direccion del driver del motor 2
 * STEP_PIN2 es el pin step del driver para el motor 2
 * EN_PIN2 es el pin enable del driver del motor 2
 */
#define DIR_PIN 33
#define STEP_PIN 25
#define EN_PIN 14
#define DIR_PIN2 4
#define STEP_PIN2 21
#define EN_PIN2 15

//no_picos representa el numero de picos de Stelle
#define no_picos 6

/**
 * si PROCESSING_SIMULATOR se define entonces se enviaran mensajes por el serial para conocer los pasos que dan los motores.
 * si DEBUGGING_DATA se define se mostrara mensajes por serial que sirven para debugear
 * si DEBUGGING_DETAIL se define se mostraran mensajes por serial que ayudan a debuguear el codigo
 * si DISABLE_MOTORS se define, los motores no se moveran a la hora de dibujar los archivos.
 */
// #define PROCESSING_SIMULATOR
#define DEBUGGING_DATA
//#define DEBUGGING_DETAIL
//#define DISABLE_MOTORS
//#define DEBUGGIN_LED2
#define IMPLEMENT_ACCELERATION
#define RESQUEST_ANSWER
#define DEBUGGING_BLUETOOTH

/**
 * encoder es el pin para la senal del encoder
 * hall es el pin para la senal del sensor de efecto hall
 */
#define hall1           39
#define hall2           36

/**
 * DIAG_PIN es el pin diag para detectar colisiones del motor 1
 * DIAG_PIN2 es el pin diag para detectar colisiones del motor 2
 */
#define DIAG_PIN            34 
#define DIAG_PIN2           22 

#define SERIAL_PORT         Serial1 // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS      0b00 // TMC2209 Driver address according to MS1 and MS2


#define SERIAL_PORT2        Serial2 // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS2     0b00 // TMC2209 Driver address according to MS1 and MS2

#define radius_1 152.0
#define radius_2 100.0

#define PIN_ProducType 35

/**
 * Direcciones de la ROM
 */
#define EEPROM_SIZE 512

#define ADDRESSPLAYLIST 0
#define ADDRESSPOSITION 41
#define ADDRESSORDERMODE 50
#define ADDRESSPALLETE 60
#define ADDRESSSPEEDMOTOR 70
#define ADDRESSPERIODLED 80
#define ADDRESSBTNAME 90
#define ADDRESSINTERMEDIATECALIBRATION 140
#define ADDRESSPOSITIONLIST 150                     //4 bytes
#define ADRESSLEDSDIRECTION 160                     //1 byte
#define ADDRESSCUSTOMPALLETE_COLORS 199
#define ADDRESSCUSTOMPALLETE_INCREMENTINDEX 198
#define ADDRESSCUSTOMPALLETE_POSITIONS 200
#define ADDRESSCUSTOMPALLETE_RED 216
#define ADDRESSCUSTOMPALLETE_GREEN 232
#define ADDRESSCUSTOMPALLETE_BLUE 248
#define ADDRESSPOLESENSE1 401
#define ADDRESSPOLESENSE2 402
#define ADDRESSESTOVERIFY 14
#define ADDRESSBRIGHTNESS 170   //1 byte


/**
 * Restricciones
 */
extern int MAX_CHARS_PLAYLIST;
extern int MAX_SPEED_MOTOR;
extern int MIN_SPEED_MOTOR;
extern int MAX_PERIOD_LED;
extern int MIN_PERIOD_LED;
extern int MAX_PALLETE;
extern int MIN_PALLETE;
extern int MAX_CHARACTERS_BTNAME;
extern int MIN_REPRODUCTION_MODE;
extern int MAX_REPRODUCTION_MODE;
extern int MAX_POSITIONLIST;
extern int LEDS_OF_HALO;
extern int LEDS_OF_STELLE;
extern int MAX_STEPS_PER_SECOND;
extern int MAX_CHARACTER_PER_LINE;
extern int OUTOFTIME_BLUETOOTH;
extern int OUTOFTIME_BLUETOOTH_LINE;
extern int NORMAL_CURRENT;
extern int CURRENT_IN_CALIBRATION;


//
#define ACCEL_THRESHOLD         500.0
#define SAFE_SPEED              1.0
#define MAX_STEPS_PER_INSTRUCTION 100.0
#define SAMPLES 50
#define CHUNKSIZE 512
#define MAX_LED_SUM 538
#define TIME_WAITFORSD 1000
#define FILENAME_BASE "Sandsara-trackNumber-"
#define SPEED_WHEN_IS_CONNECTED_TO_BLE 25
#define THREAD_LEDS_TIME 10 //in milliseconds
#define MAX_DISTANCE_FOR_MOVEMENT 2.5 //this distance is in milimeters and it represents the max distance the ball will move per instrucicion in MoveTo function
#define MAX_NUMBERLEDS 100
//Disable interrupts during updating leds
//#define FASTLED_ALLOW_INTERRUPTS 0

/**
 * Valores por defecto de Sandsara
 */
#define SPEED_MOTOR_DEFAULT 25
#define PLAYLIST_DEFAULT "/"
#define PALLETE_DEFAULT 1
#define PERIOD_LED_DEFAULT 10
#define ORDERMODE_DEFAULT 3
#define BLUETOOTHNAME "Sandsara"

/**
 * Valores para saber si ya se realizo la busqueda de la zona cero
 */
#define CEROZONE_PERFORMED 233
#define CEROZONE_NO_PERFORMED 177

/**
 * delay de leds y codigos de palletas
 */
#define DELAYCOLORCODE 10
#define CODE_NOSD_PALLETE 71
#define CODE_UPDATING_PALLETE 72
#define CODE_CALIBRATING_PALLETE 73
#define CODE_SDEMPTY_PALLETE 74

/**
 * El numero que avanzan los leds en la paleta de colores
 */
#define INCREMENTINDEXPALLETE 3

/**
 * Configuracion de la SD
 */
#define SPI_SPEED_TO_SD SD_SCK_MHZ(16)
#define SD_CS_PIN 5

/**
 * Buffer de tamano de nombre de un archivo en sd
 */
#define NAME_LENGTH 60

/**
 * Parametros de ida al centro en espiral
 * cada EVERY_MILIMITERS en modulo de recorrido dara una vuelta.
 */
extern int EVERY_MILIMITERS;
extern int SPEED_TO_CENTER;

/**
 * BUFFER_BLUETOOTH es la memoria que se reserva para recibir los archivos
 */
#define BUFFER_BLUETOOTH 30000

/**
 * MAX_COLORSPERPALLETE representa el numero maximo de colores que hay por paleta
 */
#define MAX_COLORSPERPALLETE 16

/**
 * 
 */
#define MODE_CALIBRATION    1
#define MODE_PLAY           2
#define MODE_PAUSE          3
#define MODE_SLEEP          4
#define MODE_BUSY           5

//App Variables
#define MIN_SLIDER_MSPEED       1
#define MAX_SLIDER_MSPEED       10
#define MIN_SLIDER_BRIGHTNESS   0
#define MAX_SLIDER_BRIGHTNESS   100
#define MIN_SLIDER_LEDSPEED     1
#define MAX_SLIDER_LEDSPEED     100

#endif