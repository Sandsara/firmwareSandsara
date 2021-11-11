#include <Arduino.h>
#include "SdFiles.h"
#include "Motors.h"
#include "Bluetooth.h"
#include <Adafruit_NeoPixel.h>
#define FASTLED_ESP32_I2S true
#include <FastLED.h>
#include "WorkingArea.h"
#include "Testing.h"
#include "esp_int_wdt.h"
#include "esp_task_wdt.h"

#include "SPI.h"
#include "SdFat.h"
SdFat SD;
bool readingSDFile = false;
#include <math.h>

#include <EEPROM.h>

char dataS[] = {
    0x00, 0x31, 0x36, 0x2c, 0x37, 0x36, 0x2c, 0x37, 0x36, 0x2c, 0x31, 0x33, 0x34,
    0x2c, 0x33, 0x32, 0x37, 0x36, 0x38, 0x2c, 0x34, 0x2c, 0x32, 0x2c, 0x32,
    0x30, 0x2c, 0x31, 0x35, 0x30, 0x2c, 0x34, 0x30, 0x2c, 0x31, 0x32, 0x30,
    0x2c, 0x31, 0x2c, 0x35, 0x30, 0x30, 0x2c, 0x31, 0x30, 0x2c, 0x31, 0x36,
    0x2c, 0x30, 0x2c, 0x33, 0x30, 0x2c, 0x31, 0x2c, 0x32, 0x2c, 0x31, 0x30,
    0x30, 0x30, 0x2c, 0x31, 0x30, 0x30, 0x2c, 0x36, 0x30, 0x2c, 0x37, 0x32,
    0x2c, 0x31, 0x35, 0x30, 0x2c, 0x31, 0x30, 0x30, 0x30, 0x2c, 0x32, 0x30,
    0x30, 0x30, 0x30, 0x2c, 0x31, 0x30, 0x30, 0x30, 0x2c, 0x36, 0x30, 0x30,
    0x2c, 0x36, 0x30, 0x30, 0x0a
};


//====Extern Variables====
extern TMC2209Stepper tmcMotorA;
extern TMC2209Stepper tmcMotorB;
extern bool sdExists(String );
extern bool sdRemove(String );
extern bool incrementGlobal;
extern double times[];
extern int pointerGlobal;
//====

int EVERY_MILIMITERS = String(String(dataS[24]) + String(dataS[25])).toFloat();
int SPEED_TO_CENTER = String(String(dataS[27]) + String(dataS[28]) + String(dataS[29])).toFloat();

int MAX_CHARS_PLAYLIST = String(String(dataS[31]) + String(dataS[32])).toFloat();

int LEDS_OF_HALO = String(String(dataS[68]) + String(dataS[69])).toFloat();
int LEDS_OF_STELLE = String(String(dataS[71]) + String(dataS[72])).toFloat();

int NORMAL_CURRENT = String(String(dataS[94]) + String(dataS[95]) + String(dataS[96])).toFloat();
int CURRENT_IN_CALIBRATION = String(String(dataS[98]) + String(dataS[99]) + String(dataS[100])).toFloat();

//====ROM Varibales====
String  playListGlobal;
String  bluetoothNameGlobal;
int     orderModeGlobal;
int     speedMotorGlobal;
int     ledModeGlobal;
int     periodLedsGlobal;
bool    ceroZoneGlobal;
//====Global variables====
bool    ledsOffGlobal = false;
bool    rewindPlaylist = false;
bool    incrementIndexGlobal = true;
String  currentProgramGlobal;
String  nextProgramGlobal;
String  currentPlaylistGlobal;
int     currentPositionListGlobal;
int     delayLeds;
int     pListFileGlobal;
bool    changePositionList;
bool    changeProgram;
bool    stopProgramChangeGlobal = true;
bool    stop_boton;
bool    intermediateCalibration = false;
bool    firstExecution;
bool    availableDeceleration = false;
bool    turnOnLeds = false;
bool    ledsDirection;
bool    pauseModeGlobal = false;
bool    suspensionModeGlobal = false;
bool    startMovement = false;
long    q1StepsGlobal, q2StepsGlobal;
double  distanceGlobal;
bool    speedChangedMain = false;
String  changedProgram;
int     changedPosition;
bool    lastPoint = false;
int     tracksPlayed = 0;
//====
//====Pallete Color Variables====
CRGBPalette16   NO_SD_PALLETE;
CRGBPalette16   UPTADATING_PALLETE;
CRGBPalette16   CALIBRATING_PALLETE;
CRGBPalette16   SDEMPTY_PALLETE;
CRGBPalette256  customPallete;
CRGBPalette256  pallette1, pallette2,pallette3,
pallette4,pallette5,pallette6,
pallette7,pallette8,pallette9,
pallette10,pallette11,pallette12,
pallette13,pallette14,pallette15;
//====
Motors Sandsara;
Bluetooth BluetoothSand;
int errorCode;
//====function prototypes====
extern  int programming(String );
extern  void rebootWithMessage(String );
extern int stringToArray(String , uint8_t* , int );

int     moveInterpolateTo(double x, double y, double distance);
void    executeCode(int );
void    Neo_Pixel(int );
uint32_t rainbow();
void    FillLEDsFromPaletteColors(uint8_t );
void    changePalette(int );
void    SetupTotallyRandomPalette();
void    SetupBlackAndWhiteStripedPalette();
void    SetupPurpleAndGreenPalette();
void    ledsFunc( void * );
int     run_sandsara(String ,int );
int     movePolarTo(double ,double ,double, bool = false);

int     romSetPlaylist(String );
String  romGetPlaylist();
int     romSetOrderMode(int );
int     romGetOrderMode();
int     romSetPallete(int );
int     romGetPallete();
int     romSetSpeedMotor(int );
int     romGetSpeedMotor();
int     romSetPeriodLed(int );
int     romGetPeriodLed();
int     romSetCustomPallete(uint8_t* ,uint8_t* , uint8_t* ,uint8_t*, int);
int     romGetCustomPallete(CRGBPalette256 &);
int     romSetBluetoothName(String );
String  romGetBluetoothName();
int     romSetIntermediateCalibration(bool );
bool    romGetIntermediateCalibration();
int     romSetPositionList(int );
int     romGetPositionList();
int     romSetIncrementIndexPallete(bool );
bool    romGetIncrementIndexPallete();
int     romSetLedsDirection(bool );
bool    romGetLedsDirection();
int     romSetBrightness(uint8_t );
int     romGetBrightness();

void    findUpdate();
int     orderRandom(String ,int);
void    setFrom1(int [], int);
void    removeIndex(int [], int , int );
int     runFile(String );
void    goHomeSpiral();
void    bluetoothThread(void* );
double  linspace(double init,double stop, int amount,int index);
int     rgb2Interpolation(CRGBPalette256 &,uint8_t* );
void    goCenterSpiral(bool);
void    goEdgeSpiral(bool);
void    spiralGoTo(float , float );

void moveSteps(void* );
void goPolar(double module, double angle);

//====
//====Led Variables====
#define LED_PIN     32
    int NUM_LEDS;
#define BRIGHTNESS  255
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
CRGB leds[MAX_NUMBERLEDS];
//====
#include <leds.h>

unsigned long timeLeds;
uint8_t startIndex = 0;
//====Product Type====
bool productType;
//====
//====Thred====
TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t motorsTask;
//====
//====WorkingArea====
WorkingArea haloCalib;
//====Testing========
Testing haloTest;
//====
void setup()
{

    delay(1000); // power-up safety delay
    //====Serial configuration====
    Serial.begin(115200);

    //====EEPROM Initialization====
    EEPROM.begin(EEPROM_SIZE);
    delay(500);
    //====Init watchdog====
    esp_task_wdt_init(15,false);
    //====Testing====
	int dat_pin;
	dat_pin = analogRead(PIN_ProducType);
	if(dat_pin > 682 && dat_pin < 2047)
	{
		haloTest.Test();	
	}
    if(dat_pin > 2047 && dat_pin < 3413)
	{
		for(int x = 0; x < 512; x++)
        {
            EEPROM.write(x,-1);
            EEPROM.commit();
            delay(20);
        }
        Serial.println("EEprom");	
	}
    
    //====SD initialization====
    long timeSdCard = millis();
    bool sdDetected = SD.begin(SD_CS_PIN, SPI_SPEED_TO_SD);
    while(!sdDetected)
    {
        //changePalette(CODE_NOSD_PALLETE);
        #ifdef DEBUGGING_DATA
            Serial.println("Card failed, or not present");
        #endif
        if (millis() - timeSdCard > TIME_WAITFORSD){
            #ifdef DEBUGGING_DATA
                Serial.println("TIME_WAITFORSD is exceeded");
            #endif
            break;
        }
        delay(200);
        sdDetected = SD.begin(SD_CS_PIN, SPI_SPEED_TO_SD);
    }
    //====
    if(sdDetected){
        #ifdef DEBUGGING_DATA
            Serial.println("Card present");
        #endif
        findUpdate();
    }
    
    //====Palletes initialization====
    NO_SD_PALLETE= breathRed;
    UPTADATING_PALLETE = breathYellow;
    CALIBRATING_PALLETE = solidWhite;
    SDEMPTY_PALLETE = breathOrange;
    rgb2Interpolation(pallette1, palletteColors1);
    rgb2Interpolation(pallette2, palletteColors2);
    rgb2Interpolation(pallette3, palletteColors3);
    rgb2Interpolation(pallette4, palletteColors4);
    rgb2Interpolation(pallette5, palletteColors5);
    rgb2Interpolation(pallette6, palletteColors6);
    rgb2Interpolation(pallette7, palletteColors7);
    rgb2Interpolation(pallette8, palletteColors8);
    rgb2Interpolation(pallette9, palletteColors9);
    rgb2Interpolation(pallette10, palletteColors10);
    rgb2Interpolation(pallette11, palletteColors11);
    rgb2Interpolation(pallette12, palletteColors12);
    rgb2Interpolation(pallette13, palletteColors13);
    rgb2Interpolation(pallette14, palletteColors14);
    rgb2Interpolation(pallette15, palletteColors15);
    
    //====Restoring bluetooth name====
    bluetoothNameGlobal = "Sandsara " + romGetBluetoothName();
    //====Restoring of refreshing time of leds====
    periodLedsGlobal = romGetPeriodLed();
    if (periodLedsGlobal > MAX_PERIOD_LED || periodLedsGlobal < MIN_PERIOD_LED){
        periodLedsGlobal = PERIOD_LED_DEFAULT;
        romSetPeriodLed(PERIOD_LED_DEFAULT);
    }
    //====Configure the halo and bluetooth and files====
    BluetoothSand.init(bluetoothNameGlobal);
    Sandsara.init();
    SdFiles::DISTANCIA_MAX = MAX_RADIO;
    BluetoothSand.setVersion(String(v1Current) + "." + String(v2Current) + "." + String(v3Current));
    Serial.println("antes de setname");
    Serial.println(bluetoothNameGlobal);
    BluetoothSand.setName(bluetoothNameGlobal);
    BluetoothSand.setStatus(MODE_CALIBRATION);
    //====Select type of product====
    if (analogRead(PIN_ProducType) < 1000){
        productType = false;
        NUM_LEDS = LEDS_OF_HALO;
#ifdef DEBUGGING_DATA
        Serial.println("Product: Halo");
#endif
    }
    else{
        productType = true;
        NUM_LEDS = LEDS_OF_STELLE;
#ifdef DEBUGGING_DATA
        Serial.println("Product: Stelle");
#endif
    }
    delay(1000);
    //====Restoring of the latest pallete choosed====
    ledModeGlobal = romGetPallete();
    if (ledModeGlobal > MAX_PALLETE || ledModeGlobal < MIN_PALLETE){
        ledModeGlobal = PALLETE_DEFAULT;
        romSetPallete(PALLETE_DEFAULT);
    }
    changePalette(CODE_CALIBRATING_PALLETE);
    BluetoothSand.setBrightness(romGetBrightness());
    //====new task for leds====
    xTaskCreatePinnedToCore(
                    ledsFunc,
                    "Task1",
                    5000,
                    NULL,
                    5,
                    &Task1,
                    1);
    delay(500); 
    
    //====Cablibrating====
    haloCalib.prepareMotors();
    #ifdef DEBUGGING_DATA
        Serial.println("Calibrando...");
    #endif
    haloCalib.findZeroPoint();
    #ifdef DEBUGGING_DATA
        Serial.println("calibrado");
    #endif
    //====After calibration current has to be set up====
    tmcMotorA.rms_current(NORMAL_CURRENT);
    tmcMotorB.rms_current(NORMAL_CURRENT);
    pinMode(EN_PIN, OUTPUT);
    pinMode(EN_PIN2, OUTPUT);
    digitalWrite(EN_PIN, LOW);
    digitalWrite(EN_PIN2, LOW);

    BluetoothSand.setStatus(MODE_BUSY); //set status
    changePalette(CODE_NOSD_PALLETE); 
	while(!SD.begin(SD_CS_PIN, SPI_SPEED_TO_SD))
    {
        changePalette(CODE_NOSD_PALLETE);
        #ifdef DEBUGGING_DATA
            Serial.println("Card failed, or not present");
        #endif
        delay(200); 
    }
    changePalette(ledModeGlobal);
    #ifdef DEBUGGING_DATA
        Serial.println("Card present");
    #endif

    //====restore the value intermediateCalibration====
    intermediateCalibration = romGetIntermediateCalibration();
    BluetoothSand.setCalibrationStatus(intermediateCalibration);
    //====Restore speedMotor====
    speedMotorGlobal = romGetSpeedMotor();
    if (speedMotorGlobal > MAX_SPEED_MOTOR || speedMotorGlobal < MIN_SPEED_MOTOR){
        speedMotorGlobal = SPEED_MOTOR_DEFAULT;
        romSetSpeedMotor(SPEED_MOTOR_DEFAULT);
    }
    Sandsara.setSpeed(speedMotorGlobal);
    //====Restore playlist name and orderMode====
    playListGlobal = romGetPlaylist();
    orderModeGlobal = romGetOrderMode();
    if (orderModeGlobal < MIN_REPRODUCTION_MODE || orderModeGlobal > MAX_REPRODUCTION_MODE){
        orderModeGlobal = 1;
        romSetOrderMode(1);
    }
    changePalette(romGetPallete());
    if (playListGlobal.equals("/")){
        #ifdef DEBUGGING_DATA
            Serial.print("No hay una playlist guardada, se reproduciran todos los archivos en la sd");
        #endif
    }
    else
    {
        if (sdExists(playListGlobal)){
            delay(1);
            #ifdef DEBUGGING_DATA
                Serial.print("lista guardada: ");
                Serial.println(playListGlobal);
            #endif
        }
        else{
            #ifdef DEBUGGING_DATA
                Serial.println("La playlist no existe, se reproduciran todos los archivos en la sd");
            #endif
        }
    }
    #ifdef DEBUGGING_DATA
        Serial.print("orderMode guardado: ");
        Serial.println(orderModeGlobal);
    #endif
    #ifdef PROCESSING_SIMULATOR
        Serial.println("inicia");
    #endif
    //=====if the file playlist.playlist exits it will be executed====
    if (sdExists(DEFAULTPLAYLIST)){
        playListGlobal = DEFAULTPLAYLIST;
        orderModeGlobal = 1;
    }
    if (sdExists(TESTINGPLAYLIST)){
        playListGlobal = TESTINGPLAYLIST;
        orderModeGlobal = 1;
        romSetPositionList(1);
        SdFiles::DISTANCIA_MAX = l1 + l2;
    }
    //====Restore leds direction====
    ledsDirection = romGetLedsDirection();
    //====Restore cycleMode====
    bool cycleMode = romGetIncrementIndexPallete();
    //====bluetooth configuration====
    
    BluetoothSand.setPlaylistName(playListGlobal);
    BluetoothSand.setPathAmount(0);
    BluetoothSand.setPathName("");
    BluetoothSand.setPlayMode(orderModeGlobal);
    BluetoothSand.setPercentage(0);

    BluetoothSand.setLedSpeed(periodLedsGlobal);
    if(cycleMode){
        BluetoothSand.setCycleMode(1);
    }else {
        BluetoothSand.setCycleMode(0);
    }
    if (ledsDirection){
        BluetoothSand.setLedDirection(1);
    }else{
        BluetoothSand.setLedDirection(0);
    }
    BluetoothSand.setIndexPalette(ledModeGlobal);

    BluetoothSand.setMotorSpeed(speedMotorGlobal);

    xTaskCreatePinnedToCore(
                    moveSteps,   
                    "motorsTasks",     
                    5000,
                    NULL,
                    4,
                    &motorsTask,
                    0);
    delay(500); 

    if (!playListGlobal.equals(TESTINGPLAYLIST)){
        Serial.println("iniciara espiral");
        goEdgeSpiral(false);
        Sandsara.setSpeed(SPEED_TO_CENTER);
        delay(1000);
        goPolar(SdFiles::DISTANCIA_MAX, 2*PI);
        Sandsara.setSpeed(romGetSpeedMotor());
        Serial.println("termino espiral");
    }
    delay(1000);
    firstExecution = true;
}

void loop()
{
    #ifdef DEBUGGING_DATA
        Serial.println("Iniciara la funcion runSansara");
    #endif
    errorCode = run_sandsara(playListGlobal, orderModeGlobal);
    #ifdef DEBUGGING_DATA
        Serial.print("errorCode de run: ");
        Serial.println(errorCode);
    #endif
    if (errorCode == -10){
        Sandsara.completePath();
        while(!SD.begin(SD_CS_PIN, SPI_SPEED_TO_SD))
        {
            changePalette(CODE_NOSD_PALLETE);
            #ifdef DEBUGGING_DATA
                Serial.println("Card failed, or not present");
            #endif
            delay(200); 
        }
        changePalette(ledModeGlobal);
        #ifdef DEBUGGING_DATA
            Serial.println("Card present");
        #endif
        

    }
    else if (errorCode == -4){
        changePalette(CODE_SDEMPTY_PALLETE);
        SD.begin(SD_CS_PIN, SPI_SPEED_TO_SD);
    }
    firstExecution = false;
    delay(1000);
    
}

/**
 * @brief
 * @param playlist the name of playlist to be executed, e.g. "/animales.playlist"
 * @param orderMode the orden that files will be executed
 * 1, files will be reproduced in descending order according to playlist.
 * 2, All files in SD will be reproduced in a determined order.
 * @return an error code that could be one below.
 *  1, playlist or orderMode were changed.
 *  0, ends without error.
 * -1, playlist was not found.
 * -2, playlist is a directory.
 * -3, an attempt to choose an incorrect position in playlist was performed.
 * -4, number of files in playlist or SD are zero.
 * -10, SD was not found.
 */
int run_sandsara(String playList, int orderMode)
{
    BluetoothSand.setStatus(MODE_BUSY);
    #ifdef DEBUGGING_DATA
        Serial.print("playlist a reproducir: ");
        Serial.println(playList);
    #endif
    if (firstExecution){
        pListFileGlobal = romGetPositionList();
    }
    else{
        pListFileGlobal = 1;
    }
    int numberOfFiles;
    String fileName;
    
    if (orderMode == 1){
        File file;
        file = SD.open(playList);
        if (file && !file.isDirectory()){
            file.close();
            numberOfFiles = SdFiles::numberOfLines(playList);
            if (numberOfFiles < 0){
                return -4;
            }
        }
        else
        {
            file.close();
            orderMode = 2;
            numberOfFiles = SdFiles::creatListOfFiles("/DEFAULT.playlist");
            if (numberOfFiles < 0){
                return -4;
            }
            playList = "/DEFAULT.playlist";
        }
    }
    else
    {
        numberOfFiles = SdFiles::creatListOfFiles("/DEFAULT.playlist");
        if (numberOfFiles < 0){
            return -4;
        }
        playList = "/DEFAULT.playlist";
    }
    #ifdef DEBUGGING_DATA
        Serial.print("Numero de archivos: ");
        Serial.println(numberOfFiles);
    #endif

    if (numberOfFiles == 0){
        return -4;
    }
    BluetoothSand.setPlaylistName(playList);
    BluetoothSand.setPathAmount(numberOfFiles);
    changePalette(romGetPallete());
    //====restore playlist====
    currentPlaylistGlobal = playList;
    //====
    while (true)
    {
        BluetoothSand.setStatus(MODE_PLAY);
        //====Save the current position in playlist====
        romSetPositionList(pListFileGlobal);
        //====
        #ifdef DEBUGGING_DATA
            Serial.println("Abrira el siguiente archivo disponible");
        #endif
        errorCode = SdFiles::getLineNumber(pListFileGlobal, playList, fileName);
        if (errorCode < 0)
        {
            //====playList is not valid====
            delay(1000);
            return errorCode;
        }
        SdFiles::getLineNumber(pListFileGlobal + 1, playList, nextProgramGlobal);
        if (errorCode < 0)
        {
            nextProgramGlobal = "none";
        }
        if (errorCode == 2 || errorCode == 3)
        {
            delay(1000);
            break;
        }
        while (readingSDFile){
            delay(1);
        }
        readingSDFile = true;
        File current_file = SD.open("/" + fileName);
        if (!current_file)
        {
            delay(1000);
            pListFileGlobal += 1;
            readingSDFile = false;
            continue;
        }
        if (current_file.isDirectory())
        {
            delay(1000);
            pListFileGlobal += 1;
            readingSDFile = false;
            continue;
        }
        char nameF[NAME_LENGTH];
        current_file.getName(nameF,NAME_LENGTH);
        fileName = nameF;
        current_file.close();
        readingSDFile = false;
        //====run program====
        errorCode = runFile(fileName);
#ifdef DEBUGGING_DATA
        Serial.print("error code de runFile: ");
        Serial.println(errorCode);
#endif
        //====
        if (errorCode == -70)   {continue;}
        else if (errorCode == -71)   {break;}
        else if (errorCode == 20)    {pListFileGlobal = changedPosition; continue;}
        else if (errorCode == 30)    {
            while(errorCode == 30){
                fileName = changedProgram;
                errorCode = runFile(fileName);
            }
            if (errorCode == 20)    {pListFileGlobal = changedPosition; continue;}
            pListFileGlobal += 1;
            continue;
        }
        else if (errorCode == 40){
            while (suspensionModeGlobal)
            {
                ledsOffGlobal = true;
                delay(500);
            }
            ledsOffGlobal = false;
            continue;
        }
        else if (errorCode != 10)    {return errorCode;}
        //====Increment pListFileGlobal====
        pListFileGlobal += 1;
        //====
    }
    return 0;
}
/**
 * @brief execute a path
 * @param dirFile, path direction in SD.
 * @return an errorCode
 * -70, instruction continue has to be performed.
 * -71, instruction break has to be performed.
 *   1, orderMode or playlist have been changed.
 *  10, the function finished without errors.
 *  20, an instruction for changing path by postion was recieved.
 *  30, an instruction for changing path by name was recieved.
 *  40, an instruction for suspention was recieved.
 */
int runFile(String fileName){
#ifdef DEBUGGING_DATA
    Serial.print("runFile: ");
    Serial.println(fileName);
#endif
    double component_1, component_2, distance;
    int working_status = 0;
    //====restore the path name and its position in the playlist.
    currentProgramGlobal = fileName;
    currentPositionListGlobal = pListFileGlobal;
    //====
    double couplingAngle, startFileAngle, endFileAngle;
    int posisionCase;
    SdFiles file(fileName);
    if (file.fileType < 0){
        pListFileGlobal += 1;
        return -70;
    }
    BluetoothSand.setPathName(fileName);
    BluetoothSand.setPathPosition(pListFileGlobal);
    double zInit = Sandsara.getCurrentModule();
    //====read of File mode is selected.
    file.autoSetMode(zInit);
    //====
    if (!file.isValid()){
        pListFileGlobal += 1;
#ifdef DEBUGGING_DATA
        Serial.println("Archivo no valido");
#endif
        return -70;
    }
    #ifdef PROCESSING_SIMULATOR
        Serial.print("fileName: ");
        Serial.println(fileName);
    #endif
    /*Serial.print("fileName: ");
    Serial.println(fileName);*/
    startFileAngle = file.getStartAngle();
    startFileAngle = Motors::normalizeAngle(startFileAngle);
    endFileAngle = file.getFinalAngle();
    endFileAngle = Motors::normalizeAngle(endFileAngle);
    
    posisionCase = Sandsara.position();
    if (posisionCase == 2){
        couplingAngle = startFileAngle;
    }
    else if (posisionCase == 0){
        couplingAngle = endFileAngle;
    }
    else{
        couplingAngle = endFileAngle;
    }
    if (true){
        double zf, thetaf, zi, thetai, thetaFinal;
        thetaf = Motors::normalizeAngle(file.getStartAngle() - couplingAngle);
        zf = file.getStartModule();
        thetai = Sandsara.getCurrentAngle();
        zi = Sandsara.getCurrentModule();
        Sandsara.setZCurrent(zi);
        Sandsara.setThetaCurrent(thetai);
        if (thetai > thetaf){
            if (thetai - thetaf > PI){
                thetaFinal = thetai + (2*PI - (thetai - thetaf));
            }
            else{
                thetaFinal = thetaf;
            }
        }
        else{
            if (thetaf - thetai> PI){
                thetaFinal = thetai - (2*PI - (thetaf - thetai));
            }
            else{
                thetaFinal = thetaf;
            }
        }
        errorCode = movePolarTo(zf, thetaFinal, 0);
        if (errorCode != 0){
            return errorCode;
        }
    }
    //if file is thr, the start point is save in order to know where is located.
    if (file.fileType == 2)
    {
        working_status = file.getNextComponents(&component_1, &component_2);
        while (working_status == 3)
        {
            working_status = file.getNextComponents(&component_1, &component_2);
        }
        Sandsara.setZCurrent(component_1);
        Sandsara.setThetaCurrent(component_2 - couplingAngle);
    }
    
    //the next while is stopped until file is finished or is interrupted.
    while (true)
    {
        //Serial.println("ejecutara nueva posicion");
        //====check if speed change====
        
        //====check if you want to change the path====
        if (changePositionList){
            changePositionList = false;
            changeProgram = false;
            goHomeSpiral();
            #ifdef PROCESSING_SIMULATOR
                Serial.println("finished");
            #endif
            if (changeProgram){
                return 30;
            }
            else{
                return 20;
            }
        }
        if (changeProgram){
            changePositionList = false;
            changeProgram = false;
            goHomeSpiral();
            #ifdef PROCESSING_SIMULATOR
                Serial.println("finished");
            #endif
            if (changePositionList){
                return 20;
            }
            else{
                return 30;
            }
        }
        //====check for suspention or pause====
        if (pauseModeGlobal == true){
            while (pauseModeGlobal)
            {
                delay(200);
            }
        }
        if (suspensionModeGlobal == true){
            goHomeSpiral();
            #ifdef PROCESSING_SIMULATOR
                Serial.println("finished");
            #endif
            return 40;
        }
        //====Check if you want to change playlist or Orden mode====
        if (rewindPlaylist){
            goHomeSpiral();
            rewindPlaylist = false;
            #ifdef PROCESSING_SIMULATOR
                Serial.println("finished");
            #endif
            return 1;
        }
        //====
        //====Get new components of the next point to go====
        //Serial.println("antes de next components");
        working_status = file.getNextComponents(&component_1, &component_2);           
        if (working_status == 3)
        {
            continue;
        }
        if (working_status != 0 && working_status != 5)
        {
            break;
        }
        if (working_status == 5){
            Serial.print("ultimo punto\nx = ");
            Serial.print(component_1);
            Serial.print("\ty = ");
            Serial.println(component_2);
            lastPoint = true;
        }
        //====According to type of file the functions moveTo or movePolarTo will be executed====
        if (file.fileType == 1 || file.fileType == 3)
        {
            if (speedChangedMain){
                Sandsara.resetSpeeds();
                speedChangedMain = false;
            }
            Motors::rotate(component_1, component_2, -couplingAngle);
            if (productType) {
                Motors::constrainXY(component_1, component_2);
            }
            distance = Sandsara.module(component_1, component_2, Sandsara.x_current, Sandsara.y_current);
            if (distance > MAX_DISTANCE_FOR_MOVEMENT)
            {
                errorCode = moveInterpolateTo(component_1, component_2, distance);
                lastPoint = false;
                if (errorCode != 0){
                    return errorCode;
                }
            }
            else
            {
                if (lastPoint){
                    Sandsara.lastPoint = true;
                }
                Sandsara.moveTo(component_1, component_2);
                lastPoint = false;
            }
        }
        else if (file.fileType == 2)
        {
            errorCode = movePolarTo(component_1, component_2, couplingAngle, true);
            if (errorCode != 0){
                return errorCode;
            }
        }
        else
        {
            break;
        }
        //Serial.println("saliendo de mover");
    }
    
    //====update z and theta current====
    Sandsara.setZCurrent(Sandsara.getCurrentModule());
    if (Sandsara.getCurrentAngle() > PI){
        Sandsara.setThetaCurrent(Sandsara.getCurrentAngle() - 2*PI);
    }
    else{
        Sandsara.setThetaCurrent(Sandsara.getCurrentAngle());
    }
    //====
    //====check if there was a SD problem====
    if (working_status == -10)
    {
        #ifdef DEBUGGING_DATA
            Serial.println("There were problems for reading SD");
            Serial.println("Se mandara a cero");
        #endif
        movePolarTo(0, 0, 0, true);
        #ifdef PROCESSING_SIMULATOR
            Serial.println("finished");
        #endif
        return -10;
    }
    //====
    Sandsara.completePath();
    //====

    // adding 1 to the counter of played tracks
    tracksPlayed += 1;
    Serial.print("tracks played: ");
    Serial.println(tracksPlayed);

    posisionCase = Sandsara.position();
    if (posisionCase == 2){
        movePolarTo(SdFiles::DISTANCIA_MAX, 0, 0, true);
    }
    else if (posisionCase == 0){
        #ifdef DEBUGGING_DATA
            Serial.println("Se mandara a cero...");
        #endif
        // delay(3000);
        // lastPoint = true;
        movePolarTo(0, 0, 0, true);
        // lastPoint = false;
        Sandsara.completePath();
		if (intermediateCalibration == true || tracksPlayed >= TRACKS_BEFORE_CALIBRATION)
		{
            #ifdef DEBUGGING_DATA
                Serial.println("se realizara la calibracion intermedia");
            #endif

            Serial.println("calibration starts");
			haloCalib.verificacion_cal();
            Serial.println("calibration ends");

            tracksPlayed = 0;
		}
	}
    #ifdef PROCESSING_SIMULATOR
        Serial.println("finished");
    #endif


    delay(1000);
    return 10;
}

/**
 * @brief returns to home (0,0) on a spiral path.
 * 
 */
void goHomeSpiral(){
    //delay(200);
    Sandsara.stopAndResetPositions();
    float currentModule = Sandsara.getCurrentModule();
    availableDeceleration = true;
    if (currentModule < SdFiles::DISTANCIA_MAX / 2){//sqrt(2)){
        goCenterSpiral(false);
    }
    else
    {
        goEdgeSpiral(false);
    }
    availableDeceleration = false;
    delay(1000); 
}

/**
 * @brief returns to home (0,0) on a spiral path.
 * 
 */
void goCenterSpiral(bool stop){
    Sandsara.setSpeed(SPEED_TO_CENTER);
    stopProgramChangeGlobal = stop;
    spiralGoTo(0,PI/2);
    stopProgramChangeGlobal = true;
    Sandsara.setSpeed(romGetSpeedMotor());
    Sandsara.completePath();
}

/**
 * @brief returns to the outer end (SdFiles::DISTANCIA_MAX,0) on a spiral path.
 * 
 */
void goEdgeSpiral(bool stop){
    Sandsara.setSpeed(SPEED_TO_CENTER);
    stopProgramChangeGlobal = stop;
    spiralGoTo(SdFiles::DISTANCIA_MAX,0);
    stopProgramChangeGlobal = true;
    Sandsara.setSpeed(romGetSpeedMotor());
    Sandsara.completePath();
}

/**
 * @brief move to a certain position in spiral path.
 */
void spiralGoTo(float module, float angle){
    float degreesToRotate;
    Sandsara.setZCurrent(Sandsara.getCurrentModule());
    degreesToRotate = int((Sandsara.getCurrentModule() - module)/EVERY_MILIMITERS) * 2*PI;
    Sandsara.setThetaCurrent(Sandsara.getCurrentAngle() + degreesToRotate);
    lastPoint = true;

    movePolarTo(module, angle, 0, true);
    lastPoint = false;
}

void goPolar(double module, double angle){
    Sandsara.setZCurrent(Sandsara.getCurrentModule());
    Sandsara.setThetaCurrent(Sandsara.getCurrentAngle());
    
    lastPoint = true;
    movePolarTo(module, Sandsara.getCurrentAngle()  + angle, 0, true);
    lastPoint = false;

    Sandsara.completePath();
}

//=========================================================
/**
 * @brief this function is used to go from one point to another in a straight line path formed by equidistant points of 1 mm.
 * @param x X axis coordinate of the target point, measured in milimeters.
 * @param y Y axis coordinate of the target point, measured in milimeters.
 * @param distance is the distance between the current point and the target point.
 * @return an error code relating to bluetooth, this could be.
 * 1, playlist has changed.
 * 2, orderMode has changed.
 * 0, finished.
 */
int moveInterpolateTo(double x, double y, double distance)
{
    double alpha = atan2(y - Sandsara.y_current, x - Sandsara.x_current);
    double delta_x, delta_y;
    double x_aux = Sandsara.x_current, y_aux = Sandsara.y_current;
    delta_x = cos(alpha);
    delta_y = sin(alpha);
    int intervals = distance;
	
    for (int i = 1; i <= intervals; i++)
    {
        if (speedChangedMain){
            Sandsara.resetSpeeds();
            speedChangedMain = false;
        }
        //====check if this function has to stop because of program has change, pause or stop. or playlist has changed====
        if ((changePositionList || changeProgram || suspensionModeGlobal || rewindPlaylist) && stopProgramChangeGlobal){
            return 0;
        }
        //====
        x_aux += delta_x;
        y_aux += delta_y;
        if (i == intervals && lastPoint){
            Serial.println("true en el ultimo movimiento");
            Sandsara.lastPoint = true;
        }
        Sandsara.moveTo(x_aux, y_aux);
    }
    if (lastPoint){
        Serial.println("true en el ultimo movimiento");
        Sandsara.lastPoint = true;
    }
    Sandsara.moveTo(x, y);
    return 0;
}

/**
 * @brief Interpola los puntos necesarios entre el punto actual y el siguiente con el objetivo que
 * se mueva en coordenadas polares como lo hace sisyphus.
 * @param component_1 valor en el eje z polar, medido en milimetros.
 * @param component_2 valor en el eje theta polar, medido en radianes.
 * @param couplingAngle es el angulo que se va a rotar el punto con coordenadas polares component_1, component_2 (modulo y angulo, respectivamente).
 * @note es muy importante que se hayan definido las variables zCurrent y thetaCurrent antes.
 * de llamar a esta funcion porque es apartir de estas variables que se calcula cuanto se debe mover.
 * @return un codigo de error:
 * 0, no se recibio nada por bluetooth
 * 1, se cambio la playlist
 * 2, se cambio el orderMode
 */
int movePolarTo(double component_1, double component_2, double couplingAngle, bool littleMovement){
    //Serial.println("dentro de movePolar");
    double zNext = component_1;
    double thetaNext = component_2 - couplingAngle;
    double thetaCurrent = Sandsara.getThetaCurrent();
    double zCurrent = Sandsara.getZCurrent();
    double slicesFactor, distance;
    long slices;
    double deltaTheta, deltaZ;
    double thetaAuxiliar, zAuxliar, xAux, yAux;
    bool lastPointInThisFunction = false;
    if (lastPoint){
        lastPointInThisFunction = true;
        lastPoint = false;
    }
    deltaTheta = thetaNext - thetaCurrent;
    deltaZ = zNext - zCurrent;
    slicesFactor = Sandsara.arcLength(deltaZ, deltaTheta, zCurrent);
    slices = slicesFactor;
    if (slices < 1)
    {
        slices = 1;
    }
    deltaTheta = (thetaNext - thetaCurrent) / slices;
    deltaZ = (zNext - zCurrent) / slices;
    
    for (long i = 0; i < slices; i++)
    {
        if (speedChangedMain){
            Sandsara.resetSpeeds();
            speedChangedMain = false;
        }
        ///====comprobar si se desea cambiar de archivo o suspender o cambiar playlist u orden====
        if ((changePositionList || changeProgram || suspensionModeGlobal || rewindPlaylist) && stopProgramChangeGlobal){
            return 0;
        }
        thetaAuxiliar = thetaCurrent + deltaTheta * double(i);
        zAuxliar = zCurrent + deltaZ * double(i);
        xAux = zAuxliar * cos(thetaAuxiliar);
        yAux = zAuxliar * sin(thetaAuxiliar);
        // si el producto es Stelle entonces limita a x y y.
        if (productType){
            Motors::constrainXY(xAux, yAux);
        }
        distance = Sandsara.module(xAux, yAux, Sandsara.x_current, Sandsara.y_current);
// #ifdef DEBUGGING_DATA
//         Serial.print("distance: ");
//         Serial.println(distance);
// #endif
        if (distance > MAX_DISTANCE_FOR_MOVEMENT)
        {
            if (lastPointInThisFunction && i == (slices - 1)){
                lastPoint = true;
            }
            errorCode = moveInterpolateTo(xAux, yAux, distance);
            lastPoint = false;
            if (errorCode != 0){
                return errorCode;
            }
        }
        else{
            if (lastPointInThisFunction && i == (slices - 1)){
                Sandsara.lastPoint = true;
            }
            Sandsara.moveTo(xAux, yAux, littleMovement);
        }
    }
    xAux = zNext * cos(thetaNext);
    yAux = zNext * sin(thetaNext);

    Sandsara.moveTo(xAux, yAux, littleMovement);
    Sandsara.setThetaCurrent(thetaNext);
    Sandsara.setZCurrent(zNext);
    return 0;
}

//====ROM functions Section====

/**
 * @brief actualiza el valor, en la ROM/FLASH, el nombre de la lista de reproducción.
 * @param str es la dirección en la SD de la lista de reproducción, ejemplo "/animales.playlist".
 * @note únicamente guarda el nombre, ignorando '/' y ".playlist", por ejemplo, si str es "/animales.playlist" solo guarda "animales".
 *pero la funcion romGetPlaylist() la devuelve como "/animales.playList" .
 */
int romSetPlaylist(String str){
    if (str.charAt(0) == '/')
    {
        str.remove(0,1);
    }
    if (str.indexOf(".playlist") >= 0)
    {
        str.remove(str.indexOf(".playlist"));
    }
    if (str.length() > MAX_CHARS_PLAYLIST){
        return -1;
    }
    int i = 0;
    for ( ; i < str.length(); i++){
        EEPROM.write(ADDRESSPLAYLIST + i, str.charAt(i));
    }
    EEPROM.write(ADDRESSPLAYLIST + i, '\0');
    EEPROM.commit();
    return 0;
}

/**
 * @brief recupera el nombre de la playlist guardada en ROM
 * @return la direccion del archivo, ejemplo "/animales.playlist"
 * @note regresa siempre un nombre con terminacion ".playlist"
 */
String romGetPlaylist(){
    String str = "/";
    char chr;
    for (int i = 0; i < MAX_CHARS_PLAYLIST; i++){
        chr = EEPROM.read(ADDRESSPLAYLIST + i);
        if (chr == '\0')
        {
            str.concat(".playlist");
            return str;
        }
        str.concat( chr );
    }
    return "/";
}

/**
 * @brief guarda el tipo de orden de reporduccion en la memoria ROM
 * @param orderMode el valor que corresponde al orden de reproduccion que va a ser almacenado en ROM.
 * @return 0
 */
int romSetOrderMode(int orderMode){
    uint8_t Mode = orderMode;
    EEPROM.write(ADDRESSORDERMODE, Mode);
    EEPROM.commit();
    return 0;
}

/**
 * @brief recupera el valor correspondiente al tipo de reporduccion guardado en la memoria ROM.
 * @return el valor correspondiente al tipo de orden de reporduccion guardado en la ROM.
 */
int romGetOrderMode(){
    uint8_t orderMode;
    orderMode = EEPROM.read(ADDRESSORDERMODE);
    return orderMode;
}

/**
 * @brief guarda la paleta de colores en rom
 * @param pallete es un numero entero que indica la paleta de colres actual
 * @return 0
 */
int romSetPallete(int pallete){
    uint8_t* p = (uint8_t* ) &pallete;
    for (int i = 0; i < sizeof(pallete); i++){
        EEPROM.write(ADDRESSPALLETE + i, *(p + i));
    }
    EEPROM.commit();
    return 0;
}

/**
 * @brief recupera, de la ROM, la ultima palleta de colores guardada.
 * @return un numero entero que indaca una paleta de colores. 
 */
int romGetPallete(){
    int pallete;
    uint8_t* p = (uint8_t* ) &pallete;
    for (int i = 0; i < sizeof(pallete); i++){
        *(p + i) = EEPROM.read(ADDRESSPALLETE + i);
    }
    if (pallete>MAX_PALLETE || pallete < MIN_PALLETE){
        Serial.print("pallete excedido por ");
        Serial.println(pallete);
        pallete = 16;
    }
    return pallete;
}

/**
 * @brief guarda la velocidad de Sandsara en la ROM
 * @param speed es la velocidad en milimetros por segundo de Sandsara.
 * @return 0
 */
int romSetSpeedMotor(int speed){
    uint8_t* p = (uint8_t* ) &speed;
    for (int i = 0; i < sizeof(speed); i++){
        EEPROM.write(ADDRESSSPEEDMOTOR + i, *(p + i));
    }
    EEPROM.commit();
    return 0;
}

/**
 * @brief recupera, de la ROM, la velocidad de Sandsara.
 * @return un numero entero que indaca la velocidad de Sandsara, medida en milimetros por segundo. 
 */
int romGetSpeedMotor(){
    int speed;
    uint8_t* p = (uint8_t* ) &speed;
    for (int i = 0; i < sizeof(speed); i++){
        *(p + i) = EEPROM.read(ADDRESSSPEEDMOTOR + i);
    }
    return speed;
}

/**
 * @brief guarda el tiempo de refresco de los leds en ROM.
 * @param periodLed es el tiempo de refresco de los leds, medido en milisegundos.
 * @return 0
 */
int romSetPeriodLed(int periodLed){
    uint8_t* p = (uint8_t* ) &periodLed;
    for (int i = 0; i < sizeof(periodLed); i++){
        EEPROM.write(ADDRESSPERIODLED + i, *(p + i));
    }
    EEPROM.commit();
    return 0;
}

/**
 * @brief recupera, de la ROM, el tiempo de refresco de los leds.
 * @return el tiempo de refresco de los leds. 
 */
int romGetPeriodLed(){
    int periodLed;
    uint8_t* p = (uint8_t* ) &periodLed;
    for (int i = 0; i < sizeof(periodLed); i++){
        *(p + i) = EEPROM.read(ADDRESSPERIODLED + i);
    }
    return periodLed;
}

/**
 * @brief guarda el nombre del dispositivo bluetooth.
 * @param str corresponde al nombre del bluetooth.
 */
int romSetBluetoothName(String str){
    if (str.length() > MAX_CHARACTERS_BTNAME){
        return -1;
    }
    int i = 0;
    for ( ; i < str.length(); i++){
        EEPROM.write(ADDRESSBTNAME + i, str.charAt(i));
    }
    EEPROM.write(ADDRESSBTNAME + i, '\0');
    EEPROM.commit();
    return 0;
}

/**
 * @brief recupera el nombre del bluetooth guardado en ROM
 * @return el nombre del bluetooth guardado en ROM.
 * @note si no encuentra un nombre guardado, devuelve el nombre "Sandsara".
 */
String romGetBluetoothName(){
    String str = "";
    char chr;
    for (int i = 0; i < MAX_CHARACTERS_BTNAME; i++){
        chr = EEPROM.read(ADDRESSBTNAME + i);
        if (chr == '\0')
        {
            if (str.equals("")){
                return "Sandsara";
            }
            return str;
        }
        str.concat( chr );
    }
    return "Sandsara";
}

/**
 * @brief guarda la variable incrementIndexPallete.
 * @param incrementIndex es el valor que se va a guardar.
 * @return 0.
 */
int romSetIncrementIndexPallete(bool incrementIndex){
    if (incrementIndex){
        EEPROM.write(ADDRESSCUSTOMPALLETE_INCREMENTINDEX, 255);
    }
    else{
        EEPROM.write(ADDRESSCUSTOMPALLETE_INCREMENTINDEX, 0);
    }
    EEPROM.commit();
    return 0;
}

/**
 * @brief recupera la variable incrementIndexPallete.
 * @return true o false dependiendo lo que haya guardado en la memoria.
 */
bool romGetIncrementIndexPallete(){
    uint8_t var = EEPROM.read(ADDRESSCUSTOMPALLETE_INCREMENTINDEX);
    if (var > 0){
        return true;
    }
    else{
        return false;
    }
}

/**
 * @brief guarda una custom pallete en la memoria ROM.
 * @param positions es la posicion del color en la paleta de colores va de 0 a 255.
 * @param red es un array que contiene los valores de Red de las posiciones en la paleta de colores.
 * @param green es un array que contiene los valores de green de las posiciones en la paleta de colores.
 * @param blue es un array que contiene los valores de blue de las posiciones en la paleta de colores.
 * @return un codigo de error que puede siginificar lo siguiente
 * 0, guardo la nueva pallete correctamente
 * -1, el numero de colores no es correcto.
 */
int romSetCustomPallete(uint8_t* positions ,uint8_t* red , uint8_t* green, uint8_t* blue, int numberOfColors){
    if (numberOfColors > 16 || numberOfColors < 1){
        return -1;
    }
    EEPROM.write(ADDRESSCUSTOMPALLETE_COLORS, numberOfColors);
    for (int i = 0; i < numberOfColors; i++){
        EEPROM.write(ADDRESSCUSTOMPALLETE_POSITIONS + i, *(positions + i));
        EEPROM.write(ADDRESSCUSTOMPALLETE_RED + i, *(red + i));
        EEPROM.write(ADDRESSCUSTOMPALLETE_GREEN + i, *(green + i));
        EEPROM.write(ADDRESSCUSTOMPALLETE_BLUE + i, *(blue + i));
    }
    EEPROM.commit();
    return 0;
}

/**
 * @brief recupera una paleta de colores personalizada de la memoria ROM.
 * @param pallete es la paleta que se recupera de la rom.
 */
int romGetCustomPallete(CRGBPalette256 &pallete){
    uint8_t numberOfColors = EEPROM.read(ADDRESSCUSTOMPALLETE_COLORS);
    CRGBPalette256 palleteAuxiliar;
    if (numberOfColors > 16 || numberOfColors < 1){
        uint8_t bytes[8];
        bytes[0] = 0;
        bytes[1] = 255;
        bytes[2] = 0;
        bytes[3] = 0;
        bytes[4] = 255;
        bytes[5] = 0;
        bytes[6] = 255;
        bytes[7] = 0;
        palleteAuxiliar.loadDynamicGradientPalette(bytes);
        pallete = palleteAuxiliar;
        return -1;
    }
    uint8_t newPallete[4*numberOfColors];
    for (int i = 0; i < numberOfColors; i++){
        newPallete[i*4 + 0] = EEPROM.read(ADDRESSCUSTOMPALLETE_POSITIONS + i);
        newPallete[i*4 + 1] = EEPROM.read(ADDRESSCUSTOMPALLETE_RED + i);
        newPallete[i*4 + 2] = EEPROM.read(ADDRESSCUSTOMPALLETE_GREEN + i);
        newPallete[i*4 + 3] = EEPROM.read(ADDRESSCUSTOMPALLETE_BLUE + i);
    }
    rgb2Interpolation(palleteAuxiliar,newPallete);
    pallete = palleteAuxiliar;
    return 0;
}

/**
 * @brief save the variable IntermediateCalibration in ROM.
 * @param state is the value to be saved in ROM.
 * @return a code of error.
 * @note if state is true 0 will be saved in ROM and if it's false 255 will be saved instead.
 */
int romSetIntermediateCalibration(bool state){
    if (state){
        EEPROM.write(ADDRESSINTERMEDIATECALIBRATION,0);
    }
    else
    {
        EEPROM.write(ADDRESSINTERMEDIATECALIBRATION,255);
    }
    EEPROM.commit();
    return 0;
}

/**
 * @brief get the variable IntermediateCalibration from ROM.
 * @return true or false depending on what is stored in ROM.
 * @note if the stored value in ROM is greater than 0 false will be returned if it's not, true will be returned instead.
 */
bool romGetIntermediateCalibration(){
    if (EEPROM.read(ADDRESSINTERMEDIATECALIBRATION) > 0){
        return false;
    }
    else
    {
        return true;
    }
}

/**
 * @brief This function save the current playlist position in ROM.
 * @param pList is the currente position to be saved.
 * @return 0
 */
int romSetPositionList(int pList){
    uint8_t* p = (uint8_t* ) &pList;
    for (int i = 0; i < sizeof(pList); i++){
        EEPROM.write(ADDRESSPOSITIONLIST + i, *(p + i));
    }
    EEPROM.commit();
    return 0;
}

/**
 * @brief restore the current position in the playlist in ROM.
 * @return the postion list saved in ROM.
 */
int romGetPositionList(){
    int pList;
    uint8_t* p = (uint8_t* ) &pList;
    for (int i = 0; i < sizeof(pList); i++){
        *(p + i) = EEPROM.read(ADDRESSPOSITIONLIST + i);
    }
    if (pList > MAX_POSITIONLIST){
        pList = 1;
    }
    return pList;
}

/**
 * @brief stored the direction of leds in ROM.
 * @param direction is the value to be stored.
 * @return an error code.
 */
int romSetLedsDirection(bool direction){
    if (direction){
        EEPROM.write(ADRESSLEDSDIRECTION, 255);
    }
    else{
        EEPROM.write(ADRESSLEDSDIRECTION, 0);
    }
    EEPROM.commit();
    return 0;
}
bool readingBrightness = false;
/**
 * @brief stored the brightness of the led strip in ROM.
 * @param brightness is the value to be stored.
 * @return an error code.
 */
int romSetBrightness(uint8_t brightness){
    while (readingBrightness){
        delay(1);
    }
    readingBrightness = true;
    EEPROM.write(ADDRESSBRIGHTNESS, brightness);
    EEPROM.commit();
    readingBrightness = false;
    return 0;
}
/**
 * @brief to get the brightness of the led strip from ROM.
 * @return the brightness of the led strip.
 */
int romGetBrightness(){
    while (readingBrightness){
        delay(1);
    }
    readingBrightness = true;
    int brightness = EEPROM.read(ADDRESSBRIGHTNESS);
    readingBrightness = false;
    return brightness;
}
/**
 * @brief restored the direction of leds saved in ROM.
 * @return true or false depending on the stored value.
 */
bool romGetLedsDirection(){
    uint8_t var = EEPROM.read(ADRESSLEDSDIRECTION);
    if (var > 0){
        return true;
    }
    else{
        return false;
    }
}

//====Funciones de Leds====

void FillLEDsFromPaletteColors(CRGBPalette256 palette, uint8_t colorIndex)
{
    uint8_t brightness = 255;
    startIndex = colorIndex;
    for( int i = 0; i < NUM_LEDS; i++) {
        leds[i] = ColorFromPalette( palette, colorIndex, brightness, currentBlending);
        if (!incrementIndexGlobal){
            colorIndex =startIndex + float(i+1)*(255.0/float(NUM_LEDS));
        }
    }
}

/**
 * @brief Change the pallette of leds.
 * @param pallet indicates the index pallete to be choosed.
 */
void changePalette(int pallet)
{
    incrementIndexGlobal = romGetIncrementIndexPallete();
    delayLeds = romGetPeriodLed();
    if     ( pallet == 0)   { currentPalette = RainbowColors_p;         currentBlending = LINEARBLEND;}
    else if( pallet == 1)   { currentPalette = pallette1;               currentBlending = LINEARBLEND;}
    else if( pallet == 2)   { currentPalette = pallette2;               currentBlending = LINEARBLEND;}
    else if( pallet == 3)   { currentPalette = pallette3;               currentBlending = LINEARBLEND;}
    else if( pallet == 4)   { currentPalette = pallette4;               currentBlending = LINEARBLEND;}
    else if( pallet == 5)   { currentPalette = pallette5;               currentBlending = LINEARBLEND;}
    else if( pallet == 6)   { currentPalette = pallette6;               currentBlending = LINEARBLEND;}
    else if( pallet == 7)   { currentPalette = pallette7;               currentBlending = LINEARBLEND;}
    else if( pallet == 8)   { currentPalette = pallette8;               currentBlending = LINEARBLEND;}
    else if( pallet == 9)   { currentPalette = pallette9;               currentBlending = LINEARBLEND;}
    else if( pallet == 10)  { currentPalette = pallette10;               currentBlending = LINEARBLEND;}
    else if( pallet == 11)  { currentPalette = pallette11;               currentBlending = LINEARBLEND;}
    else if( pallet == 12)  { currentPalette = pallette12;               currentBlending = LINEARBLEND;}
    else if( pallet == 13)  { currentPalette = pallette13;               currentBlending = LINEARBLEND;}
    else if( pallet == 14)  { currentPalette = pallette14;               currentBlending = LINEARBLEND;}
    else if( pallet == 15)  { currentPalette = pallette15;               currentBlending = LINEARBLEND;}
    else if( pallet == 16)  { romGetCustomPallete(currentPalette);      currentBlending = LINEARBLEND;}
    else if( pallet == CODE_NOSD_PALLETE    )     { currentPalette = NO_SD_PALLETE;           incrementIndexGlobal = false;  delayLeds = DELAYCOLORCODE;}
    else if( pallet == CODE_UPDATING_PALLETE)     { currentPalette = UPTADATING_PALLETE;      incrementIndexGlobal = false;  delayLeds = DELAYCOLORCODE;}
    else if( pallet == CODE_CALIBRATING_PALLETE)  { currentPalette = CALIBRATING_PALLETE;     incrementIndexGlobal = false;  delayLeds = DELAYCOLORCODE;}
    else if( pallet == CODE_SDEMPTY_PALLETE)      { currentPalette = SDEMPTY_PALLETE;         incrementIndexGlobal = false;  delayLeds = DELAYCOLORCODE;}
    else   { currentPalette = RainbowColors_p;         currentBlending = LINEARBLEND;}
}

/**
 * @brief ledsFunc es una tarea que se corre en paralelo y que se encarga de encender los leds.
 */
void ledsFunc( void * pvParameters ){
    //====Configurar fastled====
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
    FastLED.setBrightness( romGetBrightness() );
    // FastLED.setBrightness( 255 );
    int currentBrightness = 0;
    int indexTime = 0;
    //====
    for(;;){
        if (ledsOffGlobal){
            FastLED.clear();
            FastLED.show();
            while(ledsOffGlobal){
                vTaskDelay(100);
            }
        }
        int romBrightness = romGetBrightness();
        if (romBrightness <= 48){ //valores menores a 48 son menores o iguales a 2 al remapearlos con gama8
            romBrightness = 0;
        }
        
        if (currentBrightness < romBrightness){
            currentBrightness += 3;
            if (currentBrightness > romBrightness){
                currentBrightness = romBrightness;
            }
            FastLED.setBrightness(gamma8[currentBrightness]);
        }
        if (currentBrightness > romBrightness){
            currentBrightness -= 3;
            if (currentBrightness < romBrightness){
                currentBrightness = romBrightness;
            }
            FastLED.setBrightness(gamma8[currentBrightness]);
        }
        FillLEDsFromPaletteColors(currentPalette, startIndex);
        for (int i = 0; i < NUM_LEDS; i++){
            leds[i].red = gamma8[leds[i].red];
            leds[i].green = gamma8[leds[i].green];
            leds[i].blue = gamma8[leds[i].blue];
            int sum = leds[i].red + leds[i].green + leds[i].blue;
            if (sum > MAX_LED_SUM){
                double factor = double(MAX_LED_SUM)/double(sum);
                leds[i].red *= factor;
                leds[i].green *= factor;
                leds[i].blue *= factor;
            }
        }      
        FastLED.show();
        if (indexTime >= delayLeds/THREAD_LEDS_TIME){
            if (ledsDirection){
                startIndex += 1;
            }
            else{
                startIndex -= 1;
            }
            indexTime = 0;
        }
        //vTaskDelay(delayLeds);
        FastLED.delay(THREAD_LEDS_TIME);
        indexTime ++;
    }
}

/**
 * @brief this function find if there is a file update in SD, and if there is one the firmware is updated.
 */
void findUpdate(){
    File file;
    File root = SD.open("/");
    String fileName;
    while(true){
        file = root.openNextFile();
        if (!file){
            root.close();
            return;
        }
        char nameF[NAME_LENGTH];
        file.getName(nameF,NAME_LENGTH);
        fileName = nameF;
        if (fileName.indexOf("firmware-") == 0){
            int indexDash1 = fileName.indexOf("-");
            int indexDash2 = fileName.indexOf("-", indexDash1 + 1);
            int indexDot1 = fileName.indexOf(".");
            int indexDot2 = fileName.indexOf(".", indexDot1 + 1);
            int indexDot3 = fileName.indexOf(".", indexDot2 + 1);
            if (indexDash1 == -1 || indexDash2 == -1 || indexDot1 == -1 || indexDot2 == -1 || indexDot3 == -1){
                #ifdef DEBUGGING_DATA
                    Serial.println("No se encontraron 3 puntos y 2 dash");
                #endif
                continue;
            }
            if (!fileName.substring(indexDot3).equals(".bin")){
                #ifdef DEBUGGING_DATA
                    Serial.println("no es un .bin");
                #endif
                continue;
            }
            int v1 = fileName.substring(indexDash1 + 1, indexDot1).toInt();
            int v2 = fileName.substring(indexDot1 + 1, indexDot2).toInt();
            int v3 = fileName.substring(indexDot2 + 1, indexDash2).toInt();
            int hash = fileName.substring(indexDash2 + 1, indexDot3).toInt();
            if (hash != v1 + v2 + v3 + 1){
                #ifdef DEBUGGING_DATA
                    Serial.println("El hash no coincide");
                #endif
                continue;
            }
            if (v1 > v1Current){
                changePalette(CODE_UPDATING_PALLETE);
                int errorCode = programming(fileName);
                if (errorCode == 1){
                    rebootWithMessage("Reiniciando");
                }
                continue;
            }
            else if(v1 == v1Current){
                if (v2 > v2Current){
                    changePalette(CODE_UPDATING_PALLETE);
                    int errorCode = programming(fileName);
                    if (errorCode == 1){
                        rebootWithMessage("Reiniciando");
                    }
                    continue;
                }
                else if(v2 == v2Current){
                    if(v3 > v3Current){
                        changePalette(CODE_UPDATING_PALLETE);
                        int errorCode = programming(fileName);
                        if (errorCode == 1){
                            rebootWithMessage("Reiniciando");
                        }
                        continue;
                    }
                    else{
                        continue;
                    }
                }
                else{
                    continue;
                }
            }
            else{
                continue;
            }
        }
    }
}

/**
 * @brief reordena las lineas del archivo dirFile de forma random en un archivo llamado RANDOM.playlist y borrar el archivo dirFile
 * @param dirFile es la direccion del archivo a ordenar ej. "/lista.txt"
 * @param numberLines es el numero de lineas que contiene el archivo
 * @return 1 si se ordeno de forma correcta
 * -1, no se pudo crear el archivo auxiliar.
 * -2, hubo problemas al leer el archivo dirFile.
 */
int orderRandom(String dirFile, int numberLines){
    File file;
    String fileName;
    int randomNumber, errorCode, limit = numberLines;
    sdRemove("/RANDOM.playlist");
    file = SD.open("/RANDOM.playlist", FILE_WRITE);
    if (!file){
        return -1;
    }
    int availableNumber[numberLines];
    setFrom1(availableNumber, numberLines);
    for(int i = 0; i < limit; i++){
        randomNumber = random(0, numberLines);
        errorCode = SdFiles::getLineNumber(availableNumber[randomNumber], dirFile, fileName);
        if (errorCode < 0){
            file.close();
            sdRemove(dirFile);
            return -2;
        }
        file.print(fileName + "\r\n");
        removeIndex(availableNumber, randomNumber, numberLines);
        numberLines -= 1;
    }
    file.close();
    sdRemove(dirFile);
    return 1;
}

/**
 * @brief inicializa los valores del vector en orden ascendente empezando en 1.
 * @param list es el array que se desea inicializar.
 * @param elements es la cantidad de elementos que va a contener el array.
 * @note ejemplo, el array resultante sera de forma general como este list[0] = 1, list[1] = 2, list[2] = 3... list[elements - 1] = elements.
 */
void setFrom1(int list[], int elements){
    for (int i = 1; i <= elements; i++){
        list[i-1] = i;
    }
}

/**
 * @brief elimina el elemento en la posicion index del array list
 * @param list es el array que se va a modificar.
 * @param index la posicion del elemento que se desea quitar.
 * @note ejemplo un array con los elementos 1,2,3,4,5,6,7,8,9 se le quita su elemento en la posicion 5 quedaria 1,2,3,4,5,7,8,9
 */
void removeIndex(int list[], int index, int elements){
    for(int i=index; i < elements; i++){
        list[i] = list[i+1];
    }
}

/**
 * @brief realiza una interpolacion lineal entre el valor init y stop dividio en tantas partes como lo indica la variable amount.
 * @param init es el valor inicial de la interpolacion.
 * @param stop es el valor final de la interpolacion.
 * @param amount es la cantidad de divisiones que va a tener la linea.
 * @param index es la posicion correspondiente a cada division en la linea, el index 0 representa el valor init.
 * @return el valor en la posicion index de la linea.
 */
double linspace(double init,double stop, int amount,int index){
    return init + (stop - init)/(amount - 1) * (index);
}

/**
 * @brief performs an interpolation with the method of RGB^2
 * @returns an error code.
 * -1 means the first value of matrix is not 0 what is not allowed.
 *  0 means everything is ok.
 */
int rgb2Interpolation(CRGBPalette256& pallete,uint8_t* matrix){
    if (*(matrix) != 0){
        return -1;
    }
    int i = 0;
    for (i = 0; i < 16; i++){
        int index = 0;
        for (int j = *(matrix + i*4); j < *(matrix + i*4 + 4); j++){
            int amount = *(matrix + i*4 + 4) - *(matrix + i*4);
            pallete[j].red = pow(linspace(pow(*(matrix + i*4 + 1),2.2),pow(*(matrix + i*4 + 1 + 4),2.2), amount, index), 1/2.2);
            pallete[j].green = pow(linspace(pow(*(matrix + i*4 + 2),2.2),pow(*(matrix + i*4 + 2 + 4),2.2), amount, index), 1/2.2);
            pallete[j].blue = pow(linspace(pow(*(matrix + i*4 + 3),2.2),pow(*(matrix + i*4 + 3 + 4),2.2), amount, index), 1/2.2);
            index ++;
        }
        if (*(matrix + i*4 + 4) == 255){
            break;
        }
    }
    pallete[255].red = *(matrix + i*4 + 4 + 1);
    pallete[255].green = *(matrix + i*4 + 4 + 2);
    pallete[255].blue = *(matrix + i*4 + 4 + 3);
    return 0;
}

extern double   maxPathSpeedGlobal;
extern long     maxStepsGlobal;
/**
 * @brief mueve los motores 1 y 2.
 * 
 * La velocidad de los motores se ajusta dependiendo de la distancia a recorrer del punto inicial al final, con la intencion
 * de que la velocidad a la que viaje la esfera sea una velocidad constante. Si la velocidad de los motores es constante, la velocidad
 * a la que se mueve la esfera no lo va a ser debido a la geometria del mecanismo.
 * Cada vez que se mueve la posicion a la que se movio se guarda como la posicion actual del robot.
 * @param q1_steps es el numero de pasos que va a girar el motor correspondiente al angulo q1.
 * @param q2_steps es el numero de pasos que va a girar el motor correspondiente al angulo q2.
 * @param distance es la distancia que va a recorrer entre el punto actual y el punto despues del movimiento.
 * @note La distancia se mide en milimetros 
 */
void moveSteps(void* pvParameters)
{ 
    long positions[2];
    float maxSpeed;
    long q1Steps, q2Steps;
    double distance;
    double pathSpeed = romGetSpeedMotor();
    long maxSteps;
    //this initialization has to be performed after Sandsara.init() 
    double q1Current = Sandsara.q1_current, q2Current = Sandsara.q2_current;
    Sandsara.setRealQ1(q1Current);
    Sandsara.setRealQ2(q2Current);
    //double milimiterSpeed = romGetSpeedMotor();
    unsigned long t = micros();
    for (;;){
        if (startMovement){
            q1Steps = q1StepsGlobal;
            q2Steps = q2StepsGlobal;
            distance = distanceGlobal;
            #ifdef IMPLEMENT_ACCELERATION
                pathSpeed = maxPathSpeedGlobal;

                maxSteps = maxStepsGlobal;
            #endif
            startMovement = false;
            
            //Serial.println(pathSpeed);
            #ifdef IMPLEMENT_ACCELERATION
                //pathSpeed
            #endif
            #ifndef IMPLEMENT_ACCELERATION
                if (abs(q1Steps) > abs(q2Steps + q1Steps)){
                    maxSpeed = abs(q1Steps);
                }
                else{
                    maxSpeed = abs(q2Steps + q1Steps);
                }
                maxSpeed = (maxSpeed / distance) * Sandsara.millimeterSpeed;
            #endif
             #ifdef IMPLEMENT_ACCELERATION
                maxSpeed = (maxSteps / distance) * pathSpeed;
            #endif
            if (maxSpeed > MAX_STEPS_PER_SECOND * MICROSTEPPING)
                maxSpeed = MAX_STEPS_PER_SECOND * MICROSTEPPING;

            #ifdef PROCESSING_SIMULATOR
                Serial.print(q1Steps);
                Serial.print(",");
                Serial.print(q2Steps + q1Steps);
                Serial.print(",");
                Serial.println(int(maxSpeed));
            #endif
            
            Sandsara.stepper1.setMaxSpeed(maxSpeed);
            Sandsara.stepper2.setMaxSpeed(maxSpeed);
            positions[0] = Sandsara.stepper1.currentPosition() + q1Steps;
            positions[1] = Sandsara.stepper2.currentPosition() + q2Steps + q1Steps;
            #ifndef DISABLE_MOTORS
                Sandsara.steppers.moveTo(positions);
                /*String info1;
                String info2;
                info1 = "1:" + String(int(Sandsara.stepper1.speed())) + "," + String(q1Steps) + ",1";
                info2 = "2:" + String(int(Sandsara.stepper2.speed())) + "," + String(q2Steps) + "," + String(pathSpeed);
                Serial.println(info1);
                Serial.println(info2);*/
                Sandsara.setRealSpeed1(Sandsara.stepper1.speed());
                Sandsara.setRealSpeed2(Sandsara.stepper2.speed());
                Sandsara.steppers.runSpeedToPosition();
            #endif
            q1Current += Sandsara.degrees_per_step * q1Steps;
            q2Current += Sandsara.degrees_per_step * q2Steps;
            q1Current = Motors::normalizeAngle(q1Current);
            q2Current = Motors::normalizeAngle(q2Current);
            Sandsara.setRealQ1(q1Current);
            Sandsara.setRealQ2(q2Current);
        }
        vTaskSuspend(motorsTask);
    }
}