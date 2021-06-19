#include "Bluetooth.h"
#include <Update.h>
#include <SdFiles.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <BLEAdvertisedDevice.h>
#include "Motors.h"
#include <EEPROM.h>
#include <FastLED.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

int MAX_SPEED_MOTOR = String(String(dataS[34]) + String(dataS[35]) + String(dataS[36])).toFloat();
int MIN_SPEED_MOTOR = String(String(dataS[38])).toFloat();

int MAX_PERIOD_LED = String(String(dataS[40]) + String(dataS[41]) + String(dataS[42])).toFloat();
int MIN_PERIOD_LED = String(String(dataS[44]) + String(dataS[45])).toFloat();

int MAX_PALLETE = String(String(dataS[47]) + String(dataS[48])).toFloat();
int MIN_PALLETE = String(dataS[50]).toFloat();

int MAX_CHARACTERS_BTNAME = String(String(dataS[52]) + String(dataS[53])).toFloat();

int MIN_REPRODUCTION_MODE = String(dataS[55]).toFloat();
int MAX_REPRODUCTION_MODE = String(dataS[57]).toFloat();

int MAX_POSITIONLIST = String(String(dataS[59]) + String(dataS[60]) + String(dataS[61]) + String(dataS[62])).toFloat();

extern  int     periodLedsGlobal;
extern  int     delayLeds;
extern  int     romSetPlaylist(String );
extern  String  romGetPlaylist();
extern  int     romSetOrderMode(int );
extern  int     romGetOrderMode();
extern  int     romSetPallete(int );
extern  int     romGetPallete();
extern  int     romSetSpeedMotor(int );
extern  int     romGetSpeedMotor();
extern  int     romSetPeriodLed(int );
extern  int     romGetPeriodLed();
extern  int     romSetCustomPallete(uint8_t* ,uint8_t* , uint8_t* ,uint8_t*, int);
extern  int     romSetBluetoothName(String );
extern  String  romGetBluetoothName();
extern  int     romSetIntermediateCalibration(bool );
extern  bool    romGetIntermediateCalibration();
extern  int     romSetPositionList(int );
extern  int     romGetPositionList();
extern  bool    romSetIncrementIndexPallete(bool );
extern  int     romGetIncrementIndexPallete();
extern  bool    romSetLedsDirection(bool );
extern  bool    romGetLedsDirection();
extern  int     romSetBrightness(uint8_t );
extern  int     romGetBrightness();
extern  bool    incrementIndexGlobal;
extern  bool    ledsDirection;
extern  int     ledModeGlobal;
extern  void    changePalette(int );
extern  int     stringToArray(String , uint8_t* , int );
extern  bool    changePositionList;
extern  bool    changeProgram;
extern  bool    sdExists(String );
extern  String  playListGlobal;
extern  SdFat   SD;
extern  bool     sdRemove(String );
extern  bool     sdExists(String );
extern  bool     readingSDFile;
extern  int      orderModeGlobal;
extern  bool     rewindPlaylist;
extern  bool     pauseModeGlobal;
extern  String  bluetoothNameGlobal;
extern  bool    suspensionModeGlobal;
extern  Motors Sandsara;
extern  bool    speedChangedMain;
extern  bool    intermediateCalibration;


//====variables====
extern  String      changedProgram;
extern  int         changedPosition;
bool        receiveFlag = false;
bool        sendFlag = false;
char        buffer[10000];
char        *pointerB;
int         bufferSize;
File        fileReceive;
File        fileSend;

//====Prototypes====
int performUpdate(Stream &, size_t );
int updateFromFS(SdFat &, String );
int programming(String );
void rebootWithMessage(String );
int stringToArray(String , uint8_t* , int );

//====BLE Characteristics======
//=============================

//Led strip config
BLECharacteristic *ledCharacteristic_speed;
BLECharacteristic *ledCharacteristic_update;
BLECharacteristic *ledCharacteristic_cycleMode;
BLECharacteristic *ledCharacteristic_direction;
BLECharacteristic *ledCharacteristic_brightness;
BLECharacteristic *ledCharacteristic_amountColors;
BLECharacteristic *ledCharacteristic_positions;
BLECharacteristic *ledCharacteristic_red;
BLECharacteristic *ledCharacteristic_green;
BLECharacteristic *ledCharacteristic_blue;
BLECharacteristic *ledCharacteristic_errorMsg;
BLECharacteristic *ledCharacteristic_indexPalette;

//playlist config
BLECharacteristic *playlistCharacteristic_name;
BLECharacteristic *playlistCharacteristic_pathAmount;
BLECharacteristic *playlistCharacteristic_pathName;
BLECharacteristic *playlistCharacteristic_pathPosition;
BLECharacteristic *playlistCharacteristic_readPlaylistFlag;
BLECharacteristic *playlistCharacteristic_readPath;
BLECharacteristic *playlistCharacteristic_addPath;
BLECharacteristic *playlistCharacteristic_mode;
BLECharacteristic *playlistCharacteristic_progress;
BLECharacteristic *playlistCharacteristic_errorMsg;

//Files
BLECharacteristic *fileCharacteristic_receiveFlag;
BLECharacteristic *fileCharacteristic_receive;
BLECharacteristic *fileCharacteristic_exists;
BLECharacteristic *fileCharacteristic_delete;
BLECharacteristic *fileCharacteristic_sendFlag;
BLECharacteristic *fileCharacteristic_send;
BLECharacteristic *fileCharacteristic_errorMsg;     

//====General====
BLECharacteristic *generalCharacteristic_version;
BLECharacteristic *generalCharacteristic_name;
BLECharacteristic *generalCharacteristic_status;
BLECharacteristic *generalCharacteristic_pause;
BLECharacteristic *generalCharacteristic_play;
BLECharacteristic *generalCharacteristic_sleep;
BLECharacteristic *generalCharacteristic_speed;
BLECharacteristic *generalCharacteristic_restart;
BLECharacteristic *generalCharacteristic_factoryReset;
BLECharacteristic *generalCharacteristic_errorMsg;
BLECharacteristic *generalCharacteristic_calibration;

//================================Callbacks=============================
//======================================================================

class bleServerCallback : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer){
#ifdef DEBUGGING_BLUETOOTH
            Serial.println("BLE Server connected");
        #endif
        Sandsara.setSpeed(SPEED_WHEN_IS_CONNECTED_TO_BLE);
        romSetSpeedMotor(SPEED_WHEN_IS_CONNECTED_TO_BLE);
        speedChangedMain = true;
    }

    void onDisconnect(BLEServer *pServer){
        #ifdef DEBUGGING_BLUETOOTH
            Serial.println("BLE Server disconnected");
        #endif
        //Set motor speed
        int speed = String(generalCharacteristic_speed->getValue().c_str()).toInt();
        speed = map(speed,MIN_SLIDER_MSPEED,MAX_SLIDER_MSPEED,MIN_SPEED_MOTOR,MAX_SPEED_MOTOR);
        if (speed > MAX_SPEED_MOTOR || speed < MIN_SPEED_MOTOR){
            speed = SPEED_MOTOR_DEFAULT;
        }
        Sandsara.setSpeed(speed);
        romSetSpeedMotor(speed);
        speedChangedMain = true;
        //END
        //recieving and sending process
        if (receiveFlag)
        {
            char namePointer [100];
            fileReceive.getName(namePointer, 100);
            String name  = namePointer;
            fileReceive.close();
            if (sdRemove(name)){
                #ifdef DEBUGGING_BLUETOOTH
                    Serial.print("receiving transmision was canceled, file: ");
                    Serial.print(name);
                    Serial.println(" was deleted");
                #endif
            }
            else
            {
                #ifdef DEBUGGING_BLUETOOTH
                Serial.print("receiving transmision was canceled, but file: ");
                Serial.print(name);
                Serial.println(" was not deleted");
                #endif
            }
            pauseModeGlobal = false;
            receiveFlag = false;
        }
        if (sendFlag){
            char namePointer [100];
            fileSend.getName(namePointer, 100);
            String name  = namePointer;
            fileSend.close();
            #ifdef DEBUGGING_BLUETOOTH
                Serial.print("sending transmision was canceled, file: ");
                Serial.print(name);
                Serial.println(" was no sent");
            #endif
            pauseModeGlobal = false;
            sendFlag = false;
        }
        

    }
};

/**
 * @brief **[BLE Command]** This function is called when you write or read the ledSpeed characteristic
 * 
 * **Properties:**  
 * - Write
 * - Read
 * 
 * **UUID: 1a9a7b7e-2305-11eb-adc1-0242ac120002**
 * 
 * **Usage:**  
 * you can write this characteristic to set the speed of the LEDs. In the same way, you can read it to know the speed of the LEDs
 * 
 * **Range:**  
 * accepted values go from 1 to 100, where 1 is the slowest speed and 100 is the fastest speed.
 * 
 * @note when you write to this characteristic you will be notified by the ledCharacteristic_errorMsg with some of these values:
 * - "ok" means the value was set successfully.
 * - "error = -70" the values is out of range.
 */
class speedLedCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxValue = characteristic->getValue();
        String value = rxValue.c_str();
        int periodLed = value.toInt();
        characteristic->setValue(String(periodLed).c_str());
        periodLed = map(periodLed, MIN_SLIDER_LEDSPEED,MAX_SLIDER_LEDSPEED,MAX_PERIOD_LED,MIN_PERIOD_LED);
        if(periodLed < MIN_PERIOD_LED || periodLed > MAX_PERIOD_LED){
            ledCharacteristic_errorMsg->setValue("error = -70");
            ledCharacteristic_errorMsg->notify();
            return;
        }
        #ifdef DEBUGGING_BLUETOOTH
            Serial.print("WRITE ledSpeed: ");
            Serial.println(periodLed);
        #endif
        periodLedsGlobal = periodLed;
        delayLeds = periodLed;
        romSetPeriodLed(periodLedsGlobal);
        ledCharacteristic_errorMsg->setValue("ok");
        ledCharacteristic_errorMsg->notify();
    } //onWrite
    #ifdef DEBUGGING_BLUETOOTH
        void onRead(BLECharacteristic *characteristic)
        {
            std::string rxValue = characteristic->getValue();
            String value = rxValue.c_str();
            Serial.print("READ ledSpeed: ");
            Serial.println(value);
        }
    #endif
};

/**
 * @brief **[BLE Command]** This function is called when you write or read the cycleMode characteristic
 * 
 * the cycle mode is used to control the sequence of the LEDs and there is 2 mode, cycle and solid.  
 * - **cycle mode** means that the LED strip will show all the colors from the selected palette at the same tame.  
 * - **solid mode** means that the LED strip will set a same color choosed from the selected palette for all the LEDs in the LED strip.  
 * 
 * **Properties:**  
 * - Write
 * - Read
 * 
 * **UUID: 1a9a7dea-2305-11eb-adc1-0242ac120002**
 * 
 * **Usage:**  
 * you can write to this characteristic to set the cycle mode. In the same way, you can read it to know the cycle mode
 * 
 * **Range:**  
 * accepted values are 0 or any other diffeent than 0.
 * - 0 to set cycle mode.
 * - any other number to set solid mode.  
 * 
 * @note when you write to this characteristic you will be notified by the ledCharacteristic_errorMsg with some of these values:
 * - "ok" means the value was set successfully.
 * - "error = -70" the values is out of range.
 */
class cycleModeCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxValue = characteristic->getValue();
        String value = rxValue.c_str();
        int cycleMode = value.toInt();
        if (cycleMode > 0){
            romSetIncrementIndexPallete(true);
        }
        else{
            romSetIncrementIndexPallete(false);
        }
        incrementIndexGlobal = romGetIncrementIndexPallete();
        #ifdef DEBUGGING_BLUETOOTH
            Serial.print("WRITE cycleMode: ");
            Serial.println(incrementIndexGlobal);
        #endif
        characteristic->setValue(String(incrementIndexGlobal).c_str());
        ledCharacteristic_errorMsg->setValue("ok");
        ledCharacteristic_errorMsg->notify();
    } //onWrite

    #ifdef DEBUGGING_BLUETOOTH
        void onRead(BLECharacteristic *characteristic)
        {
            std::string rxValue = characteristic->getValue();
            String value = rxValue.c_str();
            Serial.print("READ cycleMode: ");
            Serial.println(value);
        }
    #endif
};

/**
 * @brief **[BLE Command]** This class is used to create and object to be called when you write the ledDirection characteristic.
 * 
 * the led strip has 2 directions, clock wise and counter clockwise, you can choose one changing the characteristic.
 * 
 * **Properties:**  
 * - Write
 * - Read
 * 
 * **UUID: 1a9a8042-2305-11eb-adc1-0242ac120002**
 * 
 * **Usage:**  
 * you can write to this characteristic to set direction of the led strip, or in the same way you can read it to know it.
 * 
 * **Range:**  
 * accepted values are 0 or any other diffeent than 0.
 * - 1 for clockwise direction.
 * - 0 for counter clockwise direction.  
 * 
 * @note when you write to this characteristic you will be notified by the ledCharacteristic_errorMsg with some of these values:
 * - "ok" means the value was set successfully.
 */
class directionCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        //retorna ponteiro para o registrador contendo o valor atual da caracteristica
        std::string rxValue = characteristic->getValue();
        String value = rxValue.c_str();
        int direction = value.toInt();
        if (direction != 0){
            romSetLedsDirection(true);
            #ifdef DEBUGGING_BLUETOOTH
                Serial.print("WRITE direction: ");
                Serial.println("true");
            #endif
            characteristic->setValue("1");
        }
        else{
            romSetLedsDirection(false);
            #ifdef DEBUGGING_BLUETOOTH
                Serial.print("WRITE direction: ");
                Serial.println("false");
            #endif
            characteristic->setValue("0");
        }
        
        ledsDirection = romGetLedsDirection();

        ledCharacteristic_errorMsg->setValue("ok");
        ledCharacteristic_errorMsg->notify();
    } //onWrite

    #ifdef DEBUGGING_BLUETOOTH
        void onRead(BLECharacteristic *characteristic)
        {
            std::string rxValue = characteristic->getValue();
            String value = rxValue.c_str();
            Serial.print("READ ledDirection: ");
            Serial.println(value);
        }
    #endif
};

/**
 * @brief **[BLE Command]** This class is used to create and object to be called when you write the ledBrightness characteristic.
 * 
 * you can change the brightness of the ledsrip just modifying this characteristic.
 * 
 * **Properties:**  
 * - Write
 * - Read
 * 
 * **UUID: 1a9a8948-2305-11eb-adc1-0242ac120002**
 * 
 * **Usage:**  
 * you have to write to this characteristic the percentage of the desired brightness or you can read it to know it.
 * 
 * **Range:**  
 * - the brightness characteristic accepts values between 0 and 100. 
 * 
 * **posible reponses:**
 * - "ok" the desired brightness was set.
 * - "error= -1" the brightness is our of range.
 * 
 * @note when you write to this characteristic you will be notified by the ledErrorMsg characteristic with the response.
 * @see 
 */
class setBrightnessCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        //retorna ponteiro para o registrador contendo o valor atual da caracteristica
        std::string rxValue = characteristic->getValue();
        String value = rxValue.c_str();
        int brightness = value.toInt();
        #ifdef DEBUGGING_BLUETOOTH
            Serial.print("WRITE brightness: ");
            Serial.println(brightness);
        #endif
        characteristic->setValue(String(brightness).c_str());
        brightness = map(brightness,MIN_SLIDER_BRIGHTNESS,MAX_SLIDER_BRIGHTNESS,48,255);
        
        if(brightness < 0 || brightness > 255){
            characteristic->setValue(String(romGetBrightness()).c_str());
            ledCharacteristic_errorMsg->setValue("error = -1");
            ledCharacteristic_errorMsg->notify();
            return;
        }
        //FastLED.setBrightness(brightness);
        romSetBrightness(brightness);
        
        ledCharacteristic_errorMsg->setValue("ok");
        ledCharacteristic_errorMsg->notify();
    } //onWrite

    #ifdef DEBUGGING_BLUETOOTH
        void onRead(BLECharacteristic *characteristic)
        {
            std::string rxValue = characteristic->getValue();
            String value = rxValue.c_str();
            Serial.print("READ brightness: ");
            Serial.println(value);
        }
    #endif
};

/**
 * @brief **[BLE Command]** This class is used to create and object to be called when you write the selectedPalette characteristic.
 * 
 * **this chacteristic isdeprecated**  
 * this characteristic used to be used to change between different preset palettes hardcoded, but this charateristic is deprecated because
 * it is posible we delete the hardcoded palettes.
 * 
 * **range: **  
 * "0" to "15" to select a preset palette.  
 * "16" to select the custom palette.
 */
class selectedPaletteCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxValue = characteristic->getValue();
        String value = rxValue.c_str();
        int valueInt = value.toInt();
        if(valueInt < MIN_PALLETE || valueInt > MAX_PALLETE){
            ledCharacteristic_errorMsg->setValue("error = -31");
            ledCharacteristic_errorMsg->notify();
            #ifdef DEBUGGING_BLUETOOTH
                Serial.println("WRITE presetPalette ERROR");
            #endif
            return;
        }
        
        ledModeGlobal = valueInt;
        changePalette(ledModeGlobal);
        romSetPallete(ledModeGlobal);
        #ifdef DEBUGGING_BLUETOOTH
            Serial.print("WRITE presetPalete: ");
            Serial.println(ledModeGlobal);
        #endif
        ledCharacteristic_errorMsg->setValue("ok");
        ledCharacteristic_errorMsg->notify();
    } //onWrite

    #ifdef DEBUGGING_BLUETOOTH
        void onRead(BLECharacteristic *characteristic)
        {
            std::string rxValue = characteristic->getValue();
            String value = rxValue.c_str();
            Serial.print("READ presetPalette: ");
            Serial.println(value);
        }
    #endif
};

/**
 * @brief **[BLE Command]** This function is called when you write the UpdatePalette characteristic
 * 
 * You can define a custom palette to be shown in the led strip and you only need to send the “main colors” of the palette
 * ( as low as 2 and up to 16) which Sandsara internally interpolates to a 256 color gradient.
 * 
 * Properties:  
 * - Write
 * 
 * **UUID: 1a9a87b8-2305-11eb-adc1-0242ac120002**
 * 
 * Usage:  
 * To define the custom palette, your need the next information: Amount of Colors, Positions, Red, Green and Blue.
 * only write to this characteristic the amount of colors, positions, reds, greens and blues values. **Every value must be a Byte**.
 * 
 * Range:  
 * - amount of colors: goes from 2 to 16.  
 * - positions: every position goes from 0 to 255.  
 * - reds: every red value goes from 0 to 255.  
 * - greens: every green value goes from 0 to 255.  
 * - blues: every blue value goes from 0 to 255.  
 * 
 * In this image you can see more clearly how to send a palette.
 * \image html updatePaletteExample.png
 * 
 * posible reponses:
 * - "ok" the custom palette was updated successfully.
 * - "error= -1" there is no data to update.
 * - "error= -2 the number of colors is out of range"
 * - "error= -3 the data is incompleted"  
 * 
 * if you want to know what is the current palette you need to read the next characteristics
 * - amount of colors characteristic (UUID: 1a9a820e-2305-11eb-adc1-0242ac120002)
 * - positions characteristic (UUID: 1a9a82d6-2305-11eb-adc1-0242ac120002)
 * - red characteristic (UUID: 1a9a83a8-2305-11eb-adc1-0242ac120002)
 * - green characteristic (UUID: 1a9a8466-2305-11eb-adc1-0242ac120002)
 * - blue characteristic (UUID: 1a9a852e-2305-11eb-adc1-0242ac120002)
 * @note when you write to this characteristic you will be notified by the ledErrorMsg characteristic with the response.
 */
class CallbacksToUpdate : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxValue = characteristic->getValue();
        String value = rxValue.c_str();
        //=====
        int sizeData = rxValue.length();
        #ifdef DEBUGGING_DATA
            Serial.print("data size: ");
            Serial.println(sizeData);
        #endif
        if (sizeData <= 0){
            #ifdef DEBUGGING_BLUETOOTH
                Serial.println("Write null data for palette");
            #endif
            ledCharacteristic_errorMsg->setValue("error= -1"); //null data
            ledCharacteristic_errorMsg->notify();
            return;
        }
        char data[sizeData+1];
        rxValue.copy(data,sizeData,0);
        Serial.print("data 0x ");
        for (int i = 0; i < sizeData; i++){
            Serial.print(data[i],HEX);
            Serial.print("-");
        }
        Serial.println("");
        int n = data[0]; //amount of colors
        //check if the amuont of colors is valid
        if (n < 2 || n > 16){
            #ifdef DEBUGGING_BLUETOOTH
                Serial.println("not a valid number of colors");
            #endif
            ledCharacteristic_errorMsg->setValue("error= -2"); //not a valid number of colors
            ledCharacteristic_errorMsg->notify();
            return;}
        // check for incomplete data
        if (n*4 != sizeData - 1){
            #ifdef DEBUGGING_BLUETOOTH
                Serial.println("WRITE incomplete data");
            #endif
            ledCharacteristic_errorMsg->setValue("error= -3"); //incomplete data
            ledCharacteristic_errorMsg->notify();
            return;
        }

        uint8_t positions[n];
        uint8_t red[n];
        uint8_t green[n];
        uint8_t blue[n];
        
        #ifdef DEBUGGING_BLUETOOTH
            Serial.print("Positions: ");
        #endif
        for(int i=0; i<n; i++){
            positions[i] = data[i + 1];
            #ifdef DEBUGGING_BLUETOOTH
                Serial.print(positions[i]);
                Serial.print(",");
            #endif
        }
        #ifdef DEBUGGING_BLUETOOTH
            Serial.print("\nred: ");
        #endif
        for(int i=0; i<n; i++){
            red[i] = data[n + i + 1];
            #ifdef DEBUGGING_BLUETOOTH
                Serial.print(red[i]);
                Serial.print(",");
            #endif
        }
        #ifdef DEBUGGING_BLUETOOTH
            Serial.print("\ngreen: ");
        #endif
        for(int i=0; i<n; i++){
            green[i] = data[2*n + i + 1];
            #ifdef DEBUGGING_BLUETOOTH
                Serial.print(green[i]);
                Serial.print(",");
            #endif
        }
        #ifdef DEBUGGING_BLUETOOTH
            Serial.print("\nblue: ");
        #endif
        for(int i=0; i<n; i++){
            blue[i] = data[3*n + i + 1];
            #ifdef DEBUGGING_BLUETOOTH
                Serial.print(blue[i]);
                Serial.print(",");
            #endif
        }
        #ifdef DEBUGGING_BLUETOOTH
            Serial.println("");
        #endif

        romSetCustomPallete(positions, red, green, blue, n);
        Bluetooth::setRed();
        Bluetooth::setGreen();
        Bluetooth::setBlue();
        Bluetooth::setPositions();
        Bluetooth::setAmountOfColors();

        ledModeGlobal = 16;
        changePalette(ledModeGlobal);
        romSetPallete(ledModeGlobal);
        ledCharacteristic_errorMsg->notify();
        #ifdef DEBUGGING_BLUETOOTH
            Serial.println("Custom palette was updated");
        #endif
        ledCharacteristic_errorMsg->setValue("ok");
    } //onWrite

    #ifdef DEBUGGING_BLUETOOTH
        void onRead(BLECharacteristic *characteristic)
        {
            std::string rxValue = characteristic->getValue();
            String value = rxValue.c_str();
            Serial.print("READ toUpdate: ");
            Serial.println(value);
        }
    #endif
};

class genericCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        //retorna ponteiro para o registrador contendo o valor atual da caracteristica
        std::string rxValue = characteristic->getValue();
        std::string uuid = characteristic->getUUID().toString();
        String value = rxValue.c_str();
        #ifdef DEBUGGING_BLUETOOTH
            for (int i = 0; i < uuid.length(); i++)
            {
                Serial.print(uuid[i]);
            }
        #endif
        //verifica se existe dados (tamanho maior que zero)
        #ifdef DEBUGGING_BLUETOOTH
            Serial.print(" Value was changed to: ");
            for (int i = 0; i < rxValue.length(); i++)
            {
                Serial.print(rxValue[i]);
            }
            Serial.println();
        #endif
    } //onWrite

    #ifdef DEBUGGING_BLUETOOTH
        void onRead(BLECharacteristic *characteristic)
        {
            std::string uuid = characteristic->getUUID().toString();
            std::string rxValue = characteristic->getValue();
            String value = rxValue.c_str();
            // Serial.print("READ ");
            String uuidString = characteristic->getUUID().toString().c_str();
            if (uuidString.equals(CHARACTERISTIC_UUID_AMOUNTCOLORS)){
                Serial.print("READ amountColors: ");
                Serial.println(value);
            }
            else if(uuidString.equals(CHARACTERISTIC_UUID_POSITIONS)){
                Serial.print("READ positions: ");
                Serial.println(value);
            }
            else if(uuidString.equals(CHARACTERISTIC_UUID_RED)){
                Serial.print("READ red: ");
                Serial.println(value);
            }
            else if(uuidString.equals(CHARACTERISTIC_UUID_GREEN)){
                Serial.print("READ green: ");
                Serial.println(value);
            }
            else if(uuidString.equals(CHARACTERISTIC_UUID_BLUE)){
                Serial.print("READ blue: ");
                Serial.println(value);
            }
            else{
                Serial.print("READ ");
                Serial.print(uuidString);
                Serial.print(": ");
                Serial.println(value);
            }
        }
    #endif
};

//====================callbacks for playlist config==============================================
//==========================================================================================================

/**
 * @brief **[BLE Command]** This class is used to create and object to be called when you write or read the playlistName characteristic.
 * 
 * Sandsara is always playing back a playlist that contains a certain paths. This characteristic is used to know or change the current playlist.
 * all the playlist are store in a text file but with .playlist extension and this contains the file name of the path to be reproduced.
 * For example, a generic playlist file will looks like this:  
 * ~~~
 * path1.txt
 * path2.txt
 * path3.txt
 * path4.txt
 * ~~~
 * 
 * **Properties:**  
 * - Write
 * - Read
 * 
 * **UUID: 9b12a048-2c6e-11eb-adc1-0242ac120002**
 * 
 * **Usage:**  
 * you can write to this characteristic to change to a playlist, or in the same way you can read it to know the current playlist.
 * To change a playlist just write to the this characteristic the name of the playlist you want to playback (ignoring its extension, 
 * for example, if you want to change to “geometry.playlist” just write “geometry”).
 * Be sure you have sent the playlist to Sandsara before or make sure it already exists in the SD card.
 * 
 * 
 * **Range:**  
 * accepted values are 0 or any other diffeent than 0.
 * - 1 for clockwise direction.
 * - 0 for counter clockwise direction.  
 * 
 * **responses: **  
 * - "ok" means the playlist was changed successfully.
 * - "-1" means the playlist you try to change does not exist.
 * 
 * @note when you write to this characteristic you will be notified by the playlistCharacteristic_errorMsg characteritic for a response.
 * 
 * @see How to send a file? --> FilesCallbacks_receive
 */
class playlistCallbacks_name : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxValue = characteristic->getValue();
        String value = rxValue.c_str();
        String playList = value + ".playlist";
        if (!sdExists(playList)){
            playlistCharacteristic_errorMsg->setValue("error= -1");
            playlistCharacteristic_errorMsg->notify();
            #ifdef DEBUGGING_BLUETOOTH
                Serial.print("WRITE playlist not exists: ");
                Serial.println(playList);
            #endif
            Bluetooth::setPlaylistName(playListGlobal);
            return;
        }
        playListGlobal = "/" + playList;
        romSetPlaylist(playListGlobal);
        orderModeGlobal = 1;
        romSetOrderMode(orderModeGlobal);
        rewindPlaylist = true;
        #ifdef DEBUGGING_BLUETOOTH
            Serial.print("WRITE playlist: ");
            Serial.println(playListGlobal);
        #endif
        playlistCharacteristic_errorMsg->setValue("ok");
        playlistCharacteristic_errorMsg->notify();
    } //onWrite 

    #ifdef DEBUGGING_BLUETOOTH
        void onRead(BLECharacteristic *characteristic)
        {
            std::string rxValue = characteristic->getValue();
            String value = rxValue.c_str();
            Serial.print("READ playlistName: ");
            Serial.println(value);
        }
    #endif
};

/**
 * @brief **[BLE Command]** This class is used to create and object to be called when you write or read the pathName characteristic.
 * 
 * all the paths has and unique name and the sandsara tracks are name starting with "Sandsara-trackNumber-", for example, "Sandsara-trackNumber-0001.thr".
 * you can change or know the current path with this characteristic.
 * 
 * **Properties:**  
 * - Write
 * - Read
 * 
 * **UUID: 9b12a534-2c6e-11eb-adc1-0242ac120002**
 * 
 * **Usage:**  
 * you can write to this characteristic to change the current path, or in the same way you can read it to know it.
 * 
 * **Range:**  
 * accepted values are any name, for example "my track.thr", "path x.txt", etc.
 * 
 * **responses: **  
 * - "ok" means the path was changed successfully.
 * - "-1" means the playlist you try to change does not exist.
 * 
 * this characteristic is used to "test" a path, this is the behavior whe you change a path using this characteristic.
 * 1. sandsara is playing the Geometries playlist and it is playing the 15 postion track.
 * 2. you change to "my track.txt" using this characteristic.
 * 3. sandara will play "my track.txt"
 * 4. when it finish, sandsara will go back to 15 positon track of the Geometries playlist.
 * 
 * @note when you write to this characteristic you will be notified by the playlistCharacteristic_errorMsg characteritic for a response.
 * @warning we recomment to use playlistCallbacks_pathPosition to change a path.
 * @see How to send a file? --> FilesCallbacks_receive
 */
class playlistCallbacks_pathName : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxValue = characteristic->getValue();
        String name = rxValue.c_str();
        #ifdef DEBUGGING_BLUETOOTH
            Serial.print("WRITE pathName: ");
            Serial.println(name);
        #endif
        if (SdFiles::getType(name) < 0){
            playlistCharacteristic_errorMsg->setValue("error= -2");
            playlistCharacteristic_errorMsg->notify();
            #ifdef DEBUGGING_BLUETOOTH
                Serial.println("Nombre incorrecto");
            #endif
            return;
        }
        if (!sdExists(name)){
            playlistCharacteristic_errorMsg->setValue("error= -3");
            playlistCharacteristic_errorMsg->notify();
            #ifdef DEBUGGING_BLUETOOTH
                Serial.println("no existe el archivo");
            #endif
            return;
        }
        changedProgram = name;
        playlistCharacteristic_errorMsg->setValue("ok");
        playlistCharacteristic_errorMsg->notify();
        changeProgram = true;
        changePositionList = false;
    } //onWrite

    #ifdef DEBUGGING_BLUETOOTH
        void onRead(BLECharacteristic *characteristic)
        {
            std::string rxValue = characteristic->getValue();
            String value = rxValue.c_str();
            Serial.print("READ pathName: ");
            Serial.println(value);
        }
    #endif
};

/**
 * @brief **[BLE Command]** This class is used to create and object to be called when you write or read the pathPosition characteristic.
 * 
 * Sandsara always are playing a playlist and every path has an index starting at 1, you can change the current path by its index in the playlist.
 * 
 * **Properties:**  
 * - Write
 * - Read
 * 
 * **UUID: 9b12a62e-2c6e-11eb-adc1-0242ac120002**
 * 
 * **Usage:**  
 * you can change the current path just writing to this characteristic the index of the desied path, or in the same way you can read it to know current path index.
 * 
 * 
 * **Range:**  
 * accepted values goes from 1 to the number of paths in the current playlist.
 * 
 * **Responses:**  
 * - "ok" means the path was changed successfully.
 * 
 * this is the behavior whe you change a path using this characteristic.
 * 1. sandsara is playing the Geometries playlist and it is playing the 15 postion track.
 * 2. you change to third position path using this characteristic.
 * 3. sandara will play the third position path.
 * 4. when it finish, sandsara will continue with the fourth index path.
 * 
 * @note when you write to this characteristic you will be notified by the playlistCharacteristic_errorMsg characteritic for a response.
 * @warning make sure to change to a valid index
 * @see How to send a file? --> FilesCallbacks_receive
 */
class playlistCallbacks_pathPosition : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxValue = characteristic->getValue();
        String value = rxValue.c_str();
        int position = value.toInt();
        #ifdef DEBUGGING_BLUETOOTH
            Serial.print("WRITE pathPosition: ");
            Serial.println(position);
        #endif
        characteristic->setValue(String(position).c_str());
        changedPosition = position;
        Bluetooth::setPercentage(0);
        playlistCharacteristic_errorMsg->setValue("ok");
        playlistCharacteristic_errorMsg->notify();
        changePositionList = true;
        changeProgram = false;
    } //onWrite

    #ifdef DEBUGGING_BLUETOOTH
        void onRead(BLECharacteristic *characteristic)
        {
            std::string rxValue = characteristic->getValue();
            String value = rxValue.c_str();
            Serial.print("READ pathPosition: ");
            Serial.println(value);
        }
    #endif
};

/**
 * @brief **[BLE Command]** Deprecated.
 */
class playlistCallbacks_addPath : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxValue = characteristic->getValue();
        String pathName = rxValue.c_str();
        while (readingSDFile){
            delay(1);
        }
        readingSDFile = true;
        File file = SD.open(playListGlobal, FILE_WRITE);
        if (!file){
            playlistCharacteristic_errorMsg->setValue("error= -4");
            playlistCharacteristic_errorMsg->notify();
            #ifdef DEBUGGING_BLUETOOTH
                Serial.println("no existe el path en la SD");
            #endif
            return; 
        }
        pathName = "\r\n" + pathName;
        file.print(pathName);
        file.close();
        readingSDFile = false;
        #ifdef DEBUGGING_BLUETOOTH
            Serial.print("path name added: ");
            Serial.println(pathName);
        #endif
        playlistCharacteristic_errorMsg->setValue("ok");
        playlistCharacteristic_errorMsg->notify();
    } //onWrite

    #ifdef DEBUGGING_BLUETOOTH
        void onRead(BLECharacteristic *characteristic)
        {
            std::string rxValue = characteristic->getValue();
            String value = rxValue.c_str();
            Serial.print("READ addPath: ");
            Serial.println(value);
        }
    #endif
};

/**
 * @brief **[BLE Command]** Deprecated.
 */
class playlistCallbacks_mode : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxValue = characteristic->getValue();
        String value = rxValue.c_str();
        int mode = value.toInt();
        if(mode < MIN_REPRODUCTION_MODE || mode > MAX_REPRODUCTION_MODE){
            characteristic->setValue(String(orderModeGlobal).c_str());
            playlistCharacteristic_errorMsg->setValue("error= -5");
            playlistCharacteristic_errorMsg->notify();
            return;
        }
        orderModeGlobal = mode;
        romSetOrderMode(orderModeGlobal);
        rewindPlaylist = true;
        #ifdef DEBUGGING_BLUETOOTH
            Serial.print("WRITE playlistMode: ");
            Serial.println(mode);
        #endif
        playlistCharacteristic_errorMsg->setValue("ok");
        playlistCharacteristic_errorMsg->notify();
    } //onWrite

    #ifdef DEBUGGING_BLUETOOTH
        void onRead(BLECharacteristic *characteristic)
        {
            std::string rxValue = characteristic->getValue();
            String value = rxValue.c_str();
            Serial.print("READ playlistMode: ");
            Serial.println(value);
        }
    #endif
};

//================================Sending files=====================================
//==================================================================================

/**
 * @brief **[BLE Command]** This class is used to create and object to be called when you write the receiveFlag characteristic.
 * 
 * receiveFlag characteristic is used to start the transfer of a file from the BLE client to this ESP32.
 * 
 * **Properties:**  
 * - Write
 * 
 * **UUID: fcbff68e-2af1-11eb-adc1-0242ac120002**
 * 
 * **Usage:**  
 * in order to transfer a file to this device you need to follow the next steps:
 * 1. Activate notifications of the FilesCallbacks_send characteristic asociated to this class.
 * 2. write to this characteristic the name of file you want to transfer.
 * 3. write to the FilesCallbacks_send the bytes of the file in chunks of 512 bytes.
 * 4. wait to be notified from theFilesCallbacks_send.
 * 5. repeat steps 3 and 4 until you have sent all the bytes of the file
 * 6. write to this characteristic any message to finish the transfer
 * 7. you will be notify by FileErrorMsg characteristic with "done"
 * 
 * **Range:**  
 * accepted values are a name, for example, "New Path.thr" to start the transmition of a file and any other character to finish it.  
 * 
 * @note you may will be notified by fileErrorMsg characteristic with some of this massage:
 * - "ok" means the file was created and is waiting for the bytes of the file.
 * - "error= -2" means the file couldn't be created maybe because there is no sd card inserted.
 * @warning if you try to send a file named equals than other inside the sd card it will be deleted to accept this new file, so if you're
 * not sure if the name of your file name already exists in the sd card, Use the FilesCallbacks_checkFile characteristic.
 */
class FilesCallbacks_receiveFlag : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        if (!receiveFlag){
            std::string rxData = characteristic->getValue();
            String name = rxData.c_str();
            #ifdef DEBUGGING_BLUETOOTH
                Serial.print("Inicia recepcion de archivo: ");
                Serial.println(name);
            #endif
            while (readingSDFile){
                delay(1);
            }
            readingSDFile = true;
            if (sdExists(name)){
                //fileCharacteristic_errorMsg->setValue("error= -1"); //archivo ya existe
                //fileCharacteristic_errorMsg->notify();
                sdRemove(name);
                //readingSDFile = false;
                #ifdef DEBUGGING_BLUETOOTH
                    Serial.println("se borro el archivo para poder ser recibido");
                #endif
            }
            fileReceive = SD.open(name, FILE_WRITE);
            if (!fileReceive)
            {
                fileCharacteristic_errorMsg->setValue("error= -2"); //file cannot be opened 
                fileCharacteristic_errorMsg->notify();
                #ifdef DEBUGGING_BLUETOOTH
                    Serial.println("no se pudo abrir el archivo"); //file cannot be opened
                #endif
                readingSDFile = false;
                return;
            }
            #ifdef DEBUGGING_BLUETOOTH
                //Serial.println("recibiendo bytes...");
            #endif
            pointerB = buffer;
            bufferSize = 0;
            receiveFlag = true;
            pauseModeGlobal = true;
            readingSDFile = false;
            fileCharacteristic_errorMsg->setValue("ok");
            fileCharacteristic_errorMsg->notify();
        }
        else{
            pointerB = buffer;
            while (readingSDFile){
                delay(1);
            }
            readingSDFile = true;
            fileReceive.write(buffer, bufferSize);
            readingSDFile = false;
            #ifdef DEBUGGING_BLUETOOTH
                //Serial.print("ultimo buffer size: ");
                //Serial.println(bufferSize);
            #endif
            bufferSize = 0;
            fileReceive.close();
            #ifdef DEBUGGING_BLUETOOTH
                Serial.println("termino archivo");
            #endif
            receiveFlag = false;
            pauseModeGlobal = false;
            fileCharacteristic_errorMsg->setValue("done");
            fileCharacteristic_errorMsg->notify();
        }
    }
};

/**
 * @brief **[BLE Command]** This class is used to create and object to be called when you write the FileReceive characteristic.
 * 
 * receive characteristic is used to receive the bytes of the file you want to transfer.
 * 
 * **Properties:**  
 * - Write
 * - notify
 * 
 * **UUID: fcbffa44-2af1-11eb-adc1-0242ac120002**
 * 
 * **Usage:**  
 * in order to transfer a file to this device you need to follow the next steps:
 * 1. Activate notifications of this characteristic.
 * 2. write to FilesCallbacks_receiveFlag the name of file you want to transfer.
 * 3. write to this characteristic the bytes of the file in chunks of 512 bytes.
 * 4. wait to be notified from this characteristic.
 * 5. repeat steps 3 and 4 until you have sent all the bytes of the file
 * 6. write to FilesCallbacks_receiveFlag characteristic any message to finish the transfer
 * 7. you will be notify by FileErrorMsg characteristic with "done"
 * 
 * **Range:**  
 * accepted values are a string of bytes of no more than 512 bytes length  
 * 
 */
class FilesCallbacks_receive : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        if (receiveFlag)
        {
            std::string rxData = characteristic->getValue();
            int len = rxData.length();
            if (len > 0)
            {
                size_t lens = rxData.copy(pointerB, len, 0);
                pointerB = pointerB + lens;
                bufferSize += lens;
                if (bufferSize >= 9000){
                    pointerB = buffer;
                    while (readingSDFile){
                        delay(1);
                    }
                    readingSDFile = true;
                    fileReceive.write(buffer, bufferSize);
                    readingSDFile = false;
                    bufferSize = 0;
                }
                characteristic->setValue("1");
                characteristic->notify();
            }
            
        }
    }
};

/**
 * @brief **[BLE Command]** This class is used to create and object to be called when you write the sendFlag characteristic.
 * 
 * sendFlag characteristic is used to start the transfer of a file from the SD card to the BLE Client.
 * 
 * **Properties:**  
 * - Write
 * 
 * **UUID: fcbffdaa-2af1-11eb-adc1-0242ac120002**
 * 
 * **Usage:**  
 * to read a file from the SD card you have to read chunks of bytes corresponding to the desired file, to do it  follow the next steps.
 * 1. write to this characteristic the name of the file you want to read.
 * 2. wait to be notified by the fileErrorMsg characteristic and read it (see below what response you can receive).
 * 3. read the FilesCallbacks_send characteristic to get the bytes of the desired file.
 * 4. repeat step 3 until you read less than 512 bytes. when you read less than 512 bytes means that you have read all the bytes of the file.
 * 
 * 
 * **Range:**  
 * accepted values are a name, for example, "Old Path.txt" to start the transmition of this file.  
 * 
 * @note you may will be notified by fileErrorMsg characteristic with some of this massage:
 * - "ok" means the file is ready to be read.
 * - "error= -1" the desired file does not exist.
 */
class FilesCallbacks_sendFlag : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        if (!sendFlag){
            std::string rxData = characteristic->getValue();
            
            String name = rxData.c_str();
            #ifdef DEBUGGING_BLUETOOTH
                Serial.print("Begin sending... ");
                Serial.println(name);
            #endif
            while (readingSDFile){
                delay(1);
            }
            readingSDFile = true;
            if (!sdExists(name)){
                #ifdef DEBUGGING_BLUETOOTH
                    Serial.println("no existe el archivo");
                #endif
                fileCharacteristic_errorMsg->setValue("error= -1"); //archivo no existe
                fileCharacteristic_errorMsg->notify();
                readingSDFile = false;
                return;
            }
            fileSend = SD.open(name, FILE_READ);
            if (!fileSend)
            {
                fileCharacteristic_errorMsg->setValue("error= -2"); //file cannot be opened
                fileCharacteristic_errorMsg->notify();
                #ifdef DEBUGGING_BLUETOOTH
                    Serial.println("error al abrir el archivo"); //file cannot be opened
                #endif
                readingSDFile = false;
                return;
            }
            readingSDFile = false;
            #ifdef DEBUGGING_BLUETOOTH
                Serial.println("Enviando archivo...");
            #endif
            /*uint8_t data[CHUNKSIZE];
            int dataSize = fileSend.read(data, CHUNKSIZE);
            readingSDFile = false;
            fileCharacteristic_send->setValue(data,dataSize);
            //fileCharacteristic_send->notify();*/

            sendFlag = true;
            //pauseModeGlobal = true;
            
            fileCharacteristic_errorMsg->setValue("ok");
            fileCharacteristic_errorMsg->notify();
        }
        else{
            #ifdef DEBUGGING_BLUETOOTH
                Serial.println("sendflag puesta a false");
            #endif
            sendFlag = false;
            fileSend.close();
        }
    }
};

/**
 * @brief **[BLE Command]** This class is used to create and object to be called when you write the sendFlag characteristic.
 * 
 * sendFlag characteristic is used to start the transfer of a file from the SD card to the BLE Client.
 * 
 * **Properties:**  
 * - Write
 * - notify
 * 
 * **UUID: fcbffe72-2af1-11eb-adc1-0242ac120002**
 * 
 * **Usage:**  
 * to read a file from the SD card you have to read chunks of bytes corresponding to the desired file, to do it  follow the next steps.
 * 1. write to FilesCallbacks_sendFlag characteristic the name of the file you want to read.
 * 2. wait to be notified by the fileErrorMsg characteristic and read it.
 * 3. read this characteristic to get the bytes of the desired file.
 * 4. repeat step 3 until you read less than 512 bytes. when you read less than 512 bytes means that you have read all the bytes of the file.
 * 
 * 
 * **Range:**  
 * accepted values are a name, for example, "Old Path.txt" to start the transmition of this file.  
 */
class FilesCallbacks_send : public BLECharacteristicCallbacks
{
    void onRead(BLECharacteristic *characteristic)
    {
        if (sendFlag)
        {
            uint8_t data[CHUNKSIZE];
            while (readingSDFile){
                delay(1);
            }
            readingSDFile = true;
            int dataSize = fileSend.read(data, CHUNKSIZE);
            readingSDFile = false;
            #ifdef DEBUGGING_BLUETOOTH
                Serial.print("data sent: ");
                Serial.print(dataSize);
                Serial.println(" bytes");
            #endif
            if (dataSize == 0){
                characteristic->setValue("");
            }else {
                characteristic->setValue(data,dataSize);
            }
            if(dataSize < CHUNKSIZE){
                fileSend.close();
                sendFlag = false;
                #ifdef DEBUGGING_BLUETOOTH
                    Serial.println("done");
                #endif
            }
        }
    }

};

/**
 * @brief **[BLE Command]** This class is used to create and object to be called when you write to the checkFile characteristic.
 * 
 * checkFile characteristic is used to check if a file exists in the sd card.
 * 
 * **Properties:**  
 * - Write
 * 
 * **UUID: fcbffb52-2af1-11eb-adc1-0242ac120002**
 * 
 * **Usage:**  
 * if you want to check if a file exists in the SD card just write to this characteritic the file name you want to check and wait to be notified by fileErroMsg (fcbffce2-2af1-11eb-adc1-0242ac120002).
 * to know if the file exists read fileErroMsg characteristic and if the response is 1 the file exists and if it is 0 file doesn’t exist.
 * 
 * **Range:**  
 * accepted values are a name, for example, "Old Path.txt", "dogs.txt", "geometries.playlist" and so on.
 * 
 * **Response: **  
 * you can receive one of these responses after you write to this characteristic.
 * - "0" means the file does NOT exists
 * - "1" means the file exists*.  
 * 
 * 
 * @note all the reponses has to be read from the fileErroMsg characteristic after being notified by it.
 * @warning * if the file name starts with "Sandsara-trackNumber-" you will receive "1" even if your exact file name
 * does not exists in the sd card but it exists with another extension, for example, if in the sd card exists the file called "Sandsara-trackNumber-011.txt"
 * and you ask for "Sandsara-trackNumber-011.bin" you will receive "1" as a response even if "Sandsara-trackNumber-011.bin" does no exists,
 * but "Sandsara-trackNumber-011.txt" does.
 * 
 */
class FilesCallbacks_checkFile : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxData = characteristic->getValue();
        String filename = rxData.c_str();

        if (!sdExists(filename)){
            if (filename.indexOf(FILENAME_BASE) == 0){
                int indexDot = filename.indexOf(".");
                String nameBase = filename.substring(0,indexDot);
                if (indexDot < 0){
                    #ifdef DEBUGGING_BLUETOOTH
                        Serial.print("the file does NOT exist: ");
                        Serial.println(filename);
                    #endif
                    fileCharacteristic_errorMsg->setValue("0");
                    fileCharacteristic_errorMsg->notify();
                    return;
                }
                if (sdExists(nameBase + ".bin")){
                    #ifdef DEBUGGING_BLUETOOTH
                        Serial.print("the file exists but with .bin extension: ");
                        Serial.println(filename);
                    #endif
                    fileCharacteristic_errorMsg->setValue("1");
                    fileCharacteristic_errorMsg->notify();
                    return;
                }
                if (sdExists(nameBase + ".txt")){
                    #ifdef DEBUGGING_BLUETOOTH
                        Serial.print("the file exists but with .txt extension: ");
                        Serial.println(filename);
                    #endif
                    fileCharacteristic_errorMsg->setValue("1");
                    fileCharacteristic_errorMsg->notify();
                    return;
                }
                if (sdExists(nameBase + ".thr")){
                    #ifdef DEBUGGING_BLUETOOTH
                        Serial.print("the file exists but with .thr extension: ");
                        Serial.println(filename);
                    #endif
                    fileCharacteristic_errorMsg->setValue("1");
                    fileCharacteristic_errorMsg->notify();
                    return;
                }
            }
            #ifdef DEBUGGING_BLUETOOTH
                Serial.print("the file does NOT exist: ");
                Serial.println(filename);
            #endif
            fileCharacteristic_errorMsg->setValue("0");
            fileCharacteristic_errorMsg->notify();
            return;
        }
        #ifdef DEBUGGING_BLUETOOTH
            Serial.print("the file exists: ");
            Serial.println(filename);
        #endif
        fileCharacteristic_errorMsg->setValue("1");
        fileCharacteristic_errorMsg->notify();
    }

    #ifdef DEBUGGING_BLUETOOTH
        void onRead(BLECharacteristic *characteristic)
        {
            std::string rxValue = characteristic->getValue();
            String value = rxValue.c_str();
            Serial.print("READ checkFile: ");
            Serial.println(value);
        }
    #endif
};

/**
 * @brief **[BLE Command]** This class is used to create and object to be called when you write to the deleteFile characteristic.
 * 
 * deleteFile characteristic is used to delete a file in the sd card.
 * 
 * **Properties:**  
 * - Write
 * 
 * **UUID: fcbffc24-2af1-11eb-adc1-0242ac120002**
 * 
 * **Usage:**  
 * if you want to delete a file in the SD card just write to this characteritic the file name you want to delete.
 * to verify if the file was deleted read the fileErrorMsg characteristic for the response.
 * 
 * **Range:**  
 * accepted values are a name, for example, "Old Path.txt", "dogs.txt", "geometries.playlist" and so on.
 * 
 * **Response: **  
 * you can receive one of these responses after you write to this characteristic.
 * - "0" means the file does NOT exists
 * - "1" means the file was deleted.  
 * 
 * 
 * @note all the reponses has to be read from the fileErroMsg characteristic after being notified by it.
 */
class FilesCallbacks_deleteFile : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxData = characteristic->getValue();
        String filename = rxData.c_str();

        if (!sdRemove(filename)){
            #ifdef DEBUGGING_BLUETOOTH
                Serial.print("no se pudo eliminar el archivo : ");
                Serial.println(filename);
            #endif
            fileCharacteristic_errorMsg->setValue("0");
            fileCharacteristic_errorMsg->notify();
            return;
        }
        #ifdef DEBUGGING_BLUETOOTH
            Serial.print("se elimino el archivo : ");
            Serial.println(filename);
        #endif
        fileCharacteristic_errorMsg->setValue("1");
        fileCharacteristic_errorMsg->notify();
    }
    
    #ifdef DEBUGGING_BLUETOOTH
        void onRead(BLECharacteristic *characteristic)
        {
            std::string rxValue = characteristic->getValue();
            String value = rxValue.c_str();
            Serial.print("READ deleteFile: ");
            Serial.println(value);
        }
    #endif
};

//===========================Callbacks for general config==========================
//=================================================================================


class generalCallbacks_name : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxData = characteristic->getValue();
        String bluetoothName = rxData.c_str();
        if (bluetoothName.length() > MAX_CHARACTERS_BTNAME){
            generalCharacteristic_errorMsg->setValue("error=  -1");
            generalCharacteristic_errorMsg->notify();
            #ifdef DEBUGGING_BLUETOOTH
                Serial.println("Numero maximo de characteres execedido");
            #endif
            return;
        }
        bluetoothNameGlobal = bluetoothName;
        romSetBluetoothName(bluetoothNameGlobal);
        #ifdef DEBUGGING_BLUETOOTH
            Serial.print("WRITE sandsaraName: ");
            Serial.println(bluetoothName);
        #endif
        generalCharacteristic_errorMsg->setValue("ok");
        generalCharacteristic_errorMsg->notify();
    }

    #ifdef DEBUGGING_BLUETOOTH
        void onRead(BLECharacteristic *characteristic)
        {
            std::string rxValue = characteristic->getValue();
            String value = rxValue.c_str();
            Serial.print("READ SandsaraName: ");
            Serial.println(value);
        }
    #endif
};

class generalCallbacks_pause : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        pauseModeGlobal = true;
        Bluetooth::setStatus(MODE_PAUSE);
        #ifdef DEBUGGING_BLUETOOTH
            Serial.print("WRITE pause");
        #endif
        generalCharacteristic_errorMsg->setValue("ok");
        generalCharacteristic_errorMsg->notify();
    }

    #ifdef DEBUGGING_BLUETOOTH
        void onRead(BLECharacteristic *characteristic)
        {
            std::string rxValue = characteristic->getValue();
            String value = rxValue.c_str();
            Serial.print("READ pause: ");
            Serial.println(value);
        }
    #endif
};

class generalCallbacks_play : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        pauseModeGlobal = false;
        suspensionModeGlobal = false;
        Bluetooth::setStatus(MODE_PLAY);
        #ifdef DEBUGGING_BLUETOOTH
            Serial.print("WRITE play");
        #endif
        generalCharacteristic_errorMsg->setValue("ok");
        generalCharacteristic_errorMsg->notify();
    }

    #ifdef DEBUGGING_BLUETOOTH
        void onRead(BLECharacteristic *characteristic)
        {
            std::string rxValue = characteristic->getValue();
            String value = rxValue.c_str();
            Serial.print("READ play: ");
            Serial.println(value);
        }
    #endif
};

class generalCallbacks_Sleep : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        suspensionModeGlobal = true;
        pauseModeGlobal = false;
        Bluetooth::setStatus(MODE_SLEEP);
        #ifdef DEBUGGING_BLUETOOTH
            Serial.println("WRITE sleep");
        #endif
        generalCharacteristic_errorMsg->setValue("ok");
        generalCharacteristic_errorMsg->notify();
    }
    #ifdef DEBUGGING_BLUETOOTH
        void onRead(BLECharacteristic *characteristic)
        {
            std::string rxValue = characteristic->getValue();
            String value = rxValue.c_str();
            Serial.println("READ Sleep: ");
            Serial.println(value);
        }
    #endif
};

class generalCallbacks_speed : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string rxData = characteristic->getValue();
        String value = rxData.c_str();
        int speed = value.toInt();
        #ifdef DEBUGGING_BLUETOOTH
            Serial.print("WRITE speedball : ");
            Serial.println(speed);
        #endif
        characteristic->setValue(String(speed).c_str());
        //remap the speed acoording to the range of the ball speed
        speed = map(speed,MIN_SLIDER_MSPEED,MAX_SLIDER_MSPEED,MIN_SPEED_MOTOR,MAX_SPEED_MOTOR);
        if (speed > MAX_SPEED_MOTOR || speed < MIN_SPEED_MOTOR){
            generalCharacteristic_errorMsg->setValue("error= -2");
            generalCharacteristic_errorMsg->notify();
            #ifdef DEBUGGING_BLUETOOTH
                Serial.println("ballSpeed out of range");
            #endif
            return;
        }
        // Sandsara.setSpeed(speed);
        // romSetSpeedMotor(speed);
        // speedChangedMain = true;
        
        generalCharacteristic_errorMsg->setValue("ok");
        generalCharacteristic_errorMsg->notify();
    }

    #ifdef DEBUGGING_BLUETOOTH
        void onRead(BLECharacteristic *characteristic)
        {
            std::string rxValue = characteristic->getValue();
            String value = rxValue.c_str();
            Serial.print("READ speed: ");
            Serial.println(value);
        }
    #endif
};


class generalCallbacks_restart : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        generalCharacteristic_errorMsg->setValue("ok");
        generalCharacteristic_errorMsg->notify();
        rebootWithMessage("Reiniciando por medio de callback restart");
    }
};

class generalCallbacks_factoryReset : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        for (int i = 0; i < 512; i++){
            EEPROM.write(i, -1);
        }
        EEPROM.commit();
        generalCharacteristic_errorMsg->setValue("ok");
        generalCharacteristic_errorMsg->notify();
        delay(1000);
        rebootWithMessage("Se hiso reset de fabrica, Reiniciando...");
    }
};

class generalCallbacks_status: public BLECharacteristicCallbacks {
    #ifdef DEBUGGING_BLUETOOTH
        void onRead(BLECharacteristic *characteristic){
            Serial.print("READ status: ");
            Serial.println(characteristic->getValue().c_str());
        }
    #endif
};

class generalCallbacks_calibrating: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *characteristic){
        std::string rxData = characteristic->getValue();
        String value = rxData.c_str();
        int calibrationStatus = value.toInt();

        if (calibrationStatus > 0){
            romSetIntermediateCalibration(true);
            intermediateCalibration = true;
        }
        else {
            romSetIntermediateCalibration(false);
            intermediateCalibration = false;
        }
    }
};

Bluetooth::Bluetooth(){

}

int Bluetooth::init(String name){
    BLEDevice::init(name.c_str());
    BLEServer *pServer = BLEDevice::createServer();    
    BLEService *pServiceLedConfig = pServer->createService(BLEUUID(SERVICE_UUID1), 40);
    //BLEService *pServicePath = pServer->createService(BLEUUID(SERVICE_UUID2), 30);
    //BLEService *pServiceSphere = pServer->createService(BLEUUID(SERVICE_UUID3), 30);
    BLEService *pServicePlaylist = pServer->createService(BLEUUID(SERVICE_UUID4), 30);
    BLEService *pServiceGeneralConfig = pServer->createService(BLEUUID(SERVICE_UUID5), 30);
    BLEService *pServiceFile = pServer->createService(BLEUUID(SERVICE_UUID6), 30);
    pServer->setCallbacks(new bleServerCallback());
    //====Characteristics for LEDs configuration====
    
    ledCharacteristic_indexPalette = pServiceLedConfig->createCharacteristic(
        CHARACTERISTIC_UUID_SELECTEDPALETTE,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE);
    ledCharacteristic_speed = pServiceLedConfig->createCharacteristic(
        CHARACTERISTIC_UUID_LEDSPEED,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE);
    ledCharacteristic_cycleMode = pServiceLedConfig->createCharacteristic(
        CHARACTERISTIC_UUID_CYCLEMODE,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE);
    ledCharacteristic_direction = pServiceLedConfig->createCharacteristic(
        CHARACTERISTIC_UUID_DIRECTION,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE);
    ledCharacteristic_brightness = pServiceLedConfig->createCharacteristic(
        CHARACTERISTIC_UUID_BRIGHTNESS,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE);
    ledCharacteristic_amountColors = pServiceLedConfig->createCharacteristic(
        CHARACTERISTIC_UUID_AMOUNTCOLORS,
            BLECharacteristic::PROPERTY_READ);
    ledCharacteristic_positions = pServiceLedConfig->createCharacteristic(
        CHARACTERISTIC_UUID_POSITIONS,
            BLECharacteristic::PROPERTY_READ);
    ledCharacteristic_red = pServiceLedConfig->createCharacteristic(
        CHARACTERISTIC_UUID_RED,
            BLECharacteristic::PROPERTY_READ);
    ledCharacteristic_green = pServiceLedConfig->createCharacteristic(
        CHARACTERISTIC_UUID_GREEN,
            BLECharacteristic::PROPERTY_READ);
    ledCharacteristic_blue = pServiceLedConfig->createCharacteristic(
        CHARACTERISTIC_UUID_BLUE,
            BLECharacteristic::PROPERTY_READ);
    ledCharacteristic_update = pServiceLedConfig->createCharacteristic(
        CHARACTERISTIC_UUID_UPDATECPALETTE,
        BLECharacteristic::PROPERTY_WRITE);
    ledCharacteristic_errorMsg = pServiceLedConfig->createCharacteristic(
        CHARACTERISTIC_UUID_MSGERRORLEDS,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_NOTIFY);
    ledCharacteristic_errorMsg->addDescriptor(new BLE2902());
    
    ledCharacteristic_speed->setCallbacks(new speedLedCallbacks());
    ledCharacteristic_cycleMode->setCallbacks(new cycleModeCallbacks());
    ledCharacteristic_direction->setCallbacks(new directionCallbacks());
    ledCharacteristic_brightness->setCallbacks(new setBrightnessCallbacks());
    ledCharacteristic_amountColors->setCallbacks(new genericCallbacks());
    ledCharacteristic_positions->setCallbacks(new genericCallbacks);
    ledCharacteristic_red->setCallbacks(new genericCallbacks);
    ledCharacteristic_green->setCallbacks(new genericCallbacks);
    ledCharacteristic_blue->setCallbacks(new genericCallbacks);
    ledCharacteristic_update->setCallbacks(new CallbacksToUpdate());
    ledCharacteristic_indexPalette->setCallbacks(new selectedPaletteCallbacks());
    setLedSpeed(periodLedsGlobal);
    if(romGetIncrementIndexPallete()){
        ledCharacteristic_cycleMode->setValue("1");}
    else{
        ledCharacteristic_cycleMode->setValue("0");}
    if(romGetLedsDirection()){
        ledCharacteristic_direction->setValue("1");}
    else{
        ledCharacteristic_direction->setValue("0");}
    ledCharacteristic_indexPalette->setValue(String(romGetPallete()).c_str());

    setRed();
    setGreen();
    setBlue();
    setPositions();
    setAmountOfColors();

    //====Characteristics for playlist configuration====
    playlistCharacteristic_name = pServicePlaylist->createCharacteristic(
        PLAYLIST_UUID_NAME,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE);

    playlistCharacteristic_pathAmount = pServicePlaylist->createCharacteristic(
        PLAYLIST_UUID_PATHAMOUNT,
        BLECharacteristic::PROPERTY_READ);

    playlistCharacteristic_pathName = pServicePlaylist->createCharacteristic(
        PLAYLIST_UUID_PATHNAME,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE);

    playlistCharacteristic_pathPosition = pServicePlaylist->createCharacteristic(
        PLAYLIST_UUID_PATHPOSITION,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE);

    playlistCharacteristic_addPath = pServicePlaylist->createCharacteristic(
        PLAYLIST_UUID_ADDPATH,
            BLECharacteristic::PROPERTY_WRITE);

    playlistCharacteristic_mode = pServicePlaylist->createCharacteristic(
        PLAYLIST_UUID_MODE,
            BLECharacteristic::PROPERTY_WRITE);

    playlistCharacteristic_progress = pServicePlaylist->createCharacteristic(
        PLAYLIST_UUID_PATHPROGRESS,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_NOTIFY);
    playlistCharacteristic_progress->addDescriptor(new BLE2902());
    
    playlistCharacteristic_errorMsg = pServicePlaylist->createCharacteristic(
        PLAYLIST_UUID_ERRORMSG,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_NOTIFY);
    playlistCharacteristic_errorMsg->addDescriptor(new BLE2902());

    playlistCharacteristic_name->setCallbacks(new playlistCallbacks_name());
    playlistCharacteristic_pathName->setCallbacks(new playlistCallbacks_pathName());
    playlistCharacteristic_pathPosition->setCallbacks(new playlistCallbacks_pathPosition());
    playlistCharacteristic_addPath->setCallbacks(new playlistCallbacks_addPath());
    playlistCharacteristic_mode->setCallbacks(new playlistCallbacks_mode());

    //====Characteristics for General configuration====
    generalCharacteristic_version = pServiceGeneralConfig->createCharacteristic(
        GENERAL_UUID_VERSION,
        BLECharacteristic::PROPERTY_READ);

    generalCharacteristic_name = pServiceGeneralConfig->createCharacteristic(
        GENERAL_UUID_NAME,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE);

    generalCharacteristic_status= pServiceGeneralConfig->createCharacteristic(
        GENERAL_UUID_STATUS,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_NOTIFY);
    generalCharacteristic_status->addDescriptor(new BLE2902());

    generalCharacteristic_pause = pServiceGeneralConfig->createCharacteristic(
        GENERAL_UUID_PAUSE,
            BLECharacteristic::PROPERTY_WRITE);

    generalCharacteristic_play = pServiceGeneralConfig->createCharacteristic(
        GENERAL_UUID_PLAY,
            BLECharacteristic::PROPERTY_WRITE);

    generalCharacteristic_sleep = pServiceGeneralConfig->createCharacteristic(
        GENERAL_UUID_SLEEP,
            BLECharacteristic::PROPERTY_WRITE);

    generalCharacteristic_speed = pServiceGeneralConfig->createCharacteristic(
        GENERAL_UUID_SPEED,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE);

    generalCharacteristic_restart = pServiceGeneralConfig->createCharacteristic(
        GENERAL_UUID_RESTART,
            BLECharacteristic::PROPERTY_WRITE);

    generalCharacteristic_factoryReset = pServiceGeneralConfig->createCharacteristic(
        GENERAL_UUID_FACTORYRESET,
            BLECharacteristic::PROPERTY_WRITE);
    
    generalCharacteristic_calibration = pServiceGeneralConfig->createCharacteristic(
        GENERAL_UUID_CALIBRATION,
            BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_READ);

    generalCharacteristic_errorMsg = pServiceGeneralConfig->createCharacteristic(
        GENERAL_UUID_ERRORMSG,
            BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_NOTIFY);
    generalCharacteristic_errorMsg->addDescriptor(new BLE2902());

    generalCharacteristic_name->setCallbacks(new generalCallbacks_name());
    generalCharacteristic_pause->setCallbacks(new generalCallbacks_pause());
    generalCharacteristic_play->setCallbacks(new generalCallbacks_play());
    generalCharacteristic_sleep->setCallbacks(new generalCallbacks_Sleep());
    generalCharacteristic_speed->setCallbacks(new generalCallbacks_speed());
    generalCharacteristic_restart->setCallbacks(new generalCallbacks_restart());
    generalCharacteristic_factoryReset->setCallbacks(new generalCallbacks_factoryReset());
    generalCharacteristic_status->setCallbacks(new generalCallbacks_status());
    generalCharacteristic_calibration->setCallbacks(new generalCallbacks_calibrating());

    //====Characteristics for File configuration====
    fileCharacteristic_receiveFlag = pServiceFile->createCharacteristic(
        FILE_UUID_RECEIVEFLAG,
            BLECharacteristic::PROPERTY_WRITE);

    fileCharacteristic_receive = pServiceFile->createCharacteristic(
        FILE_UUID_RECEIVE,
            BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_NOTIFY);
    fileCharacteristic_receive->addDescriptor(new BLE2902());

    fileCharacteristic_exists = pServiceFile->createCharacteristic(
        FILE_UUID_EXISTS,
            BLECharacteristic::PROPERTY_WRITE);

    fileCharacteristic_delete = pServiceFile->createCharacteristic(
        FILE_UUID_DELETE,
            BLECharacteristic::PROPERTY_WRITE);

    fileCharacteristic_sendFlag = pServiceFile->createCharacteristic(
        FILE_UUID_SENDFLAG,
            BLECharacteristic::PROPERTY_WRITE);

    fileCharacteristic_send = pServiceFile->createCharacteristic(
        FILE_UUID_SEND,
            BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_NOTIFY);
    fileCharacteristic_send->addDescriptor(new BLE2902());

    fileCharacteristic_errorMsg = pServiceFile->createCharacteristic(
        FILE_UUID_ERRORMSG,
            BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_NOTIFY);
    fileCharacteristic_errorMsg->addDescriptor(new BLE2902());

    fileCharacteristic_receiveFlag->setCallbacks(new FilesCallbacks_receiveFlag());
    fileCharacteristic_receive->setCallbacks(new FilesCallbacks_receive());
    fileCharacteristic_exists->setCallbacks(new FilesCallbacks_checkFile());
    fileCharacteristic_delete->setCallbacks(new FilesCallbacks_deleteFile());
    fileCharacteristic_send->setCallbacks(new FilesCallbacks_send());
    fileCharacteristic_sendFlag->setCallbacks(new FilesCallbacks_sendFlag());

    //ledCharacteristic_speed->addDescriptor(new BLE2902());
    
    pServiceLedConfig->start();
    pServicePlaylist->start();
    pServiceGeneralConfig->start();
    pServiceFile->start();
    //pService3->start();


    // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID1);
    pAdvertising->addServiceUUID(SERVICE_UUID2);
    pAdvertising->addServiceUUID(SERVICE_UUID3);
    pAdvertising->addServiceUUID(SERVICE_UUID4);
    pAdvertising->addServiceUUID(SERVICE_UUID5);
    pAdvertising->addServiceUUID(SERVICE_UUID6);
    //pAdvertising->addServiceUUID(SERVICE_UUID8);

    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    #ifdef DEBUGGING_DATA
        Serial.println("BLE is anable");
    #endif
    
    return 0;
}

/**
 * Las siguientes funciones se encargan de calcular el hash por el metodo de MD5.
 */
typedef union uwb {
    unsigned w;
    unsigned char b[4];
} MD5union;

typedef unsigned DigestArray[4];

static unsigned func0(unsigned abcd[])
{
    return (abcd[1] & abcd[2]) | (~abcd[1] & abcd[3]);
}

static unsigned func1(unsigned abcd[])
{
    return (abcd[3] & abcd[1]) | (~abcd[3] & abcd[2]);
}

static unsigned func2(unsigned abcd[])
{
    return abcd[1] ^ abcd[2] ^ abcd[3];
}

static unsigned func3(unsigned abcd[])
{
    return abcd[2] ^ (abcd[1] | ~abcd[3]);
}

typedef unsigned (*DgstFctn)(unsigned a[]);

static unsigned *calctable(unsigned *k)
{
    double s, pwr;
    int i;

    pwr = pow(2.0, 32);
    for (i = 0; i < 64; i++)
    {
        s = fabs(sin(1.0 + i));
        k[i] = (unsigned)(s * pwr);
    }
    return k;
}

static unsigned rol(unsigned r, short N)
{
    unsigned mask1 = (1 << N) - 1;
    return ((r >> (32 - N)) & mask1) | ((r << N) & ~mask1);
}

static unsigned *MD5Hash(uint8_t *msg, int mlen)
{
    static DigestArray h0 = {0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476};
    static DgstFctn ff[] = {&func0, &func1, &func2, &func3};
    static short M[] = {1, 5, 3, 7};
    static short O[] = {0, 1, 5, 0};
    static short rot0[] = {7, 12, 17, 22};
    static short rot1[] = {5, 9, 14, 20};
    static short rot2[] = {4, 11, 16, 23};
    static short rot3[] = {6, 10, 15, 21};
    static short *rots[] = {rot0, rot1, rot2, rot3};
    static unsigned kspace[64];
    static unsigned *k;

    static DigestArray h;
    DigestArray abcd;
    DgstFctn fctn;
    short m, o, g;
    unsigned f;
    short *rotn;
    union {
        unsigned w[16];
        char b[64];
    } mm;
    int os = 0;
    int grp, grps, q, p;
    unsigned char *msg2;

    if (k == NULL)
        k = calctable(kspace);

    for (q = 0; q < 4; q++)
        h[q] = h0[q];

    {
        grps = 1 + (mlen + 8) / 64;
        msg2 = (unsigned char *)malloc(64 * grps);
        memcpy(msg2, (unsigned char *)msg, mlen);
        msg2[mlen] = (unsigned char)0x80;
        q = mlen + 1;
        while (q < 64 * grps)
        {
            msg2[q] = 0;
            q++;
        }
        {
            MD5union u;
            u.w = 8 * mlen;
            q -= 8;
            memcpy(msg2 + q, &u.w, 4);
        }
    }

    for (grp = 0; grp < grps; grp++)
    {
        memcpy(mm.b, msg2 + os, 64);
        for (q = 0; q < 4; q++)
            abcd[q] = h[q];
        for (p = 0; p < 4; p++)
        {
            fctn = ff[p];
            rotn = rots[p];
            m = M[p];
            o = O[p];
            for (q = 0; q < 16; q++)
            {
                g = (m * q + o) % 16;
                f = abcd[1] + rol(abcd[0] + fctn(abcd) + k[q + 16 * p] + mm.w[g], rotn[q % 4]);

                abcd[0] = abcd[3];
                abcd[3] = abcd[2];
                abcd[2] = abcd[1];
                abcd[1] = f;
            }
        }
        for (p = 0; p < 4; p++)
            h[p] += abcd[p];
        os += 64;
    }
    free(msg2); //THIS IS IMPORTANT
    return h;
}

String GetMD5String(uint8_t *msg, int mlen)
{
    String str;
    int j;
    unsigned *d = MD5Hash(msg, mlen);
    MD5union u;
    for (j = 0; j < 4; j++)
    {
        u.w = d[j];
        char s[9];
        sprintf(s, "%02x%02x%02x%02x", u.b[0], u.b[1], u.b[2], u.b[3]);
        str += s;
    }
    return str;
}

/**
 * Hasta aqui terminan las funciones para MD5
 */

/**
 * @brief Actualiza el firmware.
 * @return Uno de los siguientes numeros.
 * -4, No se pudo finalizar la actualizacion
 * -5, No hay suficiente espacio para el OTA.
 * -6, Ocurrio un error al actualizar el firmware
 */
int performUpdate(Stream &updateSource, size_t updateSize)
{
    if (Update.begin(updateSize))
    {
        size_t written = Update.writeStream(updateSource);
        if (written == updateSize)
        {
            #ifdef DEBUGGING_DATA
                Serial.println("Written : " + String(written) + " successfully");
            #endif
        }
        else
        {
            #ifdef DEBUGGING_DATA
                Serial.println("Written only : " + String(written) + "/" + String(updateSize) + ". Retry?");
            #endif
            
        }
        if (Update.end())
        {
            Serial.println("OTA done!");
            if (Update.isFinished())
            {
                #ifdef DEBUGGING_DATA
                    Serial.println("Update successfully completed. Rebooting.");
                #endif
                
                return 1;
            }
            else
            {
                #ifdef DEBUGGING_DATA
                    Serial.println("Update not finished? Something went wrong!");
                #endif
                return -4;
            }
        }
        else
        {
            #ifdef DEBUGGING_DATA
                Serial.println("Error Occurred. Error #: " + String(Update.getError()));
            #endif
            return -6;
        }
    }
    else
    {
        #ifdef DEBUGGING_DATA
            Serial.println("Not enough space to begin OTA");
        #endif
        return -5;
    }
}

/**
 * @brief Revisa si el archivo es valido y si lo es actualiza el firmware
 * @return Uno de los siguientes numeros
 *  1, Se actualizo el firmware
 * -1, Es un directorio
 * -2, El archivo esta vacio
 * -3, El archivo no se pudo abrir.
 * -4, No se pudo finalizar la actualizacion
 * -5, No hay suficiente espacio para el OTA.
 * -6, Ocurrio un error al actualizar el firmware
 */
int updateFromFS(SdFat &fs, String name)
{
    int errorCode;
    File updateBin = fs.open(name);
    if (updateBin)
    {
        if (updateBin.isDirectory())
        {
            Serial.println("Error, update.bin is not a file");
            updateBin.close();
            return -1;
        }

        size_t updateSize = updateBin.size();

        if (updateSize > 0)
        {
            Serial.println("Try to start update");
            errorCode = performUpdate(updateBin, updateSize);
            updateBin.close();
            sdRemove(name);
            return errorCode;
        }
        else
        {
            Serial.println("Error, file is empty");
            updateBin.close();
            return -2;
        }

        updateBin.close();
        sdRemove(name);
    }
    else
    {
        Serial.println("Could not load update.bin from sd root");
        return -3;
    }
}

/**
 * @brief Intenta actualizar el firmware
 * @return Un codigo para saber si ocurre un error a la hora de realizar la actualizacion.
 */
int programming(String name){
    int errorCode;
    errorCode = updateFromFS(SD, name);
    return errorCode;
}

/**
 * @brief Reinicia el Esp32 pero antes escribe un mensaje por Serial.
 * @return Un codigo para saber si ocurre un error a la hora de realizar la actualizacion.
 */
void rebootWithMessage(String reason){
    #ifdef DEBUGGING_DATA
        Serial.println(reason);
    #endif
    delay(2000);
    ESP.restart();
}

/**
 * @brief Convierte un string en un array
 * un string de la forma x1,x2,...,xn se convierte en un array [0]=x1, [1]=x1, ...,[n-1]=xn
 * @param str Es el string que se desea convertir.
 * @param array Es el array donde se van a guardar los valores del string
 * @param n Es el numero de elementos que tiene el string
 * @return Uno de los siguientes numeros
 * 0, Todo salio normal.
 * -1, No hay n elementos en el string
 */
int stringToArray(String str, uint8_t* array, int n){
    int i;
    for (i = 0; i<n ; i++){
        if (str.indexOf(",") < 0){
            *(array + i) = str.toInt();
            break;
        }
        *(array + i) = str.substring(0, str.indexOf(",")).toInt();
        str = str.substring(str.indexOf(",") + 1);
    }
    i++;
    if(i != n){
        return -1;
    }
    return 0;
}

void Bluetooth::setPlaylistName(String playlistName){
    if (playlistName.charAt(0) == '/'){
        playlistName.remove(0,1);
    }
    int index = playlistName.lastIndexOf('.');
    if (index > 0){
        playlistName.remove(index);
    }
    playlistCharacteristic_name->setValue(playlistName.c_str());
    #ifdef DEBUGGING_BLUETOOTH
        Serial.print("SET Playlist Name: ");
        Serial.println(playlistName.c_str());
    #endif
}
void Bluetooth::setPathAmount(int pathAmount){
    playlistCharacteristic_pathAmount->setValue(String(pathAmount).c_str());
    #ifdef DEBUGGING_BLUETOOTH
        Serial.print("SET pathAmount: ");
        Serial.println(String(pathAmount).c_str());
    #endif
}
void Bluetooth::setPathName(String pathName){
    playlistCharacteristic_pathName->setValue(pathName.c_str());
    #ifdef DEBUGGING_BLUETOOTH
        Serial.print("SET pathName: ");
        Serial.println(pathName.c_str());
    #endif
}
void Bluetooth::setPathPosition(int pathPosition){
    playlistCharacteristic_pathPosition->setValue(String(pathPosition).c_str());
    #ifdef DEBUGGING_BLUETOOTH
        Serial.print("SET pathPosition: ");
        Serial.println(String(pathPosition).c_str());
    #endif
}
void Bluetooth::setPlayMode(int mode){
    playlistCharacteristic_mode->setValue(String(mode).c_str());
    #ifdef DEBUGGING_BLUETOOTH
        Serial.print("SET PlaylistMode: ");
        Serial.println(String(mode).c_str());
    #endif
}
// void Bluetooth::setPathProgress(int progress){
//     playlistCharacteristic_progress->setValue(String(progress).c_str());
//     #ifdef DEBUGGING_BLUETOOTH
//         Serial.print("SET progress: ");
//         Serial.println(String(progress).c_str());
//     #endif
// }


void Bluetooth::setLedSpeed(int speed){
    speed = map(speed,MIN_PERIOD_LED,MAX_PERIOD_LED,MAX_SLIDER_LEDSPEED,MIN_SLIDER_LEDSPEED);
    if (speed > MAX_SLIDER_LEDSPEED){
        speed = MAX_SLIDER_LEDSPEED;
    }
    if (speed < MIN_SLIDER_LEDSPEED){
        speed = MIN_SLIDER_LEDSPEED;
    }
    ledCharacteristic_speed->setValue(String(speed).c_str());
    #ifdef DEBUGGING_BLUETOOTH
        Serial.print("SET ledSoeed: ");
        Serial.println(String(speed).c_str());
    #endif
}
void Bluetooth::setCycleMode(int cycleMode){
    ledCharacteristic_cycleMode->setValue(String(cycleMode).c_str());
    #ifdef DEBUGGING_BLUETOOTH
        Serial.print("SET cycleMode: ");
        Serial.println(String(cycleMode).c_str());
    #endif
}
void Bluetooth::setLedDirection(int ledDirection){
    ledCharacteristic_direction->setValue(String(ledDirection).c_str());
    #ifdef DEBUGGING_BLUETOOTH
        Serial.print("SET ledDirection: ");
        Serial.println(String(ledDirection).c_str());
    #endif
}
void Bluetooth::setBrightness(uint16_t brightness){
    brightness = map(brightness,0,255,MIN_SLIDER_BRIGHTNESS,MAX_SLIDER_BRIGHTNESS);
    ledCharacteristic_brightness->setValue(String(brightness).c_str());
    #ifdef DEBUGGING_BLUETOOTH
        Serial.print("SET brightness: ");
        Serial.println(String(brightness).c_str());
    #endif
}
void Bluetooth::setIndexPalette(int indexPalette){
    ledCharacteristic_indexPalette->setValue(String(indexPalette).c_str());
    #ifdef DEBUGGING_BLUETOOTH
        Serial.print("SET indexPalette: ");
        Serial.println(String(indexPalette).c_str());
    #endif
}

void Bluetooth::setVersion(String version){
    generalCharacteristic_version->setValue(version.c_str());
    #ifdef DEBUGGING_BLUETOOTH
        Serial.print("SET version: ");
        Serial.println(version.c_str());
    #endif
}
void Bluetooth::setName(String name){
    generalCharacteristic_name->setValue(name.c_str());
    #ifdef DEBUGGING_BLUETOOTH
        Serial.print("SET ble name: ");
        Serial.println(name.c_str());
    #endif
}
void Bluetooth::setStatus(int status){
    generalCharacteristic_status->setValue(String(status).c_str());
    generalCharacteristic_status->notify();
    #ifdef DEBUGGING_BLUETOOTH
        Serial.print("SET status: ");
        Serial.println(String(status).c_str());
    #endif
}
void Bluetooth::setMotorSpeed(int speed){
    speed = map(speed,MIN_SPEED_MOTOR,MAX_SPEED_MOTOR,MIN_SLIDER_MSPEED,MAX_SLIDER_MSPEED);
    if (speed > MAX_SLIDER_MSPEED){
        speed = MAX_SLIDER_MSPEED;
    }
    if (speed < MIN_SLIDER_MSPEED){
        speed = MIN_SLIDER_MSPEED;
    }
    generalCharacteristic_speed->setValue(String(speed).c_str());
    #ifdef DEBUGGING_BLUETOOTH
        Serial.print("SET motor Speed: ");
        Serial.println(String(speed).c_str());
    #endif
}
void Bluetooth::setPercentage(int percentage){
    if (percentage < 0 ){
        percentage = 0;
    }
    if (percentage > 100){
        percentage = 100;
    }
    playlistCharacteristic_progress->setValue(String(percentage).c_str());
    playlistCharacteristic_progress->notify();
    #ifdef DEBUGGING_BLUETOOTH
        Serial.print("SET percentage: ");
        Serial.println(String(percentage).c_str());
    #endif
}

void Bluetooth::setRed(){
    uint8_t numberOfColors = EEPROM.read(ADDRESSCUSTOMPALLETE_COLORS);
    if (numberOfColors > MAX_COLORSPERPALLETE || numberOfColors < 2){
        return;
    }
    uint8_t red[numberOfColors];
    String reds = "";
    for (int i = 0; i < numberOfColors; i++){
        red[i] = EEPROM.read(ADDRESSCUSTOMPALLETE_RED + i);
        reds.concat(red[i]);
        reds.concat(",");
    }
    reds.remove(reds.length() - 1);
    ledCharacteristic_red->setValue(reds.c_str());
    #ifdef DEBUGGING_BLUETOOTH
        Serial.print("SET red: ");
        Serial.println(reds.c_str());
    #endif
}
void Bluetooth::setGreen(){
    uint8_t numberOfColors = EEPROM.read(ADDRESSCUSTOMPALLETE_COLORS);
    if (numberOfColors > MAX_COLORSPERPALLETE || numberOfColors < 2){
        return;
    }
    uint8_t green[numberOfColors];
    String greens = "";
    for (int i = 0; i < numberOfColors; i++){
        green[i] = EEPROM.read(ADDRESSCUSTOMPALLETE_GREEN + i);
        greens.concat(green[i]);
        greens.concat(",");
    }
    greens.remove(greens.length() - 1);
    ledCharacteristic_green->setValue(greens.c_str());
    #ifdef DEBUGGING_BLUETOOTH
        Serial.print("SET green: ");
        Serial.println(greens.c_str());
    #endif
}
void Bluetooth::setBlue(){
    uint8_t numberOfColors = EEPROM.read(ADDRESSCUSTOMPALLETE_COLORS);
    if (numberOfColors > MAX_COLORSPERPALLETE || numberOfColors < 2){
        return;
    }
    uint8_t blue[numberOfColors];
    String blues = "";
    for (int i = 0; i < numberOfColors; i++){
        blue[i] = EEPROM.read(ADDRESSCUSTOMPALLETE_BLUE + i);
        blues.concat(blue[i]);
        blues.concat(",");
    }
    blues.remove(blues.length() - 1);
    ledCharacteristic_blue->setValue(blues.c_str());
    #ifdef DEBUGGING_BLUETOOTH
        Serial.print("SET blue: ");
        Serial.println(blues.c_str());
    #endif
}
void Bluetooth::setPositions(){
    uint8_t numberOfColors = EEPROM.read(ADDRESSCUSTOMPALLETE_COLORS);
    if (numberOfColors > MAX_COLORSPERPALLETE || numberOfColors < 2){
        return;
    }
    uint8_t position[numberOfColors];
    String positions = "";
    for (int i = 0; i < numberOfColors; i++){
        position[i] = EEPROM.read(ADDRESSCUSTOMPALLETE_POSITIONS + i);
        positions.concat(position[i]);
        positions.concat(",");
    }
    positions.remove(positions.length() - 1);
    ledCharacteristic_positions->setValue(positions.c_str());
    #ifdef DEBUGGING_BLUETOOTH
        Serial.print("SET pathPositions: ");
        Serial.println(positions.c_str());
    #endif
}
void Bluetooth::setAmountOfColors(){
    uint8_t numberOfColors = EEPROM.read(ADDRESSCUSTOMPALLETE_COLORS);
    String amount;
    amount.concat(numberOfColors);
    ledCharacteristic_amountColors->setValue(amount.c_str());
    #ifdef DEBUGGING_BLUETOOTH
        Serial.print("SET amount: ");
        Serial.println(amount.c_str());
    #endif
}

void Bluetooth::setCalibrationStatus(bool calibrationStatus){
    if (calibrationStatus){
        generalCharacteristic_calibration->setValue("1");
    }
    else{
        generalCharacteristic_calibration->setValue("0");
    }
}