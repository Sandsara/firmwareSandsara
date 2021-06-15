#ifndef _CALIBMOTOR_H_
#define _CALIBMOTOR_H_

#include <Arduino.h>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include "MeanFilterLib.h"
#include <HardwareSerial.h>
#include <EEPROM.h>
#include "config.h"

/**
 * @class WorkingArea
 * @brief Se encarga de encontrar el origen del robot.
 * 
 * Esta clase realiza una calibracion de los motores para encotrar una posicion de origen, esto se consigue
 * con la lectura de unos sensores de efecto hall. La primera vez que se enciende Sandsara esta clase tambien se encarga
 * de calibrar los sensores hall con respecto a la polaridad de los imanes en los brazos de Sandsara.
 * 
 */
class WorkingArea{
    public:
        WorkingArea();
        void verificacion_cal();
    private:

    public:
        void prepareMotors();
        int findZeroPoint();
};

#endif
