#pragma once

//Speed
//#define millimeterSpeed 15;

#include "AccelStepper.h"
#include "MultiStepper.h"
#include "Bluetooth.h"
#include "config.h"

/**
 * @class Motors
 * @brief Se encarga del control de los motores
 * 
 * Esta clase se encarga de calcular los pasos necesarios para llegar a una determinada posicion. En esta clase
 * tambien se encuentran todas las matematicas con respecto a la cinematica y cinematica inversa de Sandsara.
 * @param microstepping almacena el valor de microstepping del motor
 * @param x_current alamacena la coordenada x de la posicion actual de Sandsara (en milimetros).
 * @param y_current alamacena la coordenada y de la posicion actual de Sandsara (en milimetros).
 * @param q1_current almacena el valor actual del angulo del primer eslabon de Sandsara (en radianes).
 * @param q2_current almacena el valor actual del angulo del segundo eslabon de Sandsara (en radianes).
 * @param zCurrent se utiliza como referencia del componente  modulo para los archivos thr (en milimetros).
 * @param thetaCurrent se utiliza como referencia del componente del angulo para los archivos thr (en radianes).
 * @param couplingAngle es un angulo que se utiliza para saber cuanto rotar un archivo.
 * @param constantMotorSpeed Si esta variable es true, los motores se moveran a una velocidad constante siempre, si es false la velocidad
 * dependera de la distancia recorrida de un punto a otro.
 */
class Motors {
    public:
        AccelStepper    stepper1;
        AccelStepper    stepper2;
        MultiStepper    steppers;
        int     microstepping;
        double  degrees_per_step;
        double  couplingAngle;
        double  x_current;
        double  y_current;
        double  q1_current;
        double  q2_current;
        double  millimeterSpeed = 15;
        double  _pathSpeed;
        bool    lastPoint;
        
    private:
        double  zCurrent;
        double  thetaCurrent;
        long    maxSpeed;        
        double  x_home;
        double  y_home;
        bool    constantMotorSpeed = false;
        double  realSpeed1,realSpeed2;
        double  realQ1, realQ2;
        bool    speedChanging = false;
        double  oldSpeed1 = 0, oldSpeed2 = 0;

        long    q1StepsBuffer[SAMPLES];
        long    q2StepsBuffer[SAMPLES];
        double  distanceBuffer[SAMPLES];
        double  pathSpeedBuffer[SAMPLES];
        double  timesBuffer[SAMPLES];
        long    maxStepsBuffer[SAMPLES];
        bool    fullBuffer = false;

        double  xBuffer[SAMPLES], yBuffer[SAMPLES];
        int     pointerBuffer = 0, cPointerBuffer = 0;
        int     xyPointer = 0, cxyPointer = 0;
        int     xyPointerBuffer[SAMPLES];
        
    public:
        Motors();
        void    moveTo(double x, double y, bool = false);
        void    init(double = 0,double = 0);
        double  getCurrentModule();
        double  getCurrentAngle();
        void    setZCurrent(double );
        void    setThetaCurrent(double );
        void    setSpeed(int );
        double  getSpeed();
        double  getZCurrent();
        double  getThetaCurrent();
        static  void    rotate(double& ,double& ,double );
        static  double  zPolar(double , double );
        static  double  thetaPolar(double , double );
        static  double  normalizeAngle(double );
        static  double  arcLength(double ,double , double);
        double  module(double , double , double , double );
        int     position();
        void    completePath();
        void    setRealQ1(double );
        void    setRealQ2(double );
        double  getRealQ1();
        double  getRealQ2();
        void    stopAndResetPositions();
        void    setRealSpeed1(double );
        void    setRealSpeed2(double ); 
        void    resetSpeeds();
        static void constrainXY(double& x, double& y);

        private :
        void    moveInterpolateTo(double, double, double, bool);
        void    moveSteps(long, long, double);
        //mathematics methods
        long    calculate_steps(double , double );
        void    calculate_line_equations();
        double  polarModule(double , double , double , double );
        void    updateVariablesToMovingThread();
        void    ik(double , double , double* , double* );
};