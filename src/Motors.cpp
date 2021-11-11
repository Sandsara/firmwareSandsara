#include "Motors.h"
#include "AccelStepper.h"
#include "MultiStepper.h"
#include "Bluetooth.h"
#include "WorkingArea.h"
unsigned long timeMotor = 0;

int l1 = String(String(dataS[4]) + String(dataS[5])).toFloat();
int l2 = String(String(dataS[7]) + String(dataS[8])).toFloat();

double bigPulleySize = String(String(dataS[20])).toFloat();
double littlePulleySize = String(String(dataS[22])).toFloat();

int MAX_STEPS_PER_SECOND = String(String(dataS[74]) + String(dataS[75]) + String(dataS[76])).toFloat();
int MAX_CHARACTER_PER_LINE = String(String(dataS[78]) + String(dataS[79]) + String(dataS[80]) + String(dataS[81])).toFloat();

double m[no_picos * 2], b[no_picos * 2];
#ifdef IMPLEMENT_ACCELERATION
    double  maxPathSpeedGlobal;
    long    maxStepsGlobal;
    double  greaterValue(double , double );
    bool    lowerSpeed = false;
#endif

//====Extern varibales====
extern bool     productType;
extern bool     pauseModeGlobal;
extern bool     startMovement;
extern long     q1StepsGlobal, q2StepsGlobal;
extern double   distanceGlobal;
extern int      romGetSpeedMotor();
extern TaskHandle_t    motorsTask;
//====Function prototypes====
double dkX(double , double );
double dkY(double , double );
//====
bool q2DirectionNew = false, q1DirectionNew = false, q2DirectionOld = false, q1DirectionOld = false;
/**
 * @brief It's the class constructor
 */
Motors::Motors()
{
    this->microstepping = MICROSTEPPING;
    degrees_per_step = (littlePulleySize / bigPulleySize) * (PI / 180.0) * (1.8 / microstepping);
    Serial.print("microstepping");
    Serial.println(microstepping);
    stepper1 = AccelStepper(1, STEP_PIN, DIR_PIN);
    stepper2 = AccelStepper(1, STEP_PIN2, DIR_PIN2);
    // stepper1.setPinsInverted(true);
    // stepper2.setPinsInverted(true);
    lastPoint = false;
}


/**
 * @brief Esta funcion hace que el robot se mueve de su posicion actual hacia una nueva posicion x,y.
 * 
 * Si las coordenadas x,y se encuentran fuera del espacio de trabajo, la funcion se encarga de limitarlas.
 * La posicion x,y se mide en milimetros.
 * Si la distancia que tiene que recorrer es menor a 0.5 milimetros entonces no avanza a menos que el parametro littleMovement sea verdadero.
 * Si la distancia a recorrer es mayor a 1.1 milimetros, entonces se avanza en linea recta al nuevo punto en una trayectoria de puntos equidistantes a 1 mm formando una linea recta.
 * @param x es la coordenada en el eje x, medida en milimetros, hacia donde se desea avanzar.
 * @param y es la coordenada en el eje y, medida en milimetros, hacia donde se desea avanzar.
 * @param littleMovement es una variable que si su valor es false entonces no se movera a menos que la distancia minima sea 0.5 mm y true para que se mueva en cualquier caso.
 */
void Motors::moveTo(double x, double y, bool littleMovement)
{
    double q1, q2, distance;
    long steps_of_q1, steps_of_q2, positions[2];
    double currentSpeed1, currentSpeed2;
    MultiStepper stepps;
    AccelStepper stepper1Aux;
    AccelStepper stepper2Aux;
    stepps.addStepper(stepper1Aux);
    stepps.addStepper(stepper2Aux);
    if (pauseModeGlobal){
        while (pauseModeGlobal)
        {
            delay(200);
        }
    }
    bool ceroPoint = false;
    if (x == 0 && y ==0){
        ceroPoint = true;
    }
    ik(x, y, &q1, &q2);
    x = dkX(q1, q2);
    y = dkY(q1, q2);
    distance = module(x, y, x_current, y_current);
    // Serial.print("d= ");
    // Serial.println(distance);
    if (distance > MAX_DISTANCE_FOR_MOVEMENT)
    {
        moveInterpolateTo(x, y, distance, littleMovement);
    }
    else if (distance > 0.5 || ceroPoint)
    {
        ik(x, y, &q1, &q2);
        steps_of_q1 = calculate_steps(q1_current, q1);
        steps_of_q2 = calculate_steps(q2_current, q2);

        if (!(steps_of_q1 == 0 && steps_of_q2 == 0))// || littleMovement)
        { 
#ifdef IMPLEMENT_ACCELERATION
            //save positions in Buffer in case of restoring position when speed is changed
            xBuffer[xyPointer] = x;
            yBuffer[xyPointer] = y;

            long posLong[2];
            double posConstrained[2];
            double ACCELERATION;
            double time1, time2;
            posLong[0] = 0;
            posLong[1] = 0;
            posConstrained[0] = 0;
            posConstrained[1] = 0;
            //guardar los datos en arrays.
            double factor, deltaQ1, deltaQ2;
            int factorInt;
            int maxSteps;
            long stepsQ1Og = steps_of_q1;
            long stepsQ2Og = steps_of_q2;
            
            
            double momentaryPathSpeed = millimeterSpeed;
            ACCELERATION = momentaryPathSpeed * 2.5;
            if (abs(steps_of_q1) > abs(steps_of_q1 + steps_of_q2)){
                maxSteps = abs(steps_of_q1);
            }
            else{
                maxSteps = abs(steps_of_q2 + steps_of_q1);
            }
            maxSpeed = (maxSteps / distance) * momentaryPathSpeed;
            if (maxSpeed > MAX_STEPS_PER_SECOND * MICROSTEPPING){
                distance = maxSteps/double(MAX_STEPS_PER_SECOND * MICROSTEPPING)*momentaryPathSpeed;
            }
            if(abs(steps_of_q2) > abs(steps_of_q1)){
                factor = fabs((steps_of_q2)/MAX_STEPS_PER_INSTRUCTION);
            }
            else{
                factor = fabs(steps_of_q1/MAX_STEPS_PER_INSTRUCTION);
            }
            
            if (factor < 1.0){
                factor = 1.0;
            }
            deltaQ1 = (steps_of_q1)/factor;
            deltaQ2 = (steps_of_q2)/factor;
            factorInt = int(factor);
            for (int k=0; k <= factorInt; k++){
                posConstrained[0] += deltaQ1;
                posConstrained[1] += deltaQ2;
                if (k == factorInt){
                    posConstrained[0] = stepsQ1Og;
                    posConstrained[1] = stepsQ2Og;
                }
                steps_of_q1 = long(posConstrained[0]) - posLong[0];
                steps_of_q2 = long(posConstrained[1]) - posLong[1];
                posLong[0] = long(posConstrained[0]);
                posLong[1] = long(posConstrained[1]);
                if ((steps_of_q1 == 0 && steps_of_q2 ==0)){
                    continue;
                }

                q1StepsBuffer[pointerBuffer] = steps_of_q1;
                q2StepsBuffer[pointerBuffer] = steps_of_q2;
                distanceBuffer[pointerBuffer] = distance;
                maxStepsBuffer[pointerBuffer] = maxSteps;
                xyPointerBuffer[pointerBuffer] = xyPointer;

                positions[0] = steps_of_q1;
                positions[1] = steps_of_q2 + steps_of_q1;

                stepper1Aux.setCurrentPosition(0);
                stepper2Aux.setCurrentPosition(0);
                maxSpeed = (maxSteps / distance) * _pathSpeed;
                if (maxSpeed > MAX_STEPS_PER_SECOND * MICROSTEPPING)
                    maxSpeed = MAX_STEPS_PER_SECOND * MICROSTEPPING;
                stepper1Aux.setMaxSpeed(maxSpeed);
                stepper2Aux.setMaxSpeed(maxSpeed);
                stepps.moveTo(positions);
                currentSpeed1 = stepper1Aux.speed();
                currentSpeed2 = stepper2Aux.speed();
                if (currentSpeed1 == 0){
                    currentSpeed1 = oldSpeed1;
                }
                if (currentSpeed2 == 0){
                    currentSpeed2 = oldSpeed2;
                }
                //revisar si hay problema con el movimiento brusco
                double accel1 = fabs(currentSpeed1 - oldSpeed1);
                double accel2 = fabs(currentSpeed2 - oldSpeed2);
                if (k == factorInt || lowerSpeed){
                    accel1 = 0;
                    accel2 = 0;
                    lowerSpeed = false;
                }
                /*String info1;
                String info2;
                if ((accel1 > ACCEL_THRESHOLD) || (accel2 > ACCEL_THRESHOLD)){
                    info1 = "1:" + String(int(currentSpeed1)) + "," + String(positions[0]) + "," + String(factorInt);//",1";
                }
                else{
                    info1 = "1:" + String(int(currentSpeed1)) + "," + String(positions[0]) + "," + String(factorInt);//",0";
                }
                
                info2 = "2:" + String(int(currentSpeed2)) + "," + String(positions[1]) + "," + String(int(_pathSpeed));
                Serial.println(info1);
                Serial.println(info2);*/
                if (lastPoint && posConstrained[0] == stepsQ1Og && posConstrained[1] == stepsQ2Og){
                    accel1 = 2500;//greaterValue(fabs(currentSpeed1), fabs(currentSpeed2));
                    lastPoint = false;
                    #ifdef DEBBUGING_DATA
                        Serial.println("last point in moveTo function");
                    #endif
                }
                if ((accel1 > ACCEL_THRESHOLD) || (accel2 > ACCEL_THRESHOLD)){
                    double maxAccel, safeSpeed;
                    maxAccel = greaterValue(accel1, accel2);
                    safeSpeed = _pathSpeed / (maxAccel/double(ACCEL_THRESHOLD));
                    _pathSpeed = safeSpeed;
                    int i=pointerBuffer-1; 
                    if (i < 0){
                        i = SAMPLES - 1;
                    }
                    if (pointerBuffer != cPointerBuffer)
                    {
                        double newSpeed = safeSpeed;
                        while (true)
                        {
                            newSpeed = ACCELERATION*timesBuffer[i] + newSpeed;
                            if (newSpeed < pathSpeedBuffer[i]){ pathSpeedBuffer[i] = newSpeed; }
                            else{ break; }
                            if (i == cPointerBuffer){ break; }
                            i -= 1;
                            if (i < 0){ i = SAMPLES - 1; }
                        }
                    }
                }
                pathSpeedBuffer[pointerBuffer] = _pathSpeed;
                
                //calcular velocidades del siguiente paso.
                maxSpeed = (maxSteps / distance) * _pathSpeed;
                if (maxSpeed > MAX_STEPS_PER_SECOND * MICROSTEPPING)
                    maxSpeed = MAX_STEPS_PER_SECOND * MICROSTEPPING;
                stepper1Aux.setMaxSpeed(maxSpeed);
                stepper2Aux.setMaxSpeed(maxSpeed);
                stepps.moveTo(positions);
                currentSpeed1 = stepper1Aux.speed();
                currentSpeed2 = stepper2Aux.speed();
                
                if (currentSpeed1 != 0){
                    time1 = steps_of_q1/currentSpeed1;
                }else{time1 = 0;}
                if (currentSpeed2 != 0){
                    time2 = (steps_of_q2 + steps_of_q1)/currentSpeed2;
                }else{time2 = 0;}


                if (time1 > time2){
                    timesBuffer[pointerBuffer] = time1;
                }
                else{
                    timesBuffer[pointerBuffer] = time2;
                }
                //in order to evoid abrupt increses of acceleration, we limit the time to 30 milliseconds
                if (timesBuffer[pointerBuffer] > 0.03){
                    timesBuffer[pointerBuffer] = 0.03;
                }
                _pathSpeed = ACCELERATION * timesBuffer[pointerBuffer] + _pathSpeed;
                if (_pathSpeed > millimeterSpeed){
                    _pathSpeed = millimeterSpeed;
                }
                
                //Serial.flush();
                //acturalizar variables viejas
                
                if (currentSpeed1 != 0 && k != factorInt){
                    oldSpeed1 = currentSpeed1;
                }
                if (currentSpeed2 != 0 && k != factorInt){
                    oldSpeed2 = currentSpeed2;
                }

                //incrementar pointerBuffer donde se esta guardando el dato
                pointerBuffer += 1;
                if (pointerBuffer >= SAMPLES - 1){
                    fullBuffer = true;
                }
                if (pointerBuffer >= SAMPLES){
                    pointerBuffer = 0;
                }
                
                if (fullBuffer){
                    unsigned long t = millis();
                    while (eTaskGetState(motorsTask) != 3){
                        //Serial.println("delay de 2 segundos");
                        if (millis() - t > 500){
                            delay(1);
                        }
                        continue;
                    }
                    updateVariablesToMovingThread();
                    startMovement = true;
                    
                    vTaskResume(motorsTask);
                    //
                    cPointerBuffer += 1;
                    if (cPointerBuffer >= SAMPLES){
                        cPointerBuffer = 0;
                    }
                }
#endif
#ifndef IMPLEMENT_ACCELERATION
            while (eTaskGetState(motorsTask) != 3){
                continue;
            }
            q1StepsGlobal = steps_of_q1;
            q2StepsGlobal = steps_of_q2;
            distanceGlobal = distance;

            startMovement = true;
            vTaskResume(motorsTask);
#endif
                q1_current += degrees_per_step * steps_of_q1;
                q2_current += degrees_per_step * steps_of_q2;
                q1_current = normalizeAngle(q1_current);
                q2_current = normalizeAngle(q2_current);
                x_current = dkX(q1_current, q2_current);
                y_current = dkY(q1_current, q2_current);
            }
            
            xyPointer += 1;
            if (xyPointer >= SAMPLES){
                xyPointer = 0;
            }
        }
    }
}

/**
 * @brief execute the remaining buffered steps. 
 */
void Motors::completePath(){
    while (true)
    {
        while (eTaskGetState(motorsTask) != 3){
            continue;
        }
        if (cPointerBuffer == pointerBuffer){
            break;
        }
        updateVariablesToMovingThread();
        
        startMovement = true;
        
        vTaskResume(motorsTask);
        cPointerBuffer += 1;
        if (cPointerBuffer >= SAMPLES){
            cPointerBuffer = 0;
        }
    }
    fullBuffer = false;
    cPointerBuffer = 0;
    pointerBuffer = 0;
    oldSpeed1 = 0;
    oldSpeed2 = 0;
}
/**
 * @brief this function resets some variables and replace the position variables to the real positions of Sandsara.
 */
void Motors::stopAndResetPositions(){
    //Serial.println("stopAndResetPositions");
    while (eTaskGetState(motorsTask) != 3){
        delay(1);
        continue;
    }
    //Serial.println("salio de while");    
    cPointerBuffer = 0;
    pointerBuffer = 0;
    fullBuffer = false;
    oldSpeed1 = realSpeed1;
    oldSpeed2 = realSpeed2;
    q1_current = realQ1;
    q2_current = realQ2;
    x_current = dkX(q1_current, q2_current);
    y_current = dkY(q1_current, q2_current);
    /*Serial.print("q1_current ");
    Serial.println(q1_current);
    Serial.print("q2_current ");
    Serial.println(q2_current);*/
}

void Motors::updateVariablesToMovingThread(){
    q1StepsGlobal = q1StepsBuffer[cPointerBuffer];
    q2StepsGlobal = q2StepsBuffer[cPointerBuffer];
    distanceGlobal = distanceBuffer[cPointerBuffer];
    maxStepsGlobal = maxStepsBuffer[cPointerBuffer];
    maxPathSpeedGlobal = pathSpeedBuffer[cPointerBuffer];
    cxyPointer = xyPointerBuffer[cPointerBuffer];
}

/**
 * @brief Recalcula las velocidades de los puntos guardados para movimientos futuros
 */
void Motors::resetSpeeds(){
    int cxyPointerAux = cxyPointer;
    int xyPointerAux = xyPointer;

    double xBufferAux[SAMPLES], yBufferAux[SAMPLES];
    stopAndResetPositions();
    /*Serial.print("xActual: ");
    Serial.println(x_current);
    Serial.print("yActual: ");
    Serial.println(y_current);

    Serial.print("cxyPointer: ");
    Serial.println(cxyPointer);

    Serial.print("xBuffer: ");
    Serial.println(xBuffer[cxyPointer]);
    Serial.print("yBuffer: ");
    Serial.println(yBuffer[cxyPointer]);*/
    cxyPointer = 0;
    xyPointer = 0;
    for (int i = 0; i < SAMPLES; i++){
        xBufferAux[i] = xBuffer[i];
        yBufferAux[i] = yBuffer[i];
    }

    while (true)
    {
        cxyPointerAux += 1;
        if (cxyPointerAux >= SAMPLES){
            cxyPointerAux = 0;
        }
        if (cxyPointerAux == xyPointerAux){
            break;
        }
        moveTo(xBufferAux[cxyPointerAux], yBufferAux[cxyPointerAux]);
        lowerSpeed = false;
    }
}

double greaterValue(double a, double b){
    if (a > b){
        return a;
    }
    return b;
}
/**
 * @brief Se usa esta funcion para avanzar de la posicion actual a un punto nuevo en linea recta por medio de puntos equidistantes a un 1 mm.
 * 
 * La esfera va avanzando cada 1 mm hasta la posicion final, para esto se calcula un incremento en el eje 'x' y otro en el eje 'y' para cada interacion.
 * @param x coordenada en el eje x, medida en milimetros, a la que se desea avanzar.
 * @param y coordenada en el eje y, medida en milimetros, a la que se desea avanzar.
 * @param distance es la distancia, medida en milimetros, entre el punto actual y el punto al que se desea avanzar.
 */
void Motors::moveInterpolateTo(double x, double y, double distance, bool littlemovement)
{
    double alpha = atan2(y - y_current, x - x_current);
    double delta_x, delta_y;
    double x_aux = x_current, y_aux = y_current;
    delta_x = cos(alpha);
    delta_y = sin(alpha);
    int intervals = distance;
    for (int i = 1; i <= intervals; i++)
    {
        x_aux += delta_x;
        y_aux += delta_y;
        moveTo(x_aux, y_aux, littlemovement);
    }
    moveTo(x, y, littlemovement);
}
//====

/**
 * @brief calcula el modulo de la posicion actual de la esfera.
 * @return La distancia entre el centro y la posicion actual del robot.
 */
double Motors::getCurrentModule()
{
    return zPolar(x_current, y_current);
}

/**
 * @brief Calcula el angulo de la posicion actual de la esfera.
 * @return El angulo, medido desde la horizontal, de la posicion actual del robot.
 * @note el angulo se encuentra en el rango de [0 , 2*PI)
 */
double Motors::getCurrentAngle()
{
    return thetaPolar(x_current, y_current);
}

/**
 * @brief devuelve un valor para saber en donde se encuentra la esfera.
 * @returns un entero que puede significar lo siguiente.
 * 0, se encuentra en el centro o a 2 mm.
 * 1, no se encuentra ni en el centro ni en la orilla.
 * 2, se cuentra en la orilla o a 2 mm.
 */
int Motors::position(){
    double robotModule = getCurrentModule();
    int pos;
    if (robotModule >= l1 + l2 - 2){
        pos = 2;
    }
    else if (robotModule <= 2)
    {
        pos = 0;
    }
    else{
        pos = 1;
    }
    return pos;
}
//====Get and Set Functions====

/**
 * @brief obtiene la variable miembro zCurrent
 * @return the variable member zCurrent
 */
double Motors::getZCurrent(){
    return zCurrent;
}

/**
 * @brief obtiene la varieble miembro thetaCurrent
 * @return the variable member thetaCurrent
 */
double Motors::getThetaCurrent(){
    return thetaCurrent;
}

/**
 * @brief obtiene el valor de la velocidad actual de Sandsara en milimetros por segundo
 * @return la variable miembro millimeterSpeed.
 */
double Motors::getSpeed(){
    return millimeterSpeed;
}

/**
 * @brief cambia la velocidad de SandSara.
 * @param speed es la nueva valocidad, en milimetros por segundo, que va a tener de SandSara.
 */
void Motors::setSpeed(int speed){
    if (millimeterSpeed > speed){
        lowerSpeed = true;
    }
    millimeterSpeed = speed;
    _pathSpeed = speed;
}

/**
 * @brief moficica el miembro z_current.
 * @param z es el valor que se le va a asignar a la variable miembro zCurrent.
 * @note  Esto es importante para que los archivos .thr tengan una referencia de donde empezar a moverse.
 */
void Motors::setZCurrent(double z)
{
    zCurrent = z;
}

/**
 * @brief moficica el miembro theta_current.
 * @param theta es el valor que se le va a asignar a la variable miembro thetaCurrent.
 * @note  Esto es importante para que los archivos .thr tengan una referencia de donde empezar a moverse.
 */
void Motors::setThetaCurrent(double theta)
{
    thetaCurrent = theta;
}

/**
 * @brief moficica el miembro realQ1.
 * @param q1 es el valor que se le va a asignar a la variable miembro realQ1.
 */
void Motors::setRealQ1(double q1){
    realQ1 = q1;
}

/**
 * @brief moficica el miembro realQ2.
 * @param q2 es el valor que se le va a asignar a la variable miembro realQ1.
 */
void Motors::setRealQ2(double q2){
    realQ2 = q2;
}

/**
 * @brief moficica el miembro realQ1.
 * @return la variable miembro realQ1.
 */
double Motors::getRealQ1(){
    return realQ1;
}

/**
 * @brief moficica el miembro realQ1.
 * @return la variable miembro realQ2.
 */
double Motors::getRealQ2(){
    return realQ2;
}

/**
 * @brief moficica el miembro realSpeed1.
 * @param speed1 es el valor que se le va a asignar a la variable miembro realQ1.
 */
void Motors::setRealSpeed1(double speed1){
    realSpeed1 = speed1;
}

/**
 * @brief moficica el miembro realSpeed2.
 * @param speed2 es el valor que se le va a asignar a la variable miembro realQ1.
 */
void Motors::setRealSpeed2(double speed2){
    realSpeed2 = speed2;
}

//====Configuration Methods====
/**
 * @brief inicializa el objeto Motors
 * @param xInit es la coordenada en el eje x, medida en milimetros, que se desea como posicion inicial, por defecto es 0.
 * @param yInit es la coordenada en el eje y, medida en milimetros, que se desea como posicion inicial, por defecto es 0.
 */
void Motors::init(double xInit, double yInit)
{
    double q1, q2;
    //this 2 lines was added because microstepping, littlePulleySize and bigPulleySize need to be initialized
    this->microstepping = MICROSTEPPING;
    degrees_per_step = (littlePulleySize / bigPulleySize) * (PI / 180.0) * (1.8 / microstepping);

    stepper1.setMaxSpeed(50 * microstepping);
    stepper2.setMaxSpeed(50 * microstepping);
    stepper1.setCurrentPosition(0);
    stepper2.setCurrentPosition(0);
    steppers.addStepper(stepper1);
    steppers.addStepper(stepper2);
    
    ik(xInit, yInit, &q1, &q2);
    q1_current = q1;
    q2_current = q2;
    x_current = dkX(q1_current, q2_current);
    y_current = dkY(q1_current, q2_current);
    calculate_line_equations();
}

//----------------------------mathematics---------------------------------------------------

//Kinematics--------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
/**
 * @brief Calcula la cinematica directa del robot para la coordenada en x.
 * @param q1 es el angulo del motor que corresponde al primer eslabon de Sandsara.
 * @param q2 es el angulo del motor que corresponde al segundo eslabon de Sandsara.
 * @return la coordenada en el eje x, medida en milimetros, correspondiente a los angulos q1,q2 de Sandsara.
 */
double dkX(double q1, double q2)
{
    return l1 * cos(q1) + l2 * cos(q1 + q2);
}

/**
 * @brief Calcula la cinematica directa del robot para la coordenada en y.
 * @param q1 es el angulo del motor que corresponde al primer eslabon de Sandsara.
 * @param q2 es el angulo del motor que corresponde al segundo eslabon de Sandsara.
 * @return la coordenada en el eje y, medida en milimetros, correspondiente a los angulos q1,q2 de Sandsara.
 */
double dkY(double q1, double q2)
{
    return l1 * sin(q1) + l2 * sin(q1 + q2);
}

/**
 * @brief Calcula la cinematica inversa de Sandsara.
 * Las matematicas para resolver la cinematica inversa de Sandsara se vasan en la solucion para un robot SCARA.
 * @param x coordenada en el eje x, medida en milimetros.
 * @param y coordenada en el eje y, medida en milimetros.
 * @param q1 el angulo calculado correspondiente al motor 1 para la posicion x,y.
 * @param q1 el angulo calculado correspondiente al motor 2 para la posicion x,y.
 */
void Motors::ik(double x, double y, double *q1, double *q2)
{
    double z, z_max, theta;
    int auxiliar, i;
    ///calculus of z
    z = zPolar(x, y);
    ///calculus of theta
    theta = thetaPolar(x, y);
    ///if x,y is out of range, z will be the maximun radius possible
    if (z > MAX_RADIO)
        z = MAX_RADIO;
    ///Delimiter module z
    if (productType){
        i = theta / (2 * PI / (2 * no_picos));
        double tantheta = tan(theta);
        z_max = abs(b[i] / (tantheta - m[i]) * sqrt(1 + tantheta*tantheta));
        if (z > z_max){
            z = z_max;
        }
    }
    
    ///calculus of q1 that is always possitive
    *q1 = theta - acos(z / (2 * l1));
    if (*q1 < 0)
        *q1 = *q1 + 2 * PI;
    ///calculus of q2
    *q2 = 2 * (theta + 2 * PI - *q1);
    auxiliar = *q2 / (2 * PI);
    ///in case q2 is greater than 2pi
    *q2 = *q2 - 2 * PI * auxiliar;
}

//====Operations with components====
/**
 * @brief rota la posicion x,y con centro en 0,0 tantos grados como se desee.
 * 
 * por ejemplo, si se rota la posicion (1,0) 90 grados el resultado seria un punto (0,1).
 * @param x coordenada en el eje x.
 * @param y coordenada en el eje y.
 * @param angle es el angulo que se desea rotar los puntos x,y.
 * @note los valores x,y se pasan por referencia.
 */
void Motors::rotate(double &x, double &y, double angle)
{
    double z = zPolar(x, y);
    double theta = thetaPolar(x, y);
    theta += angle;
    x = z * cos(theta);
    y = z * sin(theta);
}

/**
 * @brief Calcula la distancia entre 2 puntos.
 * @param x1 es el valor en el eje x del primer punto.
 * @param y1 es el valor en el eje y del primer punto.
 * @param x2 es el valor en el eje x del segundo punto.
 * @param y2 es el valor en el eje y del segundo punto.
 * @return la distancia entre ambos puntos.
 */
double Motors::module(double x1, double y1, double x2, double y2)
{
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

/**
 * @brief Calcula la distancia entre 2 puntos en coordenadas Polares.
 * @param z1 es el valor en el eje z del primer punto.
 * @param t1 es el valor en el eje theta del primer punto.
 * @param z2 es el valor en el eje z del segundo punto.
 * @param t2 es el valor en el eje theta del segundo punto.
 * @return la distancia entre ambos puntos.
 */
double Motors::polarModule(double z1, double t1, double z2, double t2)
{
    double x1, y1, x2, y2;
    x1 = z1 * cos(t1);
    y1 = z1 * sin(t1);
    x2 = z2 * cos(t2);
    y2 = z2 * sin(t2);
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

/**
 * @brief calcula el valor del modulo del punto en coordenadas polares
 * @param x es el valor en el eje x del punto.
 * @param y es el valor en el eje y del punto.
 * @return el modulo del punto en coordenadas polares.
 */
double Motors::zPolar(double x, double y)
{
    return sqrt(pow(x, 2) + pow(y, 2));
}

/**
 * @brief calcula el angulo que se forma con la horizontal, de un punto x,y.
 * @param x es el valor de la coordenada en el eje x, medido en milimetros.
 * @param y es el valor de la coordenada en el eje y, medido en milimetros.
 * @return el angulo, medido desde la horizontal, del punto x,y.
 * @note el angulo se encuentra en el rango de [0 , 2*PI)
 */
double Motors::thetaPolar(double x, double y)
{
    //calculus of theta
    double theta;
    if (x > 0)
        theta = atan(y / x);
    else if (x < 0)
        theta = atan(y / x) + PI;
    else
    {
        if (y >= 0)
            theta = PI / 2.0;
        else
            theta = 3.0 / 2.0 * PI;
    }
    return normalizeAngle(theta);
}

/**
 *@brief normaliza un angulo para estar en el rango de [0,2*PI).
 *
 *@param angle es el angulo, medido en radianes, que se desea normalizar.
 *@return un valor con el rango de [0 , 2*PI).
 */
double Motors::normalizeAngle(double angle)
{
    long aux = angle / (2 * PI);
    angle -= aux * 2 * PI;
    if (angle < 0)
    {
        return angle + 2 * PI;
    }
    else
    {
        return angle;
    }
}

/**
 * @brief calcula la longitud de arco que se forma al girar de un punto A a un punto B en coordenadas polares.
 * 
 * En otras palabras, se calcula la distancia que se recorre en una trayectoria espiral de un punto A a un punto B, el numero de grados totales que gira
 * esta definido por el parametro deltaTheta.
 * @param deltaZ es la diferencia entre la componente modulo del punto final menos la componente modulo del punto inicial.
 * @param deltaTheta es la diferencia entre la componente angulo del punto final menos la componente angulo del punto inicial.
 * @param zInit es la componente modulo del punto inicial.
 */
double Motors::arcLength(double deltaZ, double deltaTheta, double zInit)
{
    double k, k2, a, a2, z, z2, inSqrt, inSqrt2;
    deltaTheta = abs(deltaTheta);
    if (deltaZ < 0)
    {
        zInit = zInit + deltaZ;
        deltaZ = abs(deltaZ);
    }
    if (deltaZ == 0)
    {
        return deltaTheta * (zInit + deltaZ);
    }
    if (deltaTheta == 0)
    {
        return deltaZ;
    }
    k = deltaZ;
    k2 = deltaZ * deltaZ;
    a = deltaTheta;
    a2 = deltaTheta * deltaTheta;
    z = zInit;
    z2 = zInit * zInit;
    inSqrt2 = a2 * z2 + k2;
    inSqrt = a2 * k2 + 2 * a2 * k * z + inSqrt2;
    return (k + z) / (2 * k) * sqrt(inSqrt) + k / (2 * a) * log(a * (sqrt(inSqrt) + a * k + a * z)) - z / (2 * k) * sqrt(inSqrt2) - k / (2 * a) * log(a * (sqrt(inSqrt2) + a * z));
}
//====Workspace mathematics====
/**
 * @brief Calcula las escuaciones que describen la geometria de Stelle.
 * 
 * Las ecuaciones que describen la geometria de Stelle son 10 lineas rectas, lo que calcula esta funcion son los parametros m y b de una recta de la forma
 * y = mx + b.
 * @return no retorna ningun valor pero modifica las variables miembro m y b.
 * @note m contiene las pendientes de las rectas que describen la geometria de Stelle.
 *       b contiene informacion de las rectas que describen la geometria de Stelle.
 */
void Motors::calculate_line_equations()
{
    double z = radius_2;
    double theta;
    for (int i = 0; i < 2 * no_picos; i++)
    {
        double x1, x2, y1, y2;
        theta = 2 * PI / (2 * no_picos) * i;
        if (i % 2 == 0)
            z = radius_1;
        else
            z = radius_2;
        x1 = z * cos(theta);
        y1 = z * sin(theta);
        if (i % 2 != 0)
            z = radius_1;
        else
            z = radius_2;
        theta = 2 * PI / (2 * no_picos) * (i + 1);
        x2 = z * cos(theta);
        y2 = z * sin(theta);
        m[i] = (y2 - y1) / (x2 - x1);
        b[i] = y2 - m[i] * x2;
    }
}

//====Motor maths====
/**
 * @brief Calcula los pasos que se debe mover el motor para ir de un angulo a otro.
 * @param q1 es el angulo inicial.
 * @param q2 es el angulo al que se quiere llegar.
 * @return los angulos necesarios para ir de q1 a q2.
 * @note existen 2 caminos para llegar a un cierto angulo, pero siempre se escoge el mas corto.
 */
long Motors::calculate_steps(double q1, double q2)
{
    double steps_option_1, steps_option_2;
    int aux1, aux2;
    steps_option_1 = 2 * PI - q2 + q1;
    steps_option_2 = 2 * PI - q1 + q2;
    aux1 = steps_option_1 / (2 * PI);
    aux2 = steps_option_2 / (2 * PI);
    steps_option_1 = steps_option_1 - aux1 * 2 * PI;
    steps_option_2 = steps_option_2 - aux2 * 2 * PI;
    if (steps_option_1 <= steps_option_2)
        return -steps_option_1 / degrees_per_step;
    else
        return steps_option_2 / degrees_per_step;
}

void Motors::constrainXY(double& x, double& y)
{
    double z = Motors::zPolar(x, y);
    ///calculus of theta
    double theta = Motors::thetaPolar(x, y);
    ///if x,y is out of range, z will be the maximun radius possible
    if (z > l1 + l2)
        z = l1 + l2;
    ///Delimiter module z
    int i = theta / (2 * PI / (2 * no_picos));
    double tantheta = tan(theta);
    double z_max = abs(b[i] / (tantheta - m[i]) * sqrt(1 + tantheta * tantheta));
    if (z > z_max) {
        z = z_max;
    }
    x = z * cos(theta);
    y = z * sin(theta);
}