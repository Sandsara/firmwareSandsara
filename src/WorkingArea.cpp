#include "WorkingArea.h"

int MICROSTEPPING = String(String(dataS[1]) + String(dataS[2])).toFloat();


byte L;
byte H;
int flag = 0;
int p = 0;
int motorAngle = 0;
int Speed = 5000;
int A = 0;
int B = 0;
int maximum = 0;
int maximum2 = 0;
int minimum = 5000;
int minimum2 = 5000;
int sensorMax1;
int sensorMin1;

TMC2209Stepper tmcMotorA(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper tmcMotorB(&SERIAL_PORT2, R_SENSE, DRIVER_ADDRESS2);

hw_timer_t *timer1 = NULL;

MeanFilter<long> meanFilter(5);
MeanFilter<long> meanFilter2(40);
MeanFilter<long> meanFilter3(10);
MeanFilter<long> meanFilter4(10);

int value[80];
int value_r[80];
int value2[300];
int value2_r[300];
int meanVector[5];
int simiVector[5];
int maximumVector[5];
int minimumVector[5];
int meanVector2[5];
int simiVector2[5];
int maximumVector2[5];
int minimumVector2[5];

void findingSlowSensor2(void);
void findingSlowSensor1(void);
void findingSlowSensor2Negative(void);
void findingSlowSensor1Negative(void);
void configTimerForMovement();
void move(int, int, int);
int zero_Hall1(void);
int zero_Hall2(void);
int Pole1(void);
int Pole2(void);
int Check_ini(void);

int avoid = 0;
int avoid2 = 0;
int level_zero1;
int level_zero2;
int sensorMax2;
int sensorMin2;
int sensorRead2;
int sensorRead1;
int sensorPole1;
int sensorPole2;
int adjustIniFlag = 0;

WorkingArea::WorkingArea()
{
}

/**
 * @brief Esta funcion activa y desactiva los pines de salida STEP mediante interrupciones. 
 * @param flag Es la variable para determinar que brazo es el que gira.
 * 0 representa el giro del brazo 1.
 * 1 representa que no se mueve ningun brazo.
 * 2 representa el giro de ambos brazos de manera simultanea.
 * 3 representa el giro del brazo 3.
 */

void WorkingArea::prepareMotors()
{
	SERIAL_PORT.begin(115200);
	SERIAL_PORT2.begin(115200);

	pinMode(DIAG_PIN, INPUT);
	pinMode(EN_PIN, OUTPUT);
	pinMode(STEP_PIN, OUTPUT);
	pinMode(DIR_PIN, OUTPUT);

	pinMode(DIAG_PIN2, INPUT);
	pinMode(EN_PIN2, OUTPUT);
	pinMode(STEP_PIN2, OUTPUT);
	pinMode(DIR_PIN2, OUTPUT);

	digitalWrite(EN_PIN, LOW);
	digitalWrite(DIR_PIN, HIGH);

	digitalWrite(EN_PIN2, LOW);
	digitalWrite(DIR_PIN2, HIGH);

	tmcMotorA.begin();                   

	tmcMotorA.pdn_disable(true);         // enables the  PDN/UART comunication.
	
	tmcMotorA.toff(4);                   // Establece el tiempo de disminucion lenta (tiempo de apagado) [1 ... 15]
	                                  // Esta configuración también limita la frecuencia máxima de chopper. Para operar con StealthChop
									  // En caso de operar solo con StealthChop, cualquier configuración está bien.
	
	tmcMotorA.blank_time(24);
	
	tmcMotorA.rms_current(CURRENT_IN_CALIBRATION);          // Fija el valor de la corriente

	tmcMotorA.microsteps(MICROSTEPPING); // Se define el valor de microstepps

	tmcMotorA.TCOOLTHRS(0xFFFFF);        // Velocidad umbral inferior para encender la energía inteligente CoolStep y StallGuard a la salida del DIAG
	
	tmcMotorA.semin(0);                  // Umbral inferior CoolStep [0 ... 15].
                                      // Si SG_RESULT cae por debajo de este umbral, CoolStep aumenta la corriente a ambas bobinas.
                                      // 0: deshabilitar CoolStep

	tmcMotorA.shaft(false);              //Establece el sentido de giro del motor mediante la comunicacion UART

	tmcMotorA.sedn(0b01);                // Establece el número de lecturas de StallGuard2 por encima del umbral superior necesario
                                      // por cada disminución de corriente de la corriente del motor.
	
	tmcMotorA.SGTHRS(STALL_VALUE);       // Nivel de umbral StallGuard4 [0 ... 255] para la detección de bloqueo. Compensa
  									  // características específicas del motor y controla la sensibilidad. Un valor más alto da un valor más alto
  									  // sensibilidad. Un valor más alto hace que StallGuard4 sea más sensible y requiere menos torque para
  									  // indica una oposicion al movimiento. 

	
	
	tmcMotorB.begin();

	tmcMotorB.pdn_disable(true);         // Activa la comunicacion PDN/UART

	tmcMotorB.toff(4);                   // Establece el tiempo de disminucion lenta (tiempo de apagado) [1 ... 15]
	                                   // Esta configuración también limita la frecuencia máxima de chopper. Para operar con StealthChop
									   // En caso de operar solo con StealthChop, cualquier configuración está bien.

	tmcMotorB.blank_time(24);

	tmcMotorB.rms_current(CURRENT_IN_CALIBRATION);          // Fija el valor de la corriente

	tmcMotorB.microsteps(MICROSTEPPING); // Se define el valor de microstepps

	tmcMotorB.TCOOLTHRS(0xFFFFF);        // Velocidad umbral inferior para encender la energía inteligente CoolStep y StallGuard a la salida del DIAG

	tmcMotorB.semin(0);                  // Umbral inferior CoolStep [0 ... 15].
                                       // Si SG_RESULT cae por debajo de este umbral, CoolStep aumenta la corriente a ambas bobinas.
                                       // 0: deshabilitar CoolStep

	//tmcMotorA.semax(2);

	tmcMotorB.shaft(false);              // Establece el sentido de giro del motor mediante la comunicacion UART

	tmcMotorB.sedn(0b01);                // Establece el número de lecturas de StallGuard2 por encima del umbral superior necesario
                                       // por cada disminución de corriente de la corriente del motor.

	tmcMotorB.SGTHRS(STALL_VALUE2);      // Nivel de umbral StallGuard4 [0 ... 255] para la detección de bloqueo. Compensa
  									   // características específicas del motor y controla la sensibilidad. Un valor más alto da un valor más alto
  									   // sensibilidad. Un valor más alto hace que StallGuard4 sea más sensible y requiere menos torque para
  									   // indica una oposicion al movimiento. 

	EEPROM.begin(EEPROM_SIZE);
    delay(100);
    while (tmcMotorA.microsteps() != MICROSTEPPING){
        tmcMotorA.microsteps(MICROSTEPPING);
        delay(100);
    }
    while (tmcMotorB.microsteps() != MICROSTEPPING){
        tmcMotorB.microsteps(MICROSTEPPING);
        delay(100);
    }
}

void IRAM_ATTR onTimer()
{

	if (flag == 0)
	{
		digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
	}
	if (flag == 2)
	{
		digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
		digitalWrite(STEP_PIN2, !digitalRead(STEP_PIN2));
	}
	if (flag == 3)
	{
		digitalWrite(STEP_PIN2, !digitalRead(STEP_PIN2));
	}
}

int WorkingArea::findZeroPoint()
{
	//====WITH STALLGUARD====
	int value_f;
	int mean;
	int cont_turn = 0;
	int cont_turn1 = 0;
	flag = 0;

	adjustIniFlag = Check_ini();
	if (adjustIniFlag == 1)
	{
		int ref_sensor1 = 0;
		int ref_sensor2 = 0;
		while (ref_sensor2 != 1)
		{
			ref_sensor2 = zero_Hall2();
		}
		while (ref_sensor1 != 1)
		{
			ref_sensor1 = zero_Hall1();
		}
		sensorMax1 = sensorMax1 + 20;
		A = sensorMax1;
		H = highByte(A);
		L = lowByte(A);
		EEPROM.write(407, H);
		EEPROM.commit();
		EEPROM.write(408, L);
		EEPROM.commit();
		sensorMax2 = sensorMax2 + 20;
		A = sensorMax2;
		H = highByte(A);
		L = lowByte(A);
		EEPROM.write(409, H);
		EEPROM.commit();
		EEPROM.write(410, L);
		EEPROM.commit();
		sensorMin1 = sensorMin1 - 20;
		A = sensorMin1;
		H = highByte(A);
		L = lowByte(A);
		EEPROM.write(411, H);
		EEPROM.commit();
		EEPROM.write(412, L);
		EEPROM.commit();
		sensorMin2 = sensorMin2 - 20;
		A = sensorMin2;
		H = highByte(A);
		L = lowByte(A);
		EEPROM.write(413, H);
		EEPROM.commit();
		EEPROM.write(414, L);
		EEPROM.commit();
	}
	if (adjustIniFlag == 0)
	{
		L = EEPROM.read(403);
		H = EEPROM.read(404);
		level_zero1 = (L << 8) | H;

		L = EEPROM.read(405);
		H = EEPROM.read(406);
		level_zero2 = (L << 8) | H;

		L = EEPROM.read(407);
		H = EEPROM.read(408);
		sensorMax1 = (L << 8) | H;

		L = EEPROM.read(409);
		H = EEPROM.read(410);
		sensorMax2 = (L << 8) | H;

		L = EEPROM.read(411);
		H = EEPROM.read(412);
		sensorMin1 = (L << 8) | H;

		L = EEPROM.read(413);
		H = EEPROM.read(414);
		sensorMin2 = (L << 8) | H;
	}

	
	digitalWrite(DIR_PIN, LOW);
	for (int i = 0; i < 40; i++)
	{
		value_f = meanFilter2.AddValue(analogRead(hall1));
	}
	sensorRead1 = value_f / 4;
	mean = sensorRead1;
	if(mean > sensorMax1 || mean < sensorMin1)
	{
		move(200,1,6000);
	}
    
	flag = 0;
	digitalWrite(EN_PIN, LOW);
	digitalWrite(DIR_PIN, HIGH);
	tmcMotorA.rms_current(800);
	configTimerForMovement();
	delay(100);
	while (true)
	{
		digitalWrite(EN_PIN2, HIGH);
		digitalWrite(DIR_PIN2, LOW);
        cont_turn1++;
		if (analogRead(PIN_ProducType) > 3500)
		{
			if (digitalRead(DIAG_PIN) == 1 || cont_turn == 1480)
			{
				cont_turn1 = 0;
				flag = 1;
				//Back a little  first arm
				digitalWrite(DIR_PIN, LOW);
				move(300, 1, 5000);

				//Move second arm 90 degrees.
				digitalWrite(EN_PIN, LOW);
				digitalWrite(EN_PIN2, LOW);
				digitalWrite(DIR_PIN2, HIGH);
				move(2400, 2, 5000);
				//move both arms until encoder detects
				digitalWrite(DIR_PIN, HIGH);
				digitalWrite(DIR_PIN2, HIGH);

				for (int i = 0; i < 40; i++)
				{
					value_f = meanFilter2.AddValue(analogRead(hall1));
				}
				sensorRead1 = value_f / 4;

				while (sensorRead1 < sensorMax1 && sensorRead1 > sensorMin1)
				{
					for (int i = 0; i < 40; i++)
					{
						value_f = meanFilter2.AddValue(analogRead(hall1));
					}
					sensorRead1 = value_f / 4;

					flag = 2;
				}
				flag = 1;

				//move second arm until hall detects
				digitalWrite(DIR_PIN2, HIGH);
				for (int i = 0; i < 40; i++)
				{
					value_f = meanFilter2.AddValue(analogRead(hall2));
				}
				sensorRead2 = value_f / 4;
				mean = sensorRead2;
				while (mean > sensorMax2 || mean < sensorMin2)
				{
					for (int i = 0; i < 40; i++)
					{
						value_f = meanFilter2.AddValue(analogRead(hall2));
					}
					sensorRead2 = value_f / 4;
					mean = sensorRead2;
					flag = 3;
				}
				flag = 1;
				digitalWrite(DIR_PIN2, LOW);
				for (int i = 0; i < 40; i++)
				{
					value_f = meanFilter2.AddValue(analogRead(hall2));
				}
				sensorRead2 = value_f / 4;
				mean = sensorRead2;
				while (mean < sensorMax2 && mean > sensorMin2)
				{
					for (int i = 0; i < 40; i++)
					{
						value_f = meanFilter2.AddValue(analogRead(hall2));
					}
					sensorRead2 = value_f / 4;
					mean = sensorRead2;
					flag = 3;
					delay(10);
					cont_turn++;
					if (analogRead(PIN_ProducType) > 3500)
					{
						if ((digitalRead(DIAG_PIN2) == 1 and avoid2 == 1) || cont_turn == 1480)
						{
							cont_turn = 0;
							flag = 1;
							
							delay(250);
							digitalWrite(DIR_PIN, LOW);
							move(540, 1, 5000);
							//second arm 90 degree
							digitalWrite(DIR_PIN2, HIGH);
							move(3200, 2, 2000);

							//move both arms at same time.
							digitalWrite(DIR_PIN, HIGH);
							digitalWrite(DIR_PIN2, HIGH);

							for (int i = 0; i < 40; i++)
							{
								value_f = meanFilter2.AddValue(analogRead(hall1));
							}
							sensorRead1 = value_f / 4;

							while (sensorRead1 < sensorMax1 && sensorRead1 > sensorMin1)
							{
								for (int i = 0; i < 40; i++)
								{
									value_f = meanFilter2.AddValue(analogRead(hall1));
								}
								sensorRead1 = value_f / 4;
								flag = 2;
							}
							digitalWrite(DIR_PIN2, LOW);
							//move second arm ultil hall
							while (mean < sensorMax2 && mean > sensorMin2)
							{
								for (int i = 0; i < 40; i++)
								{
									value_f = meanFilter2.AddValue(analogRead(hall2));
								}
								sensorRead2 = value_f / 4;
								mean = sensorRead2;
								flag = 3;
							}
							flag = 1;
							break;
						}

						if (digitalRead(DIAG_PIN2) == 1 and avoid2 == 0)
						{
							digitalWrite(DIR_PIN2, LOW);
							avoid2 = 1;
							delay(100);
							
						}
					}
				}
				flag = 1;
				
				if (adjustIniFlag == 1)
				{
					sensorPole1 = Pole1();
					EEPROM.write(ADDRESSPOLESENSE1, sensorPole1);
					EEPROM.commit();
					sensorPole2 = Pole2();
					EEPROM.write(ADDRESSPOLESENSE2, sensorPole2);
					EEPROM.commit();
				}
				if (adjustIniFlag == 0)
				{
					sensorPole1 = EEPROM.read(ADDRESSPOLESENSE1);
					sensorPole2 = EEPROM.read(ADDRESSPOLESENSE2);
				}
				if (sensorPole1 == 1)
				{
						findingSlowSensor1();
				}
				if (sensorPole1 == 0)
				{
						findingSlowSensor1Negative();
				}
				if (sensorPole2 == 1)
				{
						findingSlowSensor2();
				}
				if (sensorPole2 == 0)
				{
						findingSlowSensor2Negative();
				}
				tmcMotorA.rms_current(CURRENT_IN_CALIBRATION);
				tmcMotorB.rms_current(CURRENT_IN_CALIBRATION);
				digitalWrite(EN_PIN, LOW);
				digitalWrite(EN_PIN2, LOW);
				avoid = 1;
				return 0;
			}
		}
		//====WITHOUT STALLGUARD====
		for (int i = 0; i < 40; i++)
		{
			value_f = meanFilter2.AddValue(analogRead(hall1));
		}
		sensorRead1 = value_f / 4;
		if (sensorRead1 > sensorMax1 || sensorRead1 < sensorMin1) //stop first arm condition.
		{
			if (avoid == 0)
			{
				flag = 1;
				digitalWrite(DIR_PIN2, HIGH);
				for (int i = 0; i < 40; i++)
				{
					value_f = meanFilter2.AddValue(analogRead(hall2));
				}
				sensorRead2 = value_f / 4;
				mean = sensorRead2;
				digitalWrite(EN_PIN2, LOW); 
				while (mean > sensorMax2 || mean < sensorMin2)
				{
					for (int i = 0; i < 40; i++)
					{
						value_f = meanFilter2.AddValue(analogRead(hall2));
					}
					sensorRead2 = value_f / 4;
					mean = sensorRead2;
					flag = 3;
				}
				flag = 1;
				digitalWrite(DIR_PIN, HIGH);
				digitalWrite(DIR_PIN2, LOW);
				for (int i = 0; i < 40; i++)
				{
					value_f = meanFilter2.AddValue(analogRead(hall2));
				}
				sensorRead2 = value_f / 4;
				mean = sensorRead2;
				digitalWrite(EN_PIN2, LOW);
				digitalWrite(EN_PIN, LOW);
				tmcMotorA.rms_current(1500);
				delay(500);
				while (mean < sensorMax2 && mean > sensorMin2)
				{
					for (int i = 0; i < 40; i++)
					{
						value_f = meanFilter2.AddValue(analogRead(hall2));
					}
					sensorRead2 = value_f / 4;
					mean = sensorRead2;
					flag = 3;
					delay(10);
					cont_turn++;
					if (analogRead(PIN_ProducType) > 3500)
					{

						if ((digitalRead(DIAG_PIN2) == 1 and avoid2 == 1) || cont_turn == 1480)
						{
							cont_turn = 0;
							flag = 1;
							//secuence where arm is between both pikes//regresa main arm 30 degrees, the second one turns 90, and both move together.
							//main arm 30 degree
							delay(250);
							digitalWrite(EN_PIN2, HIGH);
							digitalWrite(DIR_PIN, LOW);
							move(540, 1, 5000);
							//second arm 90 degree
							digitalWrite(EN_PIN2, LOW);
							digitalWrite(DIR_PIN2, HIGH);
							move(3200, 2, 2000);

							//move both arms at same time.
							digitalWrite(DIR_PIN, HIGH);
							digitalWrite(DIR_PIN2, HIGH);
							for (int i = 0; i < 40; i++)
							{
								value_f = meanFilter2.AddValue(analogRead(hall1));
							}
							sensorRead1 = value_f / 4;
							while (sensorRead1 < sensorMax1 && sensorRead1 > sensorMin1)
							{
								for (int i = 0; i < 40; i++)
								{
									value_f = meanFilter2.AddValue(analogRead(hall1));
								}
								sensorRead1 = value_f / 4;
								flag = 2;
							}
							digitalWrite(DIR_PIN2, LOW);
							//move second arm ultil hall
							while (mean < sensorMax2 && mean > sensorMin2)
							{
								for (int i = 0; i < 40; i++)
								{
									value_f = meanFilter2.AddValue(analogRead(hall2));
								}
								sensorRead2 = value_f / 4;
								mean = sensorRead2;
								flag = 3;
							}
							
							flag = 1;
							break;
						}

						if (digitalRead(DIAG_PIN2) == 1 and avoid2 == 0)
						{
							digitalWrite(DIR_PIN2, LOW);
							avoid2 = 1;
							delay(100);
							
						}
					}
				}
				flag = 1;

				if (adjustIniFlag == 1)
				{
					sensorPole1 = Pole1();
					EEPROM.write(ADDRESSPOLESENSE1, sensorPole1);
					EEPROM.commit();
					sensorPole2 = Pole2();
#ifdef DEBUGGING_DATA
                        Serial.println("Polo 1");
                        Serial.println(sensorPole1);
                    #endif
                    #ifdef DEBUGGING_DATA
                        Serial.println("Polo 2");
                        Serial.println(sensorPole2);
                    #endif
					EEPROM.write(ADDRESSPOLESENSE2, sensorPole2);
					EEPROM.commit();
				}
				if (adjustIniFlag == 0)
				{
					sensorPole1 = EEPROM.read(ADDRESSPOLESENSE1);
					sensorPole2 = EEPROM.read(ADDRESSPOLESENSE2);
                    #ifdef DEBUGGING_DATA
                        Serial.println("Polo 1");
                        Serial.println(sensorPole1);
                    #endif
                    #ifdef DEBUGGING_DATA
                        Serial.println("Polo 2");
                        Serial.println(sensorPole2);
                    #endif
				}

				if (sensorPole1 == 1)
				{
						findingSlowSensor1();
				}
				if (sensorPole1 == 0)
				{	
						findingSlowSensor1Negative();
				}
				if (sensorPole2 == 1)
				{
						findingSlowSensor2();
				}
				if (sensorPole2 == 0)
				{
						findingSlowSensor2Negative();
				}
				tmcMotorA.rms_current(CURRENT_IN_CALIBRATION);
				tmcMotorB.rms_current(CURRENT_IN_CALIBRATION);
				avoid = 1;
				return 0;
			}
		}
	}
}

void configTimerForMovement()
{
	{
		cli();										  //stop interrupts
		timer1 = timerBegin(3, 8, true);			  // Se configura el timer, en este caso uso el timer 4 de los 4 disponibles en el ESP(0,1,2,3)
													  // el prescaler es 8, y true es una Flagera que indica si la interrupcion se realiza en borde o en level
		timerAttachInterrupt(timer1, &onTimer, true); //Se vincula el timer con la funcion AttachInterrup
													  //la cual se ejecuta cuando se genera la interrupcion
		timerAlarmWrite(timer1, 10000, true);		  //En esta funcion se define el valor del contador en el cual se genera la interrupción del timer
		timerAlarmEnable(timer1);					  //Función para habilitar el temporizador.
		sei();										  //allow interrupts
	}
}

/**
 * @brief Esta funcion continua con el proceso de calibracion una vez que el brazo 1 esta cerca del sensor hall.
 *  
 * Inicia moviendo el brazo uno 80 pasos en sentido antihorario para salir por completo del rango del sensor
 * Comienza a tomar mediciones del sensor en cada paso que avanza en sentido horario hasta llenar el vector value[]
 * Durante el llenado del vector tambien se determina el valor maximo entre todos los elementos del  vector
 * La grafica de los datos tiene forma gaussiana por lo cual se determina un limite del rango en cual se buscara el punto medio de la funcion,
 * dicho rango se establecio como los valores mayores al 90% del valor maximo encontrado.
 * Se determinan los pasos correspondientes a el limite derecho y el limite izquierdo de la funcion
 * Se calculan los pasos necesarios para llegar al punto medio del sensor
 * @param value[] En este vector se alamacenan los valores obtenidos del sensor Hall.
 * @param limit En esta variable se define el limite del intervalo en el cual se buscara el punto medio de los datos almacenados en el vector value.
 * @param steps_ini Almacena el numero de pasos para llegar al inicio del intervalo de busqueda.
 * @param steps_fin Almacena el numero de pasos para llegar al fin del intervalo de busqueda.
 * @param pas Esta variable almacena los pasos necesarios para posicionar el brazo 1 al centro del sensor hall.
 */
void findingSlowSensor1()
{
	int value_f;
	digitalWrite(EN_PIN, LOW);
	for (int i = 0; i < 80; i++)
	{
		value[i] = -1;
	}
	for (int j = 0; j < 80; j++)
	{
		value_r[j] = -1;
	}
	int k = 0;
	int val_sens;
	int Flag_ini = 0;
	int Flag_fin = 0;
	int steps_ini;
	int steps_fin;
	int steps_ini2;
	int steps_fin2;
	int limit;
	int count_ini = 0;
	int count_fin = 0;
	int maximum2_r = 0;
	int index_min1;
	int index_min2;
	int ind_2;
	int offset;
	int cont_t1 = 0;
	int cont_t2 = 0;
	digitalWrite(DIR_PIN, HIGH);
	for (int i = 0; i < 40; i++)
	{
		value_f = meanFilter2.AddValue(analogRead(hall1));
	}
	sensorRead1 = value_f / 4;
    //=====
	while (sensorRead1 < sensorMax1)
	{
		move(1, 1, 1000);
		for (int y = 0; y < 40; y++)
		{
			sensorRead1 = (analogRead(hall1)) / 4;
			sensorRead1 = meanFilter2.AddValue(sensorRead1);
		}
	}

	digitalWrite(DIR_PIN, HIGH);
	for (int i = 0; i < 40; i++)
	{
		value_f = meanFilter2.AddValue(analogRead(hall1));
	}
	sensorRead1 = value_f / 4;
	for (int t = 0; t < 80; t++)
	{
		for (int i = 0; i < 40; i++)
		{
			value_f = meanFilter2.AddValue(analogRead(hall1));
		}
		sensorRead1 = value_f / 4;
		move(1, 1, 1000);

		ind_2 = 79 - t;
		value_r[ind_2] = sensorRead1;
		if (value_r[ind_2] > sensorMax1)
		{
			cont_t2++;
		}

		if (sensorRead1 > maximum2_r)
		{
			maximum2_r = sensorRead1;
			index_min1 = ind_2;
		}
	}

	digitalWrite(DIR_PIN, LOW);

	for (int y = 0; y < 40; y++)
	{
		sensorRead1 = (analogRead(hall1)) / 4;
		val_sens = meanFilter2.AddValue(sensorRead1);
	}

	while (val_sens < sensorMax1)
	{
		move(1, 1, 1000);
		for (int y = 0; y < 40; y++)
		{
			sensorRead1 = (analogRead(hall1)) / 4;
			val_sens = meanFilter2.AddValue(sensorRead1);
		}
	}

	while (k < 80)
	{
		move(1, 1, 1000);
		for (int y = 0; y < 40; y++)
		{
			sensorRead1 = (analogRead(hall1)) / 4;
			val_sens = meanFilter2.AddValue(sensorRead1);
		}
		delay(1);

		if (val_sens > maximum2)
		{
			maximum2 = val_sens;
			index_min2 = k;
		}
		value[k] = val_sens;

		if (value[k] > sensorMax1)
		{
			cont_t1++;
		}
		delay(1);
		k++;
	}
	//====
	limit = maximum * 0.9;
	k = 79;
	while (k >= 0)
	{
		if (value[k] > limit && Flag_ini == 0)
		{
			count_ini++;
			if (count_ini == 9)
			{
				steps_ini = k + 9;
				Flag_ini = 1;
			}
		}
		if (value[k] < limit && Flag_ini == 1 && Flag_fin == 0)
		{
			count_fin++;
			if (count_fin == 9)
			{
				steps_fin = k + 9;
				Flag_fin = 1;
			}
		}
		k--;
	}

	Flag_ini = 0;
	Flag_fin = 0;
	count_ini = 0;
	count_fin = 0;

	k = 79;
	while (k >= 0)
	{
		if (value_r[k] > limit && Flag_ini == 0)
		{
			count_ini++;
			if (count_ini == 9)
			{
				steps_ini2 = k + 9;
				Flag_ini = 1;
			}
		}
		if (value_r[k] < limit && Flag_ini == 1 && Flag_fin == 0)
		{
			count_fin++;
			if (count_fin == 9)
			{
				steps_fin2 = k + 9;
				Flag_fin = 1;
			}
		}
		k--;
	}

	digitalWrite(DIR_PIN, HIGH);
	int pas;
	int half;
	half = (cont_t1 + cont_t2) / 2;

	int half1;

	half1 = (half) / 2;

	offset = (steps_ini2 - steps_ini) / 2;

	digitalWrite(DIR_PIN, HIGH);
	for (int i = 0; i < 40; i++)
	{
		value_f = meanFilter2.AddValue(analogRead(hall1));
	}
	sensorRead1 = value_f / 4;
	while (sensorRead1 < sensorMax1)
	{
		move(1, 1, 1000);
		for (int y = 0; y < 40; y++)
		{
			sensorRead1 = (analogRead(hall1)) / 4;
			sensorRead1 = meanFilter2.AddValue(sensorRead1);
		}
	}
	//delay(1000);
	pas = half1 + 2;
	move(pas, 1, 1000);
}

/**
 * @brief Esta funcion continua con el proceso de calibracion una vez que el brazo 2 esta cerca del sensor hall.
 * DESCRIPCION GENERAL
 * Inicia moviendose 100 pasos en sentido antihorario
 * Regresa en sentido horario hasta que es detectado por el sensor
 * Continua moviendose en sentido horario hasta completar 600 pasos,  en cada paso toma la medicion del sensor y este dato se almacena en el vector value2_r[]
 * Regresa el brazo en sentido antihorario hasta que es detectado por el sensor
 * Continua moviendose en sentido antihorario hasta completar 600 pasos, en cada paso toma la medicion del sensor y este dato se almacena en el vector value2[]
 * La grafica de ambos vectores es de forma gaussiana por lo cual se determina un limite del rango de busqueda en ambas funciones,
 * dicho rango se establecio como los valores mayores al 90% del valor maximo encontrado.
 * Se determinan los pasos correspondientes a el limite derecho y el limite izquierdo de ambas funciones y se usa el limite derecho de la funcion dos
 *  y el limite izquierdo de la funcion uno para encontrar el punto medio. 
 * Se calculan los pasos necesarios para llegar al punto medio del sensor
 * 
 * @param value2[] En este vector se alamacenan los valores obtenidos del sensor Hall.
 * @param limit En esta variable se define el limite del intervalo en el cual se buscara el punto medio de los datos almacenados en el vector value.
 * @param steps_ini Almacena el numero de pasos para llegar al inicio del intervalo de busqueda.
 * @param steps_fin Almacena el numero de pasos para llegar al fin del intervalo de busqueda.
 * @param pas Esta variable almacena los pasos necesarios para posicionar el brazo 2 al centro del sensor hall.
 */
void findingSlowSensor2()
{
	int value2_f;
	digitalWrite(EN_PIN2, LOW);
	for (int i = 0; i < 300; i++)
	{
		value2[i] = -1;
	}
	for (int j = 0; j < 300; j++)
	{
		value2_r[j] = -1;
	}
	int k = 0;
	int val_sens;
	int Flag_ini = 0;
	int Flag_fin = 0;
	int steps_ini;
	int steps_fin;
	int steps_ini2;
	int steps_fin2;
	int limit;
	int count_ini = 0;
	int count_fin = 0;
	int maximum2_r = 0;
	int index_min1;
	int index_min2;
	int ind_2;
	int offset;
	int cont_t1 = 0;
	int cont_t2 = 0;

	//digitalWrite(DIR_PIN2, HIGH);
	//move(100, 2, 8000);
	digitalWrite(DIR_PIN2, LOW);
	for (int i = 0; i < 40; i++)
	{
		value2_f = meanFilter2.AddValue(analogRead(hall2));
	}
	sensorRead2 = value2_f / 4;
	while (sensorRead2 < sensorMax2)
	{
		move(1, 2, 500);
		for (int y = 0; y < 40; y++)
		{
			sensorRead2 = (analogRead(hall2)) / 4;
			sensorRead2 = meanFilter2.AddValue(sensorRead2);
		}
	}
	//delay(1000);

	digitalWrite(DIR_PIN2, LOW);
	for (int i = 0; i < 40; i++)
	{
		value2_f = meanFilter2.AddValue(analogRead(hall2));
	}
	sensorRead2 = value2_f / 4;
	for (int t = 0; t < 300; t++)
	{
		for (int i = 0; i < 40; i++)
		{
			value2_f = meanFilter2.AddValue(analogRead(hall2));
		}
		sensorRead2 = value2_f / 4;
		move(1, 2, 500);

		ind_2 = 299 - t;
		value2_r[ind_2] = sensorRead2;
		if (value2_r[ind_2] > sensorMax2)
		{
			cont_t2++;
		}

		if (sensorRead2 > maximum2_r)
		{
			maximum2_r = sensorRead2;
			index_min1 = ind_2;
		}
	}

	digitalWrite(DIR_PIN2, HIGH);

	for (int y = 0; y < 40; y++)
	{
		sensorRead2 = (analogRead(hall2)) / 4;
		val_sens = meanFilter2.AddValue(sensorRead2);
	}

	while (val_sens < sensorMax2)
	{
		move(1, 2, 500);
		for (int y = 0; y < 40; y++)
		{
			sensorRead2 = (analogRead(hall2)) / 4;
			val_sens = meanFilter2.AddValue(sensorRead2);
		}
	}
	//delay(1000);

	while (k < 300)
	{
		move(1, 2, 500);
		for (int y = 0; y < 40; y++)
		{
			sensorRead2 = (analogRead(hall2)) / 4;
			val_sens = meanFilter2.AddValue(sensorRead2);
		}
		delay(1);

		if (val_sens > maximum2)
		{
			maximum2 = val_sens;
			index_min2 = k;
		}
		value2[k] = val_sens;

		if (value2[k] > sensorMax2)
		{
			cont_t1++;
		}
		delay(1);
		k++;
	}

	limit = maximum2 * 0.9;
	k = 299;
	while (k >= 0)
	{
		if (value2[k] > limit && Flag_ini == 0)
		{
			count_ini++;
			if (count_ini == 9)
			{
				steps_ini = k + 9;
				Flag_ini = 1;
			}
		}
		if (value2[k] < limit && Flag_ini == 1 && Flag_fin == 0)
		{
			count_fin++;
			if (count_fin == 9)
			{
				steps_fin = k + 9;
				Flag_fin = 1;
			}
		}
		k--;
	}

	Flag_ini = 0;
	Flag_fin = 0;
	count_ini = 0;
	count_fin = 0;

	k = 299;
	while (k >= 0)
	{
		if (value2_r[k] > limit && Flag_ini == 0)
		{
			count_ini++;
			if (count_ini == 9)
			{
				steps_ini2 = k + 9;
				Flag_ini = 1;
			}
		}
		if (value2_r[k] < limit && Flag_ini == 1 && Flag_fin == 0)
		{
			count_fin++;
			if (count_fin == 9)
			{
				steps_fin2 = k + 9;
				Flag_fin = 1;
			}
		}
		k--;
	}

	digitalWrite(DIR_PIN2, LOW);
	int pas;
	int half;
	half = (cont_t1 + cont_t2) / 2;

	int half1;

	half1 = (half) / 2;

	offset = (steps_ini2 - steps_ini) / 2;

	digitalWrite(DIR_PIN2, LOW);
	for (int i = 0; i < 40; i++)
	{
		value2_f = meanFilter2.AddValue(analogRead(hall2));
	}
	sensorRead2 = value2_f / 4;
	while (sensorRead2 < sensorMax2)
	{
		move(1, 2, 500);
		for (int y = 0; y < 40; y++)
		{
			sensorRead2 = (analogRead(hall2)) / 4;
			sensorRead2 = meanFilter2.AddValue(sensorRead2);
		}
	}
	//delay(1000);
	pas = half1;
	move(pas, 2, 500);
}

/**
 * @brief Esta funcion continua con el proceso de calibracion una vez que el brazo 1 esta cerca del sensor hall, esta version de la funcion contempla el polo negativo del iman 
 * DESCRIPCION GENERAL 
 * Inicia moviendo el brazo uno 80 pasos en sentido antihorario para salir por completo del rango del sensor
 * Comienza a tomar mediciones del sensor en cada paso que avanza en sentido horario hasta llenar el vector value[]
 * Durante el llenado del vector tambien se determina el valor maximo entre todos los elementos del  vector
 * La grafica de los datos tiene forma gaussiana por lo cual se determina un limite del rango en cual se buscara el punto medio de la funcion,
 *   dicho rango se establecio como los valores mayores al 90% del valor maximo encontrado.
 * Se determinan los pasos correspondientes a el limite derecho y el limite izquierdo de la funcion
 * Se calculan los pasos necesarios para llegar al punto medio del sensor
 * @param value[] En este vector se alamacenan los valores obtenidos del sensor Hall.
 * @param limit En esta variable se define el limite del intervalo en el cual se buscara el punto medio de los datos almacenados en el vector value.
 * @param steps_ini Almacena el numero de pasos para llegar al inicio del intervalo de busqueda.
 * @param steps_fin Almacena el numero de pasos para llegar al fin del intervalo de busqueda.
 * @param pas Esta variable almacena los pasos necesarios para posicionar el brazo 1 al centro del sensor hall.
 */
void findingSlowSensor1Negative()
{
	int value1_f;
	digitalWrite(EN_PIN, LOW);
	for (int i = 0; i < 80; i++)
	{
		value[i] = -1;
	}
	for (int j = 0; j < 80; j++)
	{
		value_r[j] = -1;
	}
	int k = 0;
	int val_sens;
	int Flag_ini = 0;
	int Flag_fin = 0;
	int steps_ini;
	int steps_fin;
	int steps_ini2;
	int steps_fin2;
	int limit;
	int count_ini = 0;
	int count_fin = 0;
	int minimum2_r = 5000;
	int index_min1;
	int index_min2;
	int ind_2;
	int offset;
	int cont_t1 = 0;
	int cont_t2 = 0;
	digitalWrite(DIR_PIN, HIGH);
	for (int i = 0; i < 40; i++)
	{
		value1_f = meanFilter2.AddValue(analogRead(hall1));
	}
	sensorRead1 = value1_f / 4;
	//======================================================
	while (sensorRead1 > sensorMin1)
	{
		move(1, 1, 1000);
		for (int y = 0; y < 40; y++)
		{
			sensorRead1 = (analogRead(hall1)) / 4;
			sensorRead1 = meanFilter2.AddValue(sensorRead1);
		}
	}
	//delay(1000);

	digitalWrite(DIR_PIN, HIGH);
	for (int i = 0; i < 40; i++)
	{
		value1_f = meanFilter2.AddValue(analogRead(hall1));
	}
	sensorRead1 = value1_f / 4;
	for (int t = 0; t < 80; t++)
	{
		for (int i = 0; i < 40; i++)
		{
			value1_f = meanFilter2.AddValue(analogRead(hall1));
		}
		sensorRead1 = value1_f / 4;
		move(1, 1, 1000);

		ind_2 = 79 - t;
		value_r[ind_2] = sensorRead1;
		if (value_r[ind_2] < sensorMin1)
		{
			cont_t2++;
		}

		if (sensorRead1 < minimum2_r)
		{
			minimum2_r = sensorRead1;
			index_min1 = ind_2;
		}
	}

	digitalWrite(DIR_PIN, LOW);

	for (int y = 0; y < 40; y++)
	{
		sensorRead1 = (analogRead(hall1)) / 4;
		val_sens = meanFilter2.AddValue(sensorRead1);
	}

	while (val_sens > sensorMin1)
	{
		move(1, 1, 1000);
		for (int y = 0; y < 40; y++)
		{
			sensorRead1 = (analogRead(hall1)) / 4;
			val_sens = meanFilter2.AddValue(sensorRead1);
		}
	}

	while (k < 80)
	{
		move(1, 1, 1000);
		for (int y = 0; y < 40; y++)
		{
			sensorRead1 = (analogRead(hall1)) / 4;
			val_sens = meanFilter2.AddValue(sensorRead1);
		}
		delay(1);

		if (val_sens < minimum2)
		{
			minimum2 = val_sens;
			index_min2 = k;
		}
		value[k] = val_sens;

		if (value[k] < sensorMin1)
		{
			cont_t1++;
		}
		delay(1);
		k++;
	}

	//====
	limit = minimum + ((level_zero1 - minimum) * 0.1);
	k = 79;
	while (k >= 0)
	{
		if (value[k] < limit && Flag_ini == 0)
		{
			count_ini++;
			if (count_ini == 9)
			{
				steps_ini = k + 9;
				Flag_ini = 1;
			}
		}
		if (value[k] > limit && Flag_ini == 1 && Flag_fin == 0)
		{
			count_fin++;
			if (count_fin == 9)
			{
				steps_fin = k + 9;
				Flag_fin = 1;
			}
		}
		k--;
	}

	Flag_ini = 0;
	Flag_fin = 0;
	count_ini = 0;
	count_fin = 0;

	k = 79;
	while (k >= 0)
	{
		if (value_r[k] < limit && Flag_ini == 0)
		{
			count_ini++;
			if (count_ini == 9)
			{
				steps_ini2 = k + 9;
				Flag_ini = 1;
			}
		}
		if (value_r[k] > limit && Flag_ini == 1 && Flag_fin == 0)
		{
			count_fin++;
			if (count_fin == 9)
			{
				steps_fin2 = k + 9;
				Flag_fin = 1;
			}
		}
		k--;
	}

	digitalWrite(DIR_PIN, HIGH);
	int pas;
	int half;
	half = (cont_t1 + cont_t2) / 2;

	int half1;

	half1 = (half) / 2;

	offset = (steps_ini2 - steps_ini) / 2;

	digitalWrite(DIR_PIN, HIGH);
	for (int i = 0; i < 40; i++)
	{
		value1_f = meanFilter2.AddValue(analogRead(hall1));
	}
	sensorRead1 = value1_f / 4;
	while (sensorRead1 > sensorMin1)
	{
		move(1, 1, 1000);
		for (int y = 0; y < 40; y++)
		{
			sensorRead1 = (analogRead(hall1)) / 4;
			sensorRead1 = meanFilter2.AddValue(sensorRead1);
		}
	}
	pas = half1 + 2;
	move(pas, 1, 1000);
}

/**
 * @brief Esta funcion continua con el proceso de calibracion una vez que el brazo 2 esta cerca del sensor hall, esta version de la funcion contempla el polo negativo del iman 
 * DESCRIPCION GENERAL
 * Inicia moviendose 100 pasos en sentido antihorario
 * Regresa en sentido horario hasta que es detectado por el sensor
 * Continua moviendose en sentido horario hasta completar 600 pasos,  en cada paso toma la medicion del sensor y este dato se almacena en el vector value2_r[]
 * Regresa el brazo en sentido antihorario hasta que es detectado por el sensor
 * Continua moviendose en sentido antihorario hasta completar 600 pasos, en cada paso toma la medicion del sensor y este dato se almacena en el vector value2[]
 * La grafica de ambos vectores es de forma gaussiana por lo cual se determina un limite del rango de busqueda en ambas funciones,
 * dicho rango se establecio como los valores mayores al 90% del valor maximo encontrado.
 * Se determinan los pasos correspondientes a el limite derecho y el limite izquierdo de ambas funciones y se usa el limite derecho de la funcion dos
 *  y el limite izquierdo de la funcion uno para encontrar el punto medio. 
 * Se calculan los pasos necesarios para llegar al punto medio del sensor
 * @param value2[] En este vector se alamacenan los valores obtenidos del sensor Hall.
 * @param limit En esta variable se define el limite del intervalo en el cual se buscara el punto medio de los datos almacenados en el vector value.
 * @param steps_ini Almacena el numero de pasos para llegar al inicio del intervalo de busqueda.
 * @param steps_fin Almacena el numero de pasos para llegar al fin del intervalo de busqueda.
 * @param pas Esta variable almacena los pasos necesarios para posicionar el brazo 2 al centro del sensor hall.
 */
void findingSlowSensor2Negative()
{
	int value2_f;
	digitalWrite(EN_PIN2, LOW);
	for (int i = 0; i < 300; i++)
	{
		value2[i] = -1;
	}
	for (int j = 0; j < 300; j++)
	{
		value2_r[j] = -1;
	}
	int k = 0;
	int val_sens;
	int Flag_ini = 0;
	int Flag_fin = 0;
	int steps_ini;
	int steps_fin;
	int steps_ini2;
	int steps_fin2;
	int limit;
	int count_ini = 0;
	int count_fin = 0;
	int minimum2_r = 5000;
	int index_min1;
	int index_min2;
	int ind_2;
	int offset;
	int cont_t1 = 0;
	int cont_t2 = 0;

	//digitalWrite(DIR_PIN2, HIGH);
	//move(100, 2, 8000);
	digitalWrite(DIR_PIN2, LOW);
	for (int i = 0; i < 40; i++)
	{
		value2_f = meanFilter2.AddValue(analogRead(hall2));
	}
	sensorRead2 = value2_f / 4;
	while (sensorRead2 > sensorMin2)
	{
		move(1, 2, 500);
		for (int y = 0; y < 40; y++)
		{
			sensorRead2 = (analogRead(hall2)) / 4;
			sensorRead2 = meanFilter2.AddValue(sensorRead2);
		}
	}
	//delay(1000);

	digitalWrite(DIR_PIN2, LOW);
	for (int i = 0; i < 40; i++)
	{
		value2_f = meanFilter2.AddValue(analogRead(hall2));
	}
	sensorRead2 = value2_f / 4;
	for (int t = 0; t < 300; t++)
	{
		for (int i = 0; i < 40; i++)
		{
			value2_f = meanFilter2.AddValue(analogRead(hall2));
		}
		sensorRead2 = value2_f / 4;
		move(1, 2, 500);

		ind_2 = 299 - t;
		value2_r[ind_2] = sensorRead2;
		if (value2_r[ind_2] < sensorMin2)
		{
			cont_t2++;
		}

		if (sensorRead2 < minimum2_r)
		{
			minimum2_r = sensorRead2;
			index_min1 = ind_2;
		}
	}

	digitalWrite(DIR_PIN2, HIGH);

	for (int y = 0; y < 40; y++)
	{
		sensorRead2 = (analogRead(hall2)) / 4;
		val_sens = meanFilter2.AddValue(sensorRead2);
	}

	while (val_sens > sensorMin2)
	{
		move(1, 2, 500);
		for (int y = 0; y < 40; y++)
		{
			sensorRead2 = (analogRead(hall2)) / 4;
			val_sens = meanFilter2.AddValue(sensorRead2);
		}
	}
	//delay(1000);

	while (k < 300)
	{
		move(1, 2, 500);
		for (int y = 0; y < 40; y++)
		{
			sensorRead2 = (analogRead(hall2)) / 4;
			val_sens = meanFilter2.AddValue(sensorRead2);
		}
		delay(1);

		if (val_sens < minimum2)
		{
			minimum2 = val_sens;
			index_min2 = k;
		}
		value2[k] = val_sens;

		if (value2[k] < sensorMin2)
		{
			cont_t1++;
		}

		delay(1);
		k++;
	}

	limit = minimum2 + ((level_zero2 - minimum2) * 0.1);
	k = 299;
	while (k >= 0)
	{
		if (value2[k] < limit && Flag_ini == 0)
		{
			count_ini++;
			if (count_ini == 9)
			{
				steps_ini = k + 9;
				Flag_ini = 1;
			}
		}
		if (value2[k] > limit && Flag_ini == 1 && Flag_fin == 0)
		{
			count_fin++;
			if (count_fin == 9)
			{
				steps_fin = k + 9;
				Flag_fin = 1;
			}
		}
		k--;
	}

	Flag_ini = 0;
	Flag_fin = 0;
	count_ini = 0;
	count_fin = 0;

	k = 299;
	while (k >= 0)
	{
		if (value2_r[k] < limit && Flag_ini == 0)
		{
			count_ini++;
			if (count_ini == 9)
			{
				steps_ini2 = k + 9;
				Flag_ini = 1;
			}
		}
		if (value2_r[k] > limit && Flag_ini == 1 && Flag_fin == 0)
		{
			count_fin++;
			if (count_fin == 9)
			{
				steps_fin2 = k + 9;
				Flag_fin = 1;
			}
		}
		k--;
	}

	digitalWrite(DIR_PIN2, LOW);
	int pas;
	int half;
	half = (cont_t1 + cont_t2) / 2;

	int half1;

	half1 = (half) / 2;

	offset = (steps_ini2 - steps_ini) / 2;

	digitalWrite(DIR_PIN2, LOW);
	for (int i = 0; i < 40; i++)
	{
		value2_f = meanFilter2.AddValue(analogRead(hall2));
	}
	sensorRead2 = value2_f / 4;
	while (sensorRead2 > sensorMin2)
	{
		move(1, 2, 500);
		for (int y = 0; y < 40; y++)
		{
			sensorRead2 = (analogRead(hall2)) / 4;
			sensorRead2 = meanFilter2.AddValue(sensorRead2);
		}
	}
	pas = half1;
	move(pas, 2, 500);
}

/**
 * @brief Esta funcion perimite mover los motores de manera controlada mediante numero de pasos. 
 * @param pasos Esta variable indica el numero de pasos que se desea avanzar.
 * @param motor_d Indica el motor que se quiere mover, opcion 1 o 2.
 * @param Speed Indica la velocidad del giro, si se disminuye el valor se aumenta la velocidad y si se aumenta este valor la velocidad disminuye .
 */
void move(int pasos, int motor_d, int Speed)
{
	if (motor_d == 1)
	{
		for (int i = 0; i < pasos; i++)
		{
			digitalWrite(STEP_PIN, HIGH);
			delayMicroseconds(Speed);
			digitalWrite(STEP_PIN, LOW);
			delayMicroseconds(Speed);
		}
	}

	if (motor_d == 2)
	{
		for (int j = 0; j < pasos; j++)
		{
			digitalWrite(STEP_PIN2, HIGH);
			delayMicroseconds(Speed);
			digitalWrite(STEP_PIN2, LOW);
			delayMicroseconds(Speed);
		}
	}
}

/**
 * @brief Esta funcion se utiliza entre programas para verificar que la siguiente secuencia iniciara en el punto cero.
 * en este caso la funcion contempla solo al brazo 1 con la polaridad del iman positiva. 
 * DESCRIPCION GENERAL
 * En la primera condicion de esta funcion se determina si el brazo se encuentra fuera del rango del sensor
 * En caso de entrar en esta condicion inicia una busqueda con rango maximo de 200 pasos en sentido horario
 * Si no se encuentra nada en ese rango inicia una busqueda en sentido antihorario con un rango maximo de 400 pasos
 * Si encuentra el sensor en la primera busqueda se activa una bandera para determinar que esta posicionado del lado izquierdo del sensor 
 * posteriormente se mueve 100 pasos en sentido horario y regresa en sentido antihorario para posicionarse al inicio del sensor pero del lado derecho.
 * Existe otro caso en el cual al iniciar la funcion el brazo ya se encuentra dentro del rango del sensor, en ese caso se mueve 100 pasos en sentido 
 * horario para garantizar que sale del sensor y posteriormente regresa en sentido antihorario hasta que es detectado por el sensor posicionandose asi
 * del lado derecho del sensor.
 * Finalmente cuando ya esta posicionado al inicio del lado derecho del sensor realiza el proceso de slow calibration para centrarse correctamente.
 */
void verif_cal_positiveb1(void)
{
	digitalWrite(EN_PIN2, LOW);
	digitalWrite(EN_PIN, LOW);
	int value1_f;
	flag = 1;
	int busq = 0;
	int Flag_b = 0;
	int Flag_der = 0;
	int Flag_izq = 0;

	digitalWrite(DIR_PIN, LOW);
	digitalWrite(DIR_PIN2, LOW);

	for (int i = 0; i < 40; i++)
	{
		value1_f = meanFilter2.AddValue(analogRead(hall1));
	}
	sensorRead1 = value1_f / 4;
	if (sensorRead1 < sensorMax1)
	{
		while (busq < 200)
		{
			move(1, 1, 5000);
			move(1, 2, 5000);
			busq++;

			for (int i = 0; i < 40; i++)
			{
				value1_f = meanFilter2.AddValue(analogRead(hall1));
			}
			sensorRead1 = value1_f / 4;

			if (sensorRead1 > sensorMax1)
			{
				busq = 200;
				Flag_b = 1;
				Flag_izq = 1;
			}
		}
		if (Flag_b == 0)
		{
			busq = 0;
			digitalWrite(DIR_PIN, HIGH);
			digitalWrite(DIR_PIN2, HIGH);
			while (busq < 400)
			{
				move(1, 1, 5000);
				move(1, 2, 5000);
				busq++;
				for (int i = 0; i < 40; i++)
				{
					value1_f = meanFilter2.AddValue(analogRead(hall1));
				}
				sensorRead1 = value1_f / 4;
				if (sensorRead1 > sensorMax1)
				{
					busq = 400;
					Flag_der = 1;
				}
			}
		}
		if (Flag_izq == 1)
		{
			move(100, 1, 5000);
			digitalWrite(DIR_PIN, HIGH);
			for (int i = 0; i < 40; i++)
			{
				value1_f = meanFilter2.AddValue(analogRead(hall1));
			}
			sensorRead1 = value1_f / 4;
			while (sensorRead1 < sensorMax1)
			{
				move(1, 1, 5000);
				for (int i = 0; i < 40; i++)
				{
					value1_f = meanFilter2.AddValue(analogRead(hall1));
				}
				sensorRead1 = value1_f / 4;
			}
		}
	}
	if (sensorRead1 > sensorMax1)
	{
		if (Flag_der == 0 && Flag_izq == 0)
		{
			digitalWrite(DIR_PIN, LOW);
			move(100, 1, 5000);
			digitalWrite(DIR_PIN, HIGH);
			for (int i = 0; i < 40; i++)
			{
				value1_f = meanFilter2.AddValue(analogRead(hall1));
			}
			sensorRead1 = value1_f / 4;
			while (sensorRead1 < sensorMax1)
			{
				move(1, 1, 5000);
				for (int i = 0; i < 40; i++)
				{
					value1_f = meanFilter2.AddValue(analogRead(hall1));
				}
				sensorRead1 = value1_f / 4;
			}
		}
	}

	delay(1000);
	findingSlowSensor1();
	delay(1000);
}

/**
 * @brief Esta funcion se utiliza entre programas para verificar que la siguiente secuencia iniciara en el punto cero.
 * en este caso la funcion contempla solo al brazo 2 con la polaridad del iman positiva.
 * DESCRIPCION GENERAL
 * En la primera condicion de esta funcion se determina si el brazo se encuentra fuera del rango del sensor
 * En caso de entrar en esta condicion inicia una busqueda con rango maximo de 600 pasos en sentido antihorario
 * Si no se encuentra nada en ese rango inicia una busqueda en sentido horario con un rango maximo de 1200 pasos
 * Si encuentra el sensor en la primera busqueda se activa una bandera para determinar que esta posicionado del lado derecho del sensor 
 * posteriormente se mueve 600 pasos en sentido antihorario y regresa en sentido horario para posicionarse al inicio del sensor pero del lado izquierdo.
 * Existe otro caso en el cual al iniciar la funcion el brazo ya se encuentra dentro del rango del sensor, en ese caso se mueve 600 pasos en sentido 
 * antihorario para garantizar que sale del sensor y posteriormente regresa en sentido horario hasta que es detectado por el sensor posicionandose asi
 * del lado izquierdo del sensor.
 * Finalmente cuando ya esta posicionado al inicio del lado izquierdo del sensor realiza el proceso de slow calibration para centrarse correctamente.
 */
void verif_cal_positiveb2(void)
{
	digitalWrite(EN_PIN2, LOW);
	digitalWrite(EN_PIN, LOW);
	int value2_f;
	flag = 1;
	int busq2 = 0;
	int Flag2_b = 0;
	int Flag2_der = 0;
	int Flag2_izq = 0;
	int value1_f;
	digitalWrite(DIR_PIN, HIGH);
	digitalWrite(DIR_PIN2, HIGH);

	for (int i = 0; i < 40; i++)
	{
		value2_f = meanFilter2.AddValue(analogRead(hall2));
	}
	sensorRead2 = value2_f / 4;
	if (sensorRead2 < sensorMax2)
	{
		while (busq2 < 600)
		{
			move(1, 2, 5000);
			busq2++;

			for (int i = 0; i < 40; i++)
			{
				value2_f = meanFilter2.AddValue(analogRead(hall2));
			}
			sensorRead2 = value2_f / 4;

			if (sensorRead2 > sensorMax2)
			{
				busq2 = 600;
				Flag2_b = 1;
				Flag2_izq = 1;
			}
		}
		if (Flag2_b == 0)
		{
			busq2 = 0;
			digitalWrite(DIR_PIN2, LOW);
			while (busq2 < 1200)
			{
				move(1, 2, 5000);
				busq2++;
				for (int i = 0; i < 40; i++)
				{
					value2_f = meanFilter2.AddValue(analogRead(hall2));
				}
				sensorRead2 = value2_f / 4;
				if (sensorRead2 > sensorMax2)
				{
					busq2 = 1200;
					Flag2_der = 1;
				}
			}
		}
		if (Flag2_izq == 1)
		{
			move(600, 2, 5000);
			digitalWrite(DIR_PIN2, LOW);
			for (int i = 0; i < 40; i++)
			{
				value2_f = meanFilter2.AddValue(analogRead(hall2));
			}
			sensorRead2 = value2_f / 4;
			while (sensorRead2 < sensorMax2)
			{
				move(1, 2, 5000);
				for (int i = 0; i < 40; i++)
				{
					value2_f = meanFilter2.AddValue(analogRead(hall2));
				}
				sensorRead2 = value2_f / 4;
			}
		}
	}
	if (sensorRead2 > sensorMax2)
	{
		if (Flag2_der == 0 && Flag2_izq == 0)
		{
			digitalWrite(DIR_PIN2, HIGH);
			move(250, 2, 5000);
			digitalWrite(DIR_PIN2, LOW);
			for (int i = 0; i < 40; i++)
			{
				value2_f = meanFilter2.AddValue(analogRead(hall2));
			}
			sensorRead2 = value2_f / 4;
			while (sensorRead2 < sensorMax2)
			{
				move(1, 2, 5000);
				for (int i = 0; i < 40; i++)
				{
					value2_f = meanFilter2.AddValue(analogRead(hall2));
				}
				sensorRead2 = value2_f / 4;
			}
		}
	}

	delay(1000);
	findingSlowSensor2();
	delay(1000);
}

/**
 * @brief Esta funcion se utiliza entre programas para verificar que la siguiente secuencia iniciara en el punto cero.
 * en este caso la funcion contempla solo al brazo 1 con la polaridad del iman negativa. 
 * DESCRIPCION GENERAL
 * En la primera condicion de esta funcion se determina si el brazo se encuentra fuera del rango del sensor
 * En caso de entrar en esta condicion inicia una busqueda con rango maximo de 200 pasos en sentido horario
 * Si no se encuentra nada en ese rango inicia una busqueda en sentido antihorario con un rango maximo de 400 pasos
 * Si encuentra el sensor en la primera busqueda se activa una bandera para determinar que esta posicionado del lado izquierdo del sensor 
 * posteriormente se mueve 100 pasos en sentido horario y regresa en sentido antihorario para posicionarse al inicio del sensor pero del lado derecho.
 * Existe otro caso en el cual al iniciar la funcion el brazo ya se encuentra dentro del rango del sensor, en ese caso se mueve 100 pasos en sentido 
 * horario para garantizar que sale del sensor y posteriormente regresa en sentido antihorario hasta que es detectado por el sensor posicionandose asi
 * del lado derecho del sensor.
 * Finalmente cuando ya esta posicionado al inicio del lado izquierdo del sensor realiza el proceso de slow calibration para centrarse correctamente.
 */
void verif_cal_negativeb1(void)
{
	digitalWrite(EN_PIN2, LOW);
	digitalWrite(EN_PIN, LOW);
	int value1_f;
	flag = 1;
	int busq = 0;
	int Flag_b = 0;
	int Flag_der = 0;
	int Flag_izq = 0;
	digitalWrite(DIR_PIN, LOW);
	digitalWrite(DIR_PIN2, LOW);

	for (int i = 0; i < 40; i++)
	{
		value1_f = meanFilter2.AddValue(analogRead(hall1));
	}
	sensorRead1 = value1_f / 4;
	if (sensorRead1 > sensorMin1)
	{
		while (busq < 200)
		{
			move(1, 1, 5000);
			move(1, 2, 5000);
			busq++;

			for (int i = 0; i < 40; i++)
			{
				value1_f = meanFilter2.AddValue(analogRead(hall1));
			}
			sensorRead1 = value1_f / 4;

			if (sensorRead1 < sensorMin1)
			{
				busq = 200;
				Flag_b = 1;
				Flag_izq = 1;
			}
		}
		if (Flag_b == 0)
		{
			busq = 0;
			digitalWrite(DIR_PIN, HIGH);
			digitalWrite(DIR_PIN2, HIGH);
			while (busq < 400)
			{
				move(1, 1, 5000);
				move(1, 2, 5000);
				busq++;
				for (int i = 0; i < 40; i++)
				{
					value1_f = meanFilter2.AddValue(analogRead(hall1));
				}
				sensorRead1 = value1_f / 4;
				if (sensorRead1 < sensorMin1)
				{
					busq = 400;
					Flag_der = 1;
				}
			}
		}
		if (Flag_izq == 1)
		{
			move(100, 1, 5000);
			digitalWrite(DIR_PIN, HIGH);
			for (int i = 0; i < 40; i++)
			{
				value1_f = meanFilter2.AddValue(analogRead(hall1));
			}
			sensorRead1 = value1_f / 4;
			while (sensorRead1 > sensorMin1)
			{
				move(1, 1, 5000);
				for (int i = 0; i < 40; i++)
				{
					value1_f = meanFilter2.AddValue(analogRead(hall1));
				}
				sensorRead1 = value1_f / 4;
			}
		}
	}
	if (sensorRead1 < sensorMin1)
	{
		if (Flag_der == 0 && Flag_izq == 0)
		{
			digitalWrite(DIR_PIN, LOW);
			move(100, 1, 5000);
			digitalWrite(DIR_PIN, HIGH);
			for (int i = 0; i < 40; i++)
			{
				value1_f = meanFilter2.AddValue(analogRead(hall1));
			}
			sensorRead1 = value1_f / 4;
			while (sensorRead1 > sensorMin1)
			{
				move(1, 1, 5000);
				for (int i = 0; i < 40; i++)
				{
					value1_f = meanFilter2.AddValue(analogRead(hall1));
				}
				sensorRead1 = value1_f / 4;
			}
		}
	}

	delay(1000);
	findingSlowSensor1Negative();
	delay(1000);
}

/**
 * @brief Esta funcion se utiliza entre programas para verificar que la siguiente secuencia iniciara en el punto cero.
 * en este caso la funcion contempla solo al brazo 2 con la polaridad del iman negativa.
 * DESCRIPCION GENERAL
 * En la primera condicion de esta funcion se determina si el brazo se encuentra fuera del rango del sensor
 * En caso de entrar en esta condicion inicia una busqueda con rango maximo de 600 pasos en sentido antihorario
 * Si no se encuentra nada en ese rango inicia una busqueda en sentido horario con un rango maximo de 1200 pasos
 * Si encuentra el sensor en la primera busqueda se activa una bandera para determinar que esta posicionado del lado derecho del sensor 
 * posteriormente se mueve 600 pasos en sentido antihorario y regresa en sentido horario para posicionarse al inicio del sensor pero del lado izquierdo.
 * Existe otro caso en el cual al iniciar la funcion el brazo ya se encuentra dentro del rango del sensor, en ese caso se mueve 600 pasos en sentido 
 * antihorario para garantizar que sale del sensor y posteriormente regresa en sentido horario hasta que es detectado por el sensor posicionandose asi
 * del lado izquierdo del sensor.
 * Finalmente cuando ya esta posicionado al inicio del lado izquierdo del sensor realiza el proceso de slow calibration para centrarse correctamente. 
 */
void verif_cal_negativeb2(void)
{
	digitalWrite(EN_PIN2, LOW);
	digitalWrite(EN_PIN, LOW);
	int value2_f;
	flag = 1;
	int busq2 = 0;
	int Flag2_b = 0;
	int Flag2_der = 0;
	int Flag2_izq = 0;
	int value1_f;
	digitalWrite(DIR_PIN, HIGH);
	digitalWrite(DIR_PIN2, HIGH);

	for (int i = 0; i < 40; i++)
	{
		value2_f = meanFilter2.AddValue(analogRead(hall2));
	}
	sensorRead2 = value2_f / 4;
	if (sensorRead2 > sensorMin2)
	{
		while (busq2 < 600)
		{
			move(1, 2, 5000);
			busq2++;

			for (int i = 0; i < 40; i++)
			{
				value2_f = meanFilter2.AddValue(analogRead(hall2));
			}
			sensorRead2 = value2_f / 4;

			if (sensorRead2 < sensorMin2)
			{
				busq2 = 600;
				Flag2_b = 1;
				Flag2_izq = 1;
			}
		}
		if (Flag2_b == 0)
		{
			busq2 = 0;
			digitalWrite(DIR_PIN2, LOW);
			while (busq2 < 1200)
			{
				move(1, 2, 5000);
				busq2++;
				for (int i = 0; i < 40; i++)
				{
					value2_f = meanFilter2.AddValue(analogRead(hall2));
				}
				sensorRead2 = value2_f / 4;
				if (sensorRead2 < sensorMin2)
				{
					busq2 = 1200;
					Flag2_der = 1;
				}
			}
		}
		if (Flag2_izq == 1)
		{
			move(600, 2, 5000);
			digitalWrite(DIR_PIN2, LOW);
			for (int i = 0; i < 40; i++)
			{
				value2_f = meanFilter2.AddValue(analogRead(hall2));
			}
			sensorRead2 = value2_f / 4;
			while (sensorRead2 > sensorMin2)
			{
				move(1, 2, 5000);
				for (int i = 0; i < 40; i++)
				{
					value2_f = meanFilter2.AddValue(analogRead(hall2));
				}
				sensorRead2 = value2_f / 4;
			}
		}
	}
	if (sensorRead2 < sensorMin2)
	{
		if (Flag2_der == 0 && Flag2_izq == 0)
		{
			digitalWrite(DIR_PIN2, HIGH);
			move(250, 2, 5000);
			digitalWrite(DIR_PIN2, LOW);
			for (int i = 0; i < 40; i++)
			{
				value2_f = meanFilter2.AddValue(analogRead(hall2));
			}
			sensorRead2 = value2_f / 4;
			while (sensorRead2 > sensorMin2)
			{
				move(1, 2, 5000);
				for (int i = 0; i < 40; i++)
				{
					value2_f = meanFilter2.AddValue(analogRead(hall2));
				}
				sensorRead2 = value2_f / 4;
			}
		}
	}

	delay(1000);
	findingSlowSensor2Negative();
	delay(1000);
}

/**
 * @brief Esta funcion se seleccionan las funciones necesarias para la verificacion de los brazos segun la polaridad correspondiente a cada iman.
 */
void WorkingArea::verificacion_cal()
{
	if (sensorPole1 == 1)
	{
		verif_cal_positiveb1();
	}
	if (sensorPole1 == 0)
	{
		verif_cal_negativeb1();
	}
	if (sensorPole2 == 1)
	{
		verif_cal_positiveb2();
	}
	if (sensorPole2 == 0)
	{
		verif_cal_negativeb2();
	}
}

///////////////////////

/**
 * @brief Esta funcion determina el nivel de referencia a partir del cual se considera la deteccion de campo magnetico.
 * DESCRIPCION GENERAL
 * Esta funcion inicia moviendo el brazo en sentido antihorario usando la funcion configTimerForMovement y sensando el DIAG_PIN en caso de que el brazo colisione,
 * si colisiona el brazo 1 regresa 300 pasos y el brazo 2 baja a 90 grados para evitar la colision.
 * Si el brazo logra finalizar el primer movimiento sin colisionar regresa 800 pasos e inicia el algoritmo para determinar el nivel cero del sensor.
 * Se toman 5 muestras en angulos distintos, la primera muestra se toma desde donde quedo posicionado el brazo, posteriormente se mueve 10 grados en sentido horario para tomar
 * la segunda muestra, se mueve 10 grados en sentido horario para la tercera medicion, regresa 30 grados en sentido antihorario para la cuarta medicion y avanza 10 grados mas
 * para tomar la quita medicion.
 * En cada una de las 5 muestras se toman 5 mediciones filtradas de las cuales se obtienen el valor maximo, el valor minimo, el valor promedio y un valor al que se le determino 
 * como valor similitud ya que es comparado con cada promedio de las 5 muestras tomadas a diferentes algulos y se almacena el contador de cuantos valores son similares al de la 
 * muestra analizada.
 * Finalmente se determina que si 3 muestras o mas son similares se ha hallado el valor cero del sensor el cual se almacena en una variable global y se retorna un 1 para 
 * indicar que no es necesario repetir el procedimiento.
 * @param level_zero1 esta variable global almacena el nivel de referencia para el sensor Hall 1.
 * @return Retorna un 1 si el calculo del nivel de referencia se obtuvo correctamente, en caso contrario retorna un 0 para repetir de proceso de busqueda del nivel de referencia.
 */
int zero_Hall1(void)
{
	flag = 1;
	digitalWrite(EN_PIN, LOW);
	digitalWrite(EN_PIN2, LOW);
	int value_hall1;
	int value_filt;
	int min = 5000;
	int max = 0;
	int add = 0;
	int average;
	int add_averages = 0;
	int values_similares = 0;
	int p = 0;
	flag = 2;
	int flag_c = 0;
	digitalWrite(DIR_PIN, HIGH);
	digitalWrite(DIR_PIN2, HIGH);
	if (analogRead(PIN_ProducType) > 3500)
	{
		configTimerForMovement();
		delay(500);
		while (p < 300)
		{
			if (digitalRead(DIAG_PIN) == 1)
			{
				flag = 1;
				flag_c = 1;
				//Back a little  first arm
				digitalWrite(DIR_PIN, LOW);
				move(300, 1, 3000);

				//Move second arm 90 degrees.
				digitalWrite(EN_PIN, LOW);
				digitalWrite(EN_PIN2, LOW);
				digitalWrite(DIR_PIN2, HIGH);
				move(1600, 2, 2000);
				p = 300;
			}
			p++;
			delay(10);
		}
		if (flag_c == 0)
		{
			flag = 1;
			digitalWrite(EN_PIN, LOW);
			digitalWrite(EN_PIN2, LOW);
			digitalWrite(DIR_PIN, LOW);
			move(800, 1, 2000);
		}
	}
	flag = 1;
	for (int x = 0; x < 5; x++)
	{
		for (int i = 0; i < 40; i++)
		{
			value_filt = meanFilter2.AddValue(analogRead(hall1));
		}
		value_hall1 = value_filt / 4;

		if (value_hall1 > max)
		{
			max = value_hall1;
		}
		if (value_hall1 < min)
		{
			min = value_hall1;
		}
		add = add + value_hall1;
		maximumVector[0] = max;
		minimumVector[0] = min;
		delay(100);
	}
	average = add / 5;
	meanVector[0] = average;
	add = 0;

	digitalWrite(DIR_PIN, LOW);
	move(178, 1, 3000);

	min = 5000;
	max = 0;
	for (int x = 0; x < 5; x++)
	{
		for (int i = 0; i < 40; i++)
		{
			value_filt = meanFilter2.AddValue(analogRead(hall1));
		}
		value_hall1 = value_filt / 4;

		if (value_hall1 > max)
		{
			max = value_hall1;
		}
		if (value_hall1 < min)
		{
			min = value_hall1;
		}
		add = add + value_hall1;
		maximumVector[1] = max;
		minimumVector[1] = min;
		delay(100);
	}
	average = add / 5;
	meanVector[1] = average;
	add = 0;

	digitalWrite(DIR_PIN, LOW);
	move(178, 1, 3000);

	min = 5000;
	max = 0;
	for (int x = 0; x < 5; x++)
	{
		for (int i = 0; i < 40; i++)
		{
			value_filt = meanFilter2.AddValue(analogRead(hall1));
		}
		value_hall1 = value_filt / 4;

		if (value_hall1 > max)
		{
			max = value_hall1;
		}
		if (value_hall1 < min)
		{
			min = value_hall1;
		}
		add = add + value_hall1;
		maximumVector[2] = max;
		minimumVector[2] = min;
		delay(100);
	}
	average = add / 5;
	meanVector[2] = average;
	add = 0;

	digitalWrite(DIR_PIN, HIGH);
	move(533, 1, 3000);

	min = 5000;
	max = 0;
	for (int x = 0; x < 5; x++)
	{
		for (int i = 0; i < 40; i++)
		{
			value_filt = meanFilter2.AddValue(analogRead(hall1));
		}
		value_hall1 = value_filt / 4;

		if (value_hall1 > max)
		{
			max = value_hall1;
		}
		if (value_hall1 < min)
		{
			min = value_hall1;
		}
		add = add + value_hall1;
		maximumVector[3] = max;
		minimumVector[3] = min;
		delay(100);
	}
	average = add / 5;
	meanVector[3] = average;
	add = 0;

	digitalWrite(DIR_PIN, HIGH);
	move(178, 1, 3000);

	min = 5000;
	max = 0;
	for (int x = 0; x < 5; x++)
	{
		for (int i = 0; i < 40; i++)
		{
			value_filt = meanFilter2.AddValue(analogRead(hall1));
		}
		value_hall1 = value_filt / 4;

		if (value_hall1 > max)
		{
			max = value_hall1;
		}
		if (value_hall1 < min)
		{
			min = value_hall1;
		}
		add = add + value_hall1;
		maximumVector[4] = max;
		minimumVector[4] = min;
		delay(100);
	}
	average = add / 5;
	meanVector[4] = average;
	add = 0;

	int similarity;
	int cont_simi = 0;
	for (int i = 0; i < 5; i++)
	{
		for (int j = 0; j < 5; j++)
		{
			similarity = meanVector[i] - meanVector[j];
			if (similarity < 0)
			{
				similarity = similarity * (-1);
			}
			if (similarity <= 10)
			{
				cont_simi++;
			}
		}
		simiVector[i] = cont_simi;
		cont_simi = 0;
	}
	int accum5 = 0;
	int accum4 = 0;
	int accum3 = 0;
	for (int i = 0; i < 5; i++)
	{
		if (simiVector[i] == 5)
		{
			accum5++;
		}
		if (simiVector[i] == 4)
		{
			accum4++;
		}
		if (simiVector[i] == 3)
		{
			accum3++;
		}
	}

	min = 5000;
	max = 0;
	for (int i = 0; i < 5; i++)
	{
		if (simiVector[i] >= 3)
		{
			if (maximumVector[i] > max)
			{
				max = maximumVector[i];
			}
			if (minimumVector[i] < min)
			{
				min = minimumVector[i];
			}
			add_averages = add_averages + meanVector[i];
			values_similares++;
		}
	}
	level_zero1 = add_averages / values_similares;
	A = level_zero1;
	H = highByte(A);
	L = lowByte(A);
	EEPROM.write(403, H);
	EEPROM.commit();
	EEPROM.write(404, L);
	EEPROM.commit();
	sensorMax1 = max;
	sensorMin1 = min;

	if (accum5 != 0)
	{
		return 1;
	}
	if (accum4 != 0)
	{
		return 1;
	}
	if (accum3 != 0)
	{
		return 1;
	}
}

/**
 * @brief Esta funcion determina el nivel de referencia a partir del cual se considera la deteccion de campo magnetico.
 * DESCRIPCION GENERAL
 * Esta funcion inicia moviendo el brazo en sentido antihorario usando la funcion configTimerForMovement y sensando el DIAG_PIN en caso de que el brazo colisione,
 * si colisiona el brazo 1 regresa 300 pasos y el brazo 2 baja a 90 grados para evitar la colision.
 * Si el brazo logra finalizar el primer movimiento sin colisionar regresa 800 pasos e inicia el algoritmo para determinar el nivel cero del sensor.
 * Se toman 5 muestras en angulos distintos, la primera muestra se toma desde donde quedo posicionado el brazo, posteriormente se mueve 10 grados en sentido horario para tomar
 * la segunda muestra, se mueve 10 grados en sentido horario para la tercera medicion, regresa 30 grados en sentido antihorario para la cuarta medicion y avanza 10 grados mas
 * para tomar la quita medicion.
 * En cada una de las 5 muestras se toman 5 mediciones filtradas de las cuales se obtienen el valor maximo, el valor minimo, el valor promedio y un valor al que se le determino 
 * como valor similitud ya que es comparado con cada promedio de las 5 muestras tomadas a diferentes algulos y se almacena el contador de cuantos valores son similares al de la 
 * muestra analizada.
 * Finalmente se determina que si 3 muestras o mas son similares se ha hallado el valor cero del sensor el cual se almacena en una variable global y se retorna un 1 para 
 * indicar que no es necesario repetir el procedimiento.
 * @param level_zero2 esta variable global almacena el nivel de referencia para el sensor Hall 2.
 * @return Retorna un 1 si el calculo del nivel de referencia se obtuvo correctamente, en caso contrario retorna un 0 para repetir de proceso de busqueda del nivel de referencia.
 */
int zero_Hall2(void)
{
	flag = 1;
	digitalWrite(EN_PIN, LOW);
	digitalWrite(EN_PIN2, LOW);
	int value_hall2;
	int value_filt;
	int min = 5000;
	int max = 0;
	int add = 0;
	int average;
	int add_averages2 = 0;
	int values_similares2 = 0;
	int p = 0;
	int flag_c = 0;
	

	if (analogRead(PIN_ProducType) > 3500)
	{
		digitalWrite(EN_PIN, HIGH);
		flag = 3;
		configTimerForMovement();
		delay(500);
		while (p < 300)
		{
			if (digitalRead(DIAG_PIN2) == 1)
			{
				flag = 1;
				flag_c = 1;
				//Move second arm 90 degrees.
				digitalWrite(EN_PIN, LOW);
				digitalWrite(EN_PIN2, LOW);
				digitalWrite(DIR_PIN2, LOW);
				move(1600, 2, 2000);
				p = 300;
			}
			p++;
			delay(10);
		}
		if (flag_c == 0)
		{
			flag = 1;
			//Move second arm 90 degrees.
			digitalWrite(EN_PIN, LOW);
			digitalWrite(EN_PIN2, LOW);
			digitalWrite(DIR_PIN2, LOW);
			move(800, 2, 2000);
		}
	}
	flag = 1;
	for (int x = 0; x < 5; x++)
	{
		for (int i = 0; i < 40; i++)
		{
			value_filt = meanFilter2.AddValue(analogRead(hall2));
		}
		value_hall2 = value_filt / 4;

		if (value_hall2 > max)
		{
			max = value_hall2;
		}
		if (value_hall2 < min)
		{
			min = value_hall2;
		}
		add = add + value_hall2;
		maximumVector2[0] = max;
		minimumVector2[0] = min;
		delay(100);
	}
	average = add / 5;
	meanVector2[0] = average;
	add = 0;

	digitalWrite(DIR_PIN2, LOW);
	move(178, 2, 3000);

	min = 5000;
	max = 0;
	for (int x = 0; x < 5; x++)
	{
		for (int i = 0; i < 40; i++)
		{
			value_filt = meanFilter2.AddValue(analogRead(hall2));
		}
		value_hall2 = value_filt / 4;
		if (value_hall2 > max)
		{
			max = value_hall2;
		}
		if (value_hall2 < min)
		{
			min = value_hall2;
		}
		add = add + value_hall2;
		maximumVector2[1] = max;
		minimumVector2[1] = min;
		delay(100);
	}
	average = add / 5;
	meanVector2[1] = average;
	add = 0;

	digitalWrite(DIR_PIN2, LOW);
	move(178, 2, 3000);

	min = 5000;
	max = 0;
	for (int x = 0; x < 5; x++)
	{
		for (int i = 0; i < 40; i++)
		{
			value_filt = meanFilter2.AddValue(analogRead(hall2));
		}
		value_hall2 = value_filt / 4;

		if (value_hall2 > max)
		{
			max = value_hall2;
		}
		if (value_hall2 < min)
		{
			min = value_hall2;
		}
		add = add + value_hall2;
		maximumVector2[2] = max;
		minimumVector2[2] = min;
		delay(100);
	}
	average = add / 5;
	meanVector2[2] = average;
	add = 0;

	digitalWrite(DIR_PIN2, HIGH);
	move(533, 2, 3000);

	min = 5000;
	max = 0;
	for (int x = 0; x < 5; x++)
	{
		for (int i = 0; i < 40; i++)
		{
			value_filt = meanFilter2.AddValue(analogRead(hall2));
		}
		value_hall2 = value_filt / 4;

		if (value_hall2 > max)
		{
			max = value_hall2;
		}
		if (value_hall2 < min)
		{
			min = value_hall2;
		}
		add = add + value_hall2;
		maximumVector2[3] = max;
		minimumVector2[3] = min;
		delay(100);
	}
	average = add / 5;
	meanVector2[3] = average;
	add = 0;

	digitalWrite(DIR_PIN2, HIGH);
	move(178, 2, 3000);

	min = 5000;
	max = 0;
	for (int x = 0; x < 5; x++)
	{
		for (int i = 0; i < 40; i++)
		{
			value_filt = meanFilter2.AddValue(analogRead(hall2));
		}
		value_hall2 = value_filt / 4;

		if (value_hall2 > max)
		{
			max = value_hall2;
		}
		if (value_hall2 < min)
		{
			min = value_hall2;
		}
		add = add + value_hall2;
		maximumVector2[4] = max;
		minimumVector2[4] = min;
		delay(100);
	}
	average = add / 5;
	meanVector2[4] = average;
	add = 0;

	int similarity;
	int cont_simi = 0;
	for (int i = 0; i < 5; i++)
	{
		for (int j = 0; j < 5; j++)
		{
			similarity = meanVector2[i] - meanVector2[j];
			if (similarity < 0)
			{
				similarity = similarity * (-1);
			}
			if (similarity <= 10)
			{
				cont_simi++;
			}
		}
		simiVector2[i] = cont_simi;
		cont_simi = 0;
	}
	int accum5 = 0;
	int accum4 = 0;
	int accum3 = 0;
	for (int i = 0; i < 5; i++)
	{
		if (simiVector2[i] == 5)
		{
			accum5++;
		}
		if (simiVector2[i] == 4)
		{
			accum4++;
		}
		if (simiVector2[i] == 3)
		{
			accum3++;
		}
	}

	min = 5000;
	max = 0;
	for (int i = 0; i < 5; i++)
	{
		if (simiVector2[i] >= 3)
		{
			if (maximumVector2[i] > max)
			{
				max = maximumVector2[i];
			}
			if (minimumVector2[i] < min)
			{
				min = minimumVector2[i];
			}
			add_averages2 = add_averages2 + meanVector2[i];
			values_similares2++;
		}
	}
	level_zero2 = add_averages2 / values_similares2;
	A = level_zero2;
	H = highByte(A);
	L = lowByte(A);
	EEPROM.write(405, H);
	EEPROM.commit();
	EEPROM.write(406, L);
	EEPROM.commit();
	sensorMax2 = max;
	sensorMin2 = min;

	if (accum5 != 0)
	{
		return 1;
	}
	if (accum4 != 0)
	{
		return 1;
	}
	if (accum3 != 0)
	{
		return 1;
	}
}

/**
 * @brief Esta funcion determina el polo del campo magnetico correspondiente al iman que interacciona con el brazo 1.
 * DESCRIPCION GFENERAL
 * Antes de entrar a esta funcion el brazo ya se encuentra posicionado correctamente al inicio del sensor
 * Para determinar el polo se avanzan 100 pasos para que al retornar el movimiento el brazo termine en la mis ma posicion en la que inicio.
 * Al retornar el sentido horario almacena el valor maximo y el valor minimo que haya obtenidode las mediciones tomadas en cada paso, este valor 
 * se obtiene como un valor absoluto respecto del nivel cero, el cual para este punto del programa ya se conoce.
 * finalmente se compara el valor maximo absoluto obtenido y el valor minimo absoluto obtenido, si el maximo absoluto es mayor que el minimo
 * absoluto se determina que le polo es positivo, en cambio si sucede de forma contraria se determina que el polo es negativo.
 * @param vect_Pole1 Se almacenan un conjunto de datos correspondientes a un barrido del brazo1 por debajo del sensor de efecto Hall.
 * @return Retorna un 1 si se determina que el polo positivo y un 0 si el polo es negativo.
 */
int Pole1(void)
{
	int vect_Pole1[100];
	int maximum_Pole1 = 0;
	int minimum_Pole1 = 5000;
	int dif_max_abs1;
	int dif_min_abs1;
	digitalWrite(EN_PIN, LOW);
	digitalWrite(EN_PIN2, LOW);

	for (int i = 0; i < 100; i++)
	{
		vect_Pole1[i] = -1;
	}
	int k = 0;
	int val_sens;
	digitalWrite(DIR_PIN, HIGH);

	for (int t = 0; t < 100; t++)
	{
		move(1, 1, 5000);
	}

	digitalWrite(DIR_PIN, LOW);
	while (k < 100)
	{
		move(1, 1, 5000);
		for (int y = 0; y < 40; y++)
		{
			sensorRead1 = (analogRead(hall1)) / 4;
			val_sens = meanFilter2.AddValue(sensorRead1);
		}
		delay(1);
		if (val_sens > maximum_Pole1) 
		{
			maximum_Pole1 = val_sens; 
		}
		if (val_sens < minimum_Pole1) 
		{
			minimum_Pole1 = val_sens; 
		}
		vect_Pole1[k] = val_sens; 

		delay(1);
		k++;
	}
	dif_max_abs1 = maximum_Pole1 - level_zero1;
	dif_min_abs1 = level_zero1 - minimum_Pole1;
	if (dif_max_abs1 > dif_min_abs1)
	{
		return 1;
	}
	if (dif_max_abs1 < dif_min_abs1)
	{
		return 0;
	}
}

/**
 * @brief Esta funcion determina el polo del campo magnetico correspondiente al iman que interacciona con el brazo 2.
 * DESCRIPCION GFENERAL
 * Antes de entrar a esta funcion el brazo ya se encuentra posicionado correctamente al inicio del sensor
 * Para determinar el polo se avanzan 800 pasos para que al retornar el movimiento el brazo termine en la mis ma posicion en la que inicio.
 * Al retornar el sentido horario almacena el valor maximo y el valor minimo que haya obtenidode las mediciones tomadas en cada paso, este valor 
 * se obtiene como un valor absoluto respecto del nivel cero, el cual para este punto del programa ya se conoce.
 * finalmente se compara el valor maximo absoluto obtenido y el valor minimo absoluto obtenido, si el maximo absoluto es mayor que el minimo
 * absoluto se determina que le polo es positivo, en cambio si sucede de forma contraria se determina que el polo es negativo.
 * @param vect_Pole2 Se almacenan un conjunto de datos correspondientes a un barrido del brazo2 por debajo del sensor de efecto Hall.
 * @return Retorna un 1 si se determina que el polo positivo y un 0 si el polo es negativo.
 */
int Pole2(void)
{
	int value2_f;
	int vect_Pole2[500];
	int maximum_Pole2 = 0;
	int minimum_Pole2 = 5000;
	int dif_max_abs2;
	int dif_min_abs2;
	digitalWrite(EN_PIN, LOW);
	digitalWrite(EN_PIN2, LOW);

	for (int i = 0; i < 500; i++)
	{
		vect_Pole2[i] = -1;
	}
	int k = 0;
	int val_sens;
	digitalWrite(DIR_PIN2, LOW);

	for (int t = 0; t < 500; t++)
	{
		move(1, 2, 5000);
	}
	digitalWrite(DIR_PIN2, HIGH);
	while (k < 500)
	{
		move(1, 2, 5000);
		for (int y = 0; y < 40; y++)
		{
			sensorRead2 = (analogRead(hall2)) / 4;
			val_sens = meanFilter2.AddValue(sensorRead2);
		}
		delay(1);
		if (val_sens > maximum_Pole2) 
		{
			maximum_Pole2 = val_sens; 
		}
		if (val_sens < minimum_Pole2) 
		{
			minimum_Pole2 = val_sens; 
		}
		vect_Pole2[k] = val_sens; 

		delay(1);
		k++;
	}
	dif_max_abs2 = maximum_Pole2 - level_zero2;
	dif_min_abs2 = level_zero2 - minimum_Pole2;
	if (dif_max_abs2 > dif_min_abs2)
	{
		return 1;
	}
	if (dif_max_abs2 < dif_min_abs2)
	{
		return 0;
	}
}

/**
 * @brief Esta funcion verifica si ya se han almacenado en la EEPROM las variables iniciales necesarias para en funcionamiento del sistema,
 * lo cual solo es posible cuando se programa por primera vez la ESP32 o despues de un reset completo del sistema.
 * @return Retorna un 1 si se determina que la EEPROM eseta vacia o un 0 en caso de que ya existan los valores almacenados. 
 */
int Check_ini(void)
{
	EEPROM.begin(EEPROM_SIZE);
	int value_eeprom;
	int cont_eeprom = 0;
	for (int i = ADDRESSPOLESENSE1; i < ADDRESSPOLESENSE1 + ADDRESSESTOVERIFY; i++)
	{
		value_eeprom = EEPROM.read(i);
		if (value_eeprom != 255)
		{
			cont_eeprom++;
		}
	}
	if (cont_eeprom >= ADDRESSESTOVERIFY)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}   