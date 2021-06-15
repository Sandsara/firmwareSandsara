/*Para poder usar el simulador se deben modificar las primeras lineas del archivo config.h como se muestra a continuacion.

#define PROCESSING_SIMULATOR
//#define DEBUGGING_DATA
//#define DEBUGGING_DETAIL
#define DISABLE_MOTORS

se prueba con un esp32 que manda los siguientes datos

  al inicio de cada archivo:
  Serial.print("fileName: ");
  Serial.println(name_file);
  
  para cada paso que deba dar:
  Serial.print(q1_steps);
  Serial.print(",");
  Serial.print(q2_steps + q1_steps);
  Serial.print(",");
  Serial.println(velocidad);
  
  con un mesaje de inicio en el arranque del esp32
  Serial.println("inicia");
  delay(1000);
  
  al final de cada archiv:
  Serial.println("finished");
  
  solo considerar eso y modificar el puerto COM o USB
*/
