#include <SoftwareSerial.h>
#include "Simple_MPU6050.h"         // incluye libreria Simple_MPU6050

#include <heartRate.h>            // librerías para MAX30102 
#include <MAX30105.h>
#include <spo2_algorithm.h>
#include <Wire.h>


#define TEMP_CANT_MEDICIONES 32                                // cantidad de mediciones para tomar el promedio
#define INTERVAL_SAMPLE 500                                    // intervalo de Sampling en ms (milisegundos)
#define INTERVAL_SEND 2000                                     // intervalo de envío a teléfono celular en ms (milisegundos)
#define SERIAL_BPS 115200                                      // velocidad de conexión el puerto serie bps
#define SAMPLE_SEND_RATE  INTERVAL_SEND / INTERVAL_SAMPLE      // intervalo de envío a teléfono celular en ms (milisegundos)
#define TxD 11
#define RxD 10

#define MPU6050_ADDRESS_AD0_LOW     0x68                       // direccion I2C con AD0 en LOW o sin conexion
#define MPU6050_ADDRESS_AD0_HIGH    0x69                       // direccion I2C con AD0 en HIGH
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW    // por defecto AD0 en LOW
#define OFFSETS  -4530,    -226,    5670,      23,     433,       1

Simple_MPU6050 mpu;                     // crea objeto con nombre mpu de acelerómetro
//ENABLE_MPU_OVERFLOW_PROTECTION();     // activa proteccion

SoftwareSerial comBT(RxD, TxD);         //RX, TX Objeto de comunicacion por BlueTooth

MAX30105 particleSensor;                // crea objeto de MAX30105 - sensor de heartrate y oxímetro

const byte RATE_SIZE = 4;               //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];                  //Array of heart rates
byte rateSpot = 0;

float beatsPerMinute;
int beatAvg;
int beatAvgToSend = -1;
long lastBeat = 0;

/*
#define MAX_BRIGHTNESS 255
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif
int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read
*/

long lastTime = 0;  // última toma de variable 
long deltaTime = 0; // ultima vez que se actualizó el contador


//
// Parametros de la medición de temperatura
//

float R1 = 100000;//125000;//180000;//89000; // resistencia fija del divisor de tension 
float c1 = 2.114990448e-03, c2 = 0.3832381228e-04, c3 = 5.228061052e-07;
float aTemps[TEMP_CANT_MEDICIONES];
int iLoop = 0;

float yawAnt = 0.00;
float pitchAnt = 0.00;
float rollAnt = 0.00;

char movement = 'Y';
 
// coeficientes de S-H en pagina: 
// http://www.thinksrs.com/downloads/programs/Therm%20Calc/NTCCalibrator/NTCcalculator.htm

// ***************************************************************
// Definimos funciones para gestión de medición de temperatura
// ***************************************************************

// getTemperatura(): lee un valor de temperatura desde el puerto A3 y lo devuelve
float getTemperatura(){
  int Vo;  
  float logR2, R2, temperatura;
  
  Vo = analogRead(A0);   

  //Serial.println(Vo);
 
  R2 = R1 * (1023.0 / (float)Vo - 1.0); // conversion de tension a resistencia
  logR2 = log(R2);      // logaritmo de R2 necesario para ecuacion
  temperatura = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));   // ecuacion S-H
  temperatura = temperatura - 273.15;   // Kelvin a Centigrados (Celsius)

  temperatura = temperatura + 4.0;
  
  return temperatura;
}


// init_temp(): realiza las primeras TEMP_CANT_MEDICIONES para llenar el array 
void init_temp() {
  for(int i = 0; i < TEMP_CANT_MEDICIONES; i++){
    shiftArray();
    aTemps[0] = getTemperatura();       // tomamos el valor de temperatura
    delay(INTERVAL_SAMPLE);             // demora de medio segundo entre lecturas
  }
}

void init_temp2() {
  for(int i = 0; i < TEMP_CANT_MEDICIONES; i++){
    shiftArray();
    aTemps[0] = 25.0;       // inicializamos la temperatura en 25 grados
  }
}


// shiftArray(): mueve todos los elementos de un array para que se pueda insertar un elemento en la posición 0 
void shiftArray() {
  for(int j = (TEMP_CANT_MEDICIONES - 2); j >= 0; j--){
    aTemps[j+1] = aTemps[j];
  }
  aTemps[0] = 0;
}

// getAvg(): calcula los promedios 
float getAvg(){
  float sumTemp = 0.00;
  for(int i = 0; i < TEMP_CANT_MEDICIONES; i++){
    sumTemp = sumTemp + aTemps[i];
  }
  return sumTemp/TEMP_CANT_MEDICIONES;
}


void print_v(){
  Serial.print("array=[");
  for(int i = 0; i < TEMP_CANT_MEDICIONES; i++){
    Serial.print(aTemps[i]);
    Serial.print(",");    
  }  
}

float getTemperaturaPromedio(){
  float tempAvg;
   
  shiftArray();                 // corremos un lugar
  aTemps[0] = getTemperatura(); // tomamos una muestra
  tempAvg = getAvg();           // calculamos el promedio 
  return tempAvg;
}

// ***************************************************************
// Definimos funciones para gestión de medición de movimiento
// ***************************************************************

// Colocar valores personalizados

#define spamtimer(t) for (static uint32_t SpamTimer; (uint32_t)(millis() - SpamTimer) >= (t); SpamTimer = millis())
// spamtimer funcion para generar demora al escribir en monitor serie sin usar delay()

#define printfloatx(Name,Variable,Spaces,Precision,EndTxt) print(Name); {char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}Serial.print(EndTxt);
// printfloatx funcion para mostrar en monitor serie datos para evitar el uso se multiples print()

float redondear(float valor, float redondeo){
  float valor1;
  valor1 = valor / redondeo;
  valor1 = round(valor1) * redondeo;
  return valor1;
}

// mostrar_valores funcion que es llamada cada vez que hay datos disponibles desde el sensor
void mostrar_valores (int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp) {  
  uint8_t SpamDelay = 100;                                      // demora para escribir en monitor serie de 100 mseg
  Quaternion q;                                                 // variable necesaria para calculos posteriores
  VectorFloat gravity;                                          // variable necesaria para calculos posteriores
  float ypr[3] = { 0, 0, 0 };                                   // array para almacenar valores de yaw, pitch, roll
  float xyz[3] = { 0, 0, 0 };                                   // array para almacenar valores convertidos a grados de yaw, pitch, roll
  spamtimer(SpamDelay) {                                        // si han transcurrido al menos 100 mseg entonces proceder
    mpu.GetQuaternion(&q, quat);                                // funcion para obtener valor para calculo posterior
    mpu.GetGravity(&gravity, &q);                               // funcion para obtener valor para calculo posterior
    mpu.GetYawPitchRoll(ypr, &q, &gravity);                     // funcion obtiene valores de yaw, ptich, roll
    mpu.ConvertToDegrees(ypr, xyz);                             // funcion convierte a grados sexagesimales

    float yaw = redondear(xyz[0], 10);
    float pitch = redondear(xyz[1], 10);
    float roll = redondear(xyz[2], 10);

//    Serial.printfloatx(F("Yaw")  , xyz[0], 9, 4, F(",   "));  // muestra en monitor serie rotacion de eje Z, yaw
//    Serial.printfloatx(F("Pitch"), xyz[1], 9, 4, F(",   "));  // muestra en monitor serie rotacion de eje Y, pitch
//    Serial.printfloatx(F("Roll") , xyz[2], 9, 4, F(",   "));  // muestra en monitor serie rotacion de eje X, roll
//    Serial.print(movement);  // muestra en monitor serie movement
//    Serial.println();       // salto de linea    

    if ( yaw != yawAnt || pitch != pitchAnt || roll != rollAnt){
      movement = 'Y';        
    }else{
      movement = 'N';
    }

    yawAnt = yaw;
    pitchAnt = pitch;
    rollAnt = roll; 
    
  }
}

void Serial_Send(float tempAvg, char movement, int beatAvgToSend){

      Serial.printfloatx(F("Temp")  , tempAvg, 9, 1, F(",   "));  // muestra en monitor serie la temperatura
      Serial.print("Mov?  ");
      Serial.print(movement);
      Serial.print("CMB?  ");
      Serial.print(beatAvgToSend);
      Serial.println(" ");
}

void comBT_Send(float tempAvg, int decimalTempAvg, char movement, int beatAvgToSend){
  
      String tempMsg = String(tempAvg, 1);
      char cTempMsg[5];      
      tempMsg.toCharArray(cTempMsg, 5);
      
      for (int i = 0; i < 5; i++){
        comBT.write(cTempMsg[i]);  
      }
      comBT.write(',');
      comBT.write(movement);
      comBT.write(',');

      String beatMsg = String(beatAvgToSend);
      
      char cBeatMsg[3];      
      beatMsg.toCharArray(cBeatMsg, 3);
      
      for (int i = 0; i < 3; i++){
        comBT.write(cBeatMsg[i]);  
      }
            
      comBT.write('\n');
      
}

void comBT_SendText(char mensage1[]){
  int longitud = strlen(mensage1);
  for (int i = 0; i < longitud; i++){
    comBT.write(mensage1[i]);  
  }
  comBT.write('\n');
}

// ***************************************************************
// Programa Principal
// ***************************************************************


void Inicialice_Devices(){
  
   //Inicializización de Sensor de pulsaciones y oxímetro 
  particleSensor.begin(Wire, I2C_SPEED_FAST);
  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  
  //byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  //byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  //byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  //byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  //int pulseWidth = 411; //Options: 69, 118, 215, 411
  //int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
  //particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  //Inicializización de MPU
  uint8_t val;
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE  // activacion de bus I2C a 400 Khz
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif


  //Inicializización de Acelerómetro

#ifdef OFFSETS                // si existen OFFSETS
  Serial.println(F("Usando Offsets predefinidos"));     // texto estatico
  mpu.SetAddress(MPU6050_ADDRESS_AD0_LOW).load_DMP_Image(OFFSETS);  // inicializacion de sensor

#else                   // sin no existen OFFSETS
  Serial.println(F(" No se establecieron Offsets, haremos unos nuevos.\n" // muestra texto estatico
                   " Colocar el sensor en un superficie plana y esperar unos segundos\n"
                   " Colocar los nuevos Offsets en #define OFFSETS\n"
                   " para saltar la calibracion inicial \n"
                   " \t\tPresionar cualquier tecla y ENTER"));
  while (Serial.available() && Serial.read());    // lectura de monitor serie
  while (!Serial.available());        // si no hay espera              
  while (Serial.available() && Serial.read());    // lecyura de monitor serie
  mpu.SetAddress(MPU6050_ADDRESS_AD0_LOW).CalibrateMPU().load_DMP_Image();  // inicializacion de sensor
#endif  
  mpu.on_FIFO(mostrar_valores);   // llamado a funcion mostrar_valores si memoria FIFO tiene valores

  //Inicializización de Sensor de temperatura (termistor)
  init_temp2();
  //init_temp();                    // tomamos las primeras n muestras de temperatura 
}

// Inicializa el programa.
void setup() {

  //Inicializización de BlueTooth y puerto Serie

  Serial.begin(SERIAL_BPS);       // inicializa comunicacion serie a 11500 bps  
  comBT.begin(9600);              // inicializa puerto bluetooth de comunicación

  Serial.println("Aguardando conexión BlueTooth...");

  while(!comBT.available()){ 
    
  }

  Serial.println("Inicializando...");

  comBT_SendText("Inicializando...");

  Inicialice_Devices();  

}


void loop() {

  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true){ // chequeamos si sensamos una pulsación    
    long deltaBeat = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (deltaBeat / 1000.0);    

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }


  if (irValue < 50000){
    beatAvgToSend = -1;
  }else{
    beatAvgToSend = (int) beatAvg;
  }
  
  float tempAvg;

  deltaTime = millis() - lastTime;

  if ( deltaTime > INTERVAL_SAMPLE){

    lastTime = millis();

    tempAvg = getTemperaturaPromedio(); //tomo promedio de los últimos 5 mediciones

    mpu.dmp_read_fifo();                // funcion que evalua si existen datos nuevos en el sensor y llama a mostrar valor 
  
    iLoop++;
    if (iLoop == SAMPLE_SEND_RATE){
      iLoop = 0;
      if(comBT.available()){
        comBT_Send(tempAvg, 1, movement, beatAvgToSend); // envío datos vía bluetooth    
      }
      Serial_Send(tempAvg, movement, beatAvgToSend); // envío datos vía Serial
    }
    
  }  

}
