
/*
  Capitulo 34 de Arduino desde cero en Español.
  Visualiza en monitor serie el valor de temperatura leido por un termistor
  tipo NTC en configuracion de divisor de tension con resistencia de 100 K.
  Se aplica la ecuacion de Steinhart-Hart y sus coeficientes mediante
  la pagina web indica en el comentario.

  Autor: bitwiseAr  

*/

#define CANT_MEDICIONES 32
#define INTERVAL_SAMPLE 500

float R1 = 100000;//89000;//49000;//125000;//180000;//89000;              // resistencia fija del divisor de tension 
float c1 = 2.114990448e-03, c2 = 0.3832381228e-04, c3 = 5.228061052e-07;
float aTemps[CANT_MEDICIONES];
float tempAvg; 

// coeficientes de S-H en pagina: 
// http://www.thinksrs.com/downloads/programs/Therm%20Calc/NTCCalibrator/NTCcalculator.htm

void setup() {
  Serial.begin(115200);   // inicializa comunicacion serie a 11500 bps
  init_temp();           // tomamos las primeras 5 muestras
}

float getTemperatura(){
  int Vo;  
  float logR2, R2, TEMPERATURA;
  
  Vo = analogRead(A0);      // lectura de A0

  //Serial.println(Vo);
 
  R2 = R1 * (1023.0 / (float)Vo - 1.0); // conversion de tension a resistencia
  logR2 = log(R2);      // logaritmo de R2 necesario para ecuacion
  TEMPERATURA = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));   // ecuacion S-H
  TEMPERATURA = TEMPERATURA - 273.15;   // Kelvin a Centigrados (Celsius)
  
  return TEMPERATURA;
}

void init_temp() {
  for(int i = 0; i < CANT_MEDICIONES; i++){
    shiftArray();
    aTemps[0] = getTemperatura();  // tomamos el valor de temperatura
    delay(INTERVAL_SAMPLE);             // demora de medio segundo entre lecturas
    print_v();
    Serial.println(" ] ");
  }
}

void shiftArray() {
  for(int j = (CANT_MEDICIONES - 2); j >= 0; j--){
    aTemps[j+1] = aTemps[j];
  }
  aTemps[0] = 0;
}

float getAvg(){
  float sumTemp = 0.00;
  for(int i = 0; i < CANT_MEDICIONES; i++){
    sumTemp = sumTemp + aTemps[i];
  }
  return sumTemp/CANT_MEDICIONES;
}



void print_v(){
  Serial.print("array=[");
  for(int i = 0; i < CANT_MEDICIONES; i++){
    Serial.print(aTemps[i]);
    Serial.print(",");    
  }  
}

void loop() {
  //tomo promedio de los últimos 5 mediciones

  shiftArray();
  
  aTemps[0] = getTemperatura();

  tempAvg = getAvg();


  Serial.print("Temp.Prom.: ");  // imprime valor en monitor serie
  Serial.print(tempAvg);
  //Serial.println(" C");

  Serial.print(",");

  Serial.print("Temp.punt.: ");  // imprime valor en monitor serie
  Serial.print(aTemps[0]);
  //Serial.println(" C"); 

  print_v();

  Serial.println(" ] ");

  delay(INTERVAL_SAMPLE);       // demora de medio segundo entre lecturas

  
  
}
