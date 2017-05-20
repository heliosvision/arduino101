#include <Arduino.h>
#include <SoftwareSerial.h>
#define LENG 31
unsigned char buf[LENG];

int PM01Value =	0;	// air quality PM1.0 value
int PM2_5Value = 0;	// air quality PM2.5 value
int PM10Value = 0;	// air quality PM10 value

// AIR QUALITY SENSOR 
#define	RxD 0	// SoftSerial Air Quality (BT_TX) --> Arduino (RxD) Pin
#define TxD 1	// SoftSerial Air Quality (BT_RX) <-- Arduino (TxD) Pin
SoftwareSerial softwareSerial(RxD,TxD);

// LIGHT SENSOR
#define LIGHT_PIN (2)	// Analog Light Pin

// TEMPERATURE SENSOR
#define TEMP_PIN (0)	// Analog Temperature Pin

// CO2 SENSOR
#define MG_PIN (1)	// analog input channel
#define BOOL_PIN (12)	// Arduino D2-CO2 sensor digital pinout, labled with "D" on PCB  
#define DC_GAIN (8.5)	// amplifier DC gain
#define READ_SAMPLE_TIMES (10)	// number of read samples for normal operation
#define READ_SAMPLE_INTERVAL (50)	// time interval between sample reads
#define ZERO_POINT_X (2.602)	// (user defined) the start point_on X_axis of the curve
#define ZERO_POINT_VOLTAGE (0.324)	// (user defined) output of the sensor in volts when the concentration of CO2 is 400PPM
#define MAX_POINT_VOLTAGE (0.265)	// (user defined) output of the sensor in volts when the concentration of CO2 is 10,000PPM
#define REACTION_VOLTGAE (0.059)	// (user defined) voltage drop of the sensor when move the sensor from air into 1000ppm CO2

float CO2Curve[3] = {ZERO_POINT_X, ZERO_POINT_VOLTAGE, (REACTION_VOLTGAE / (2.602 - 4))};

void setup() 
{	
  Serial.begin(9600);
  pinMode(BOOL_PIN, INPUT);
  digitalWrite(BOOL_PIN, HIGH);
  Serial.print("Starting Sensor Hub \n");

  softwareSerial.begin(9600);
  softwareSerial.setTimeout(1500);
  
  pinMode(RxD, INPUT);
  pinMode(TxD, OUTPUT); 
}

void loop() 
{
  // AIR QUALITY SENSOR 
  if(softwareSerial.find(0x42))
  {
    softwareSerial.readBytes(buf,LENG);
    if(buf[0] == 0x4d)
    {
      if(checkValue(buf, LENG))
      {
        PM01Value = transmitPM01(buf);
        PM2_5Value = transmitPM2_5(buf);
        PM10Value = transmitPM10(buf);
      }
    }
  }
	
  Serial.print("[AIR QUALITY] PM1.0: ");  
  Serial.print(PM01Value);
  Serial.print(" ug/m3  | ");            
    
  Serial.print("PM2.5: ");  
  Serial.print(PM2_5Value);
  Serial.print(" ug/m3  | ");     
      
  Serial.print("PM1 0: ");  
  Serial.print(PM10Value);
  Serial.print(" ug/m3");   

  Serial.println();

  // LIGHT SENSOR
  int lux;
  lux = analogRead(LIGHT_PIN);
  Serial.print("[AMBIENT LIGHT] ");
  Serial.print(lux,DEC);
  Serial.println(" Lux");

  // TEMPERATURE SENSOR
  int tempCelsius;
  int tempFahrenheit;
  tempFahrenheit = analogRead(TEMP_PIN);
  tempCelsius = (tempFahrenheit - 32) * 5 / 9;
  Serial.print("[TEMPERATURE] ");
  Serial.print(tempCelsius);
  Serial.println(" C");

  //CO2 SENSOR
  int particles;
  float volts;

  if (digitalRead(BOOL_PIN) ) 
  {
    Serial.print( "[CO2 LEVEL] HIGH " );
  } 
  else 
  {
    Serial.print( "[CO2 LEVEL] LOW " );
  }
  
  volts = MGRead(MG_PIN);
  particles = MGGetPPM(volts, CO2Curve);
  Serial.print("PPM: ");
  if (particles == -1) 
  {
    Serial.print("Outside range(400~10,000)");
  } 
  else 
  {
    Serial.print(particles);
  }  
  
  Serial.print( " | " );
  Serial.print( "Volts:" );
  Serial.print(volts);
  
  Serial.println();
  Serial.println();
  
  delay(1000);
}

//CO2 SENSOR functions
float MGRead(int mg_pin) 
{
  int i;
  float v = 0;
  
  for (i = 0; i < READ_SAMPLE_TIMES; i++) {
    v += analogRead(mg_pin);
    delay(READ_SAMPLE_INTERVAL);
  }
  
  v = (v / READ_SAMPLE_TIMES) * 5 / 1024 ;
  return v;
}

int MGGetPPM(float volts, float *pcurve) 
{
  volts = volts / DC_GAIN;
  
  if (volts > ZERO_POINT_VOLTAGE || volts < MAX_POINT_VOLTAGE ) 
  {
    return -1;
  } 
  else 
  {
    return pow(10, (volts - pcurve[1]) / pcurve[2] + pcurve[0]);
    volts = 0;
  }
}

// AIR QUALITY SENSOR functions
char checkValue(unsigned char *thebuf, char leng)
{  
  char receiveflag = 0;
  int receiveSum = 0;

  for(int i=0; i<(leng-2); i++)
  {
	receiveSum = receiveSum + thebuf[i];
  }
  
  receiveSum=receiveSum + 0x42;
 
  if(receiveSum == ((thebuf[leng-2]<<8) + thebuf[leng-1]))  //check the serial data 
  {
    receiveSum = 0;
    receiveflag = 1;
  }
  return receiveflag;
}

int transmitPM01(unsigned char *thebuf)
{
  int PM01Val;
  PM01Val = ((thebuf[3]<<8) + thebuf[4]);
  return PM01Val;
}

int transmitPM2_5(unsigned char *thebuf)
{
  int PM2_5Val;
  PM2_5Val = ((thebuf[5]<<8) + thebuf[6]);
  return PM2_5Val;
}

int transmitPM10(unsigned char *thebuf)
{
  int PM10Val;
  PM10Val = ((thebuf[7]<<8) + thebuf[8]);
  return PM10Val;
}
