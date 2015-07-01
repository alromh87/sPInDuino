/******************************************************************
 * PID Simple Example (Augmented with Processing.org Communication)
 * Version 0.3
 * by Brett Beauregard
 * License: Creative-Commons Attribution Share-Alike
 * April 2011
 ******************************************************************/

#include <PID_v1.h>

/* Definicion de pines */
#define INPUT_PIN                 0                //Referncia Analogica de valor deseado
#define TRIAC_CONTROL    5   // Triac control - pin
#define ZERO_DETECT        2       //Zero detect   - pin
#define  RPM_SENSOR        3       //RPM sensor - pin

#define RPM_TEST                7
#define ZERO_TEST              8

/* Definiciones para calculo de RPM */
#define MARCAS_SENSOR           8
#define RPM_MAX                    35000
#define REF_MAX                       1023

 // when using values in the main routine and IRQ routine must be volatile value
volatile byte zeroBit = LOW; // declare IRQ flag
volatile byte shoot = LOW;
 // HIGH = 1, LOW = 0

volatile unsigned long rpmcount = 0;

unsigned long time          = 0;
unsigned long timeold     = 0;
unsigned long shootAt     = 0;

unsigned long rpm;

unsigned int retraso;
float maxDelay = .9*(1000000/(float)120); //Retraso maximo: Perido de media señal senoidal en Microsegundos escalado al 90%
#define MIN_DELAY  500


//Define Variables we'll be connecting to
double Setpoint, Input, Output;

unsigned long serialTime=0; //this will help us know when to talk with processing

void setup()
{
  //initialize the serial link with processing
  Serial.begin(115200);

  pinMode(INPUT_PIN, INPUT);  
  //initialize the variables we're linked to

  Input = 0;
  rpm =0;

pinMode(RPM_TEST, OUTPUT); 
pinMode(ZERO_TEST, OUTPUT); 
  Setpoint = analogRead(INPUT_PIN);
//Triac control setup  
  pinMode(TRIAC_CONTROL, OUTPUT);  
  digitalWrite(TRIAC_CONTROL, 0); // LED off
  
  //Zero detect  
  pinMode(ZERO_DETECT, INPUT);
  digitalWrite(ZERO_DETECT, 1); // pull up on
  attachInterrupt(0, zero_fun, RISING);  // interrupt 0 digital pin 2 connected ZC circuit
// Hall sensor  
  pinMode(RPM_SENSOR, INPUT);
  digitalWrite(RPM_SENSOR, 1); // pull up on
  attachInterrupt(1, rpm_fun, CHANGE);  // interrupt 1 digital pin 3 connected hall sensor
}

void loop(){
    Setpoint    = analogRead(INPUT_PIN);
//    Setpoint    = (analogRead(INPUT_PIN)*RPM_MAX)/(float)REF_MAX;

  if(Setpoint<10){
    Input=0;
    shoot=LOW;
    return;
  }
  
  if(shoot){
    if(micros() >= shootAt){
      digitalWrite(TRIAC_CONTROL, 1); //triac on 
      delayMicroseconds(100); 
      digitalWrite(TRIAC_CONTROL, 0);  //triac off 
      shoot = LOW;
    }
  }
//TRIAC delay control      
  if (zeroBit == 1){

    time = micros();
    rpm = ((1000000*(float)rpmcount / (float)(time-timeold))*60)/ MARCAS_SENSOR;
    timeold  = micros(); //set time
    rpmcount = 0; //reset

    Input = (rpm/(float)RPM_MAX)*REF_MAX;    
//    Input =rpm;

 int out[] = {6800,6750,6500,6300,6000,5500,4700,3000};

  int in[]  = {0,3772,67500,12300,92250,16275,22500,35000};
    retraso=multiMap((analogRead(INPUT_PIN)*RPM_MAX)/(float)REF_MAX, in, out, 7);

    if(retraso<MIN_DELAY)
      retraso = MIN_DELAY;
    shoot = HIGH;
    shootAt=micros()+retraso;

    zeroBit = 0; // clear flag
  }
  
  //send-receive with processing if it's time
  if(millis()>serialTime){
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }

}


/********************************************
 * Serial Communication functions / helpers
 ********************************************/


union {                // This Data structure lets
  byte asBytes[24];    // us take the byte array
  float asFloat[6];    // sent from processing and
}                      // easily convert it to a
foo;                   // float array



// getting float values from processing into the arduino
// was no small task.  the way this program does it is
// as follows:
//  * a float takes up 4 bytes.  in processing, convert
//    the array of floats we want to send, into an array
//    of bytes.
//  * send the bytes to the arduino
//  * use a data structure known as a union to convert
//    the array of bytes back into an array of floats

//  the bytes coming from the arduino follow the following
//  format:
//  0: 0=Manual, 1=Auto, else = ? error ?
//  1: 0=Direct, 1=Reverse, else = ? error ?
//  2-5: float setpoint
//  6-9: float input
//  10-13: float output  
//  14-17: float P_Param
//  18-21: float I_Param
//  22-245: float D_Param
void SerialReceive()
{

  // read the bytes sent from Processing
  int index=0;
  byte Auto_Man = -1;
  byte Direct_Reverse = -1;
  while(Serial.available()&&index<26)
  {
    if(index==0) Auto_Man = Serial.read();
    else if(index==1) Direct_Reverse = Serial.read();
    else foo.asBytes[index-2] = Serial.read();
    index++;
  } 
  
  // if the information we got was in the correct format, 
  // read it into the system
  if(index==26  && (Auto_Man==0 || Auto_Man==1)&& (Direct_Reverse==0 || Direct_Reverse==1))
  {
    Setpoint=double(foo.asFloat[0]);
    //Input=double(foo.asFloat[1]);       // * the user has the ability to send the 
                                          //   value of "Input"  in most cases (as 
                                          //   in this one) this is not needed.
    if(Auto_Man==0)                       // * only change the output if we are in 
    {                                     //   manual mode.  otherwise we'll get an
      Output=double(foo.asFloat[2]);      //   output blip, then the controller will 
    }                                     //   overwrite.
    
    double p, i, d;                       // * read in and set the controller tunings
    p = double(foo.asFloat[3]);           //
    i = double(foo.asFloat[4]);           //
    d = double(foo.asFloat[5]);           //
    
  }
  Serial.flush();                         // * clear any random data from the serial buffer
}

// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting
// floats from processing to here no?
void SerialSend()
{
  Serial.print("PID ");
  Serial.print(Setpoint);   
  Serial.print(" ");
  Serial.print(Input);   
  Serial.print(" ");
  Serial.print(Output);   
  Serial.print(" ");
  Serial.print(0);   
  Serial.print(" ");
  Serial.print(0);   
  Serial.print(" ");
  Serial.print(0);   
  Serial.print(" ");
  Serial.print("Manual");  
  Serial.print(" ");
  Serial.print("Reverse");
  Serial.print("  ");
  Serial.println(retraso);
}

void zero_fun(){
  zeroBit = 1;
  // if zero detect set bit == 1
  digitalWrite(ZERO_TEST,!digitalRead(ZERO_TEST));
  shoot =LOW;
} 
 
void rpm_fun(){
  //Each rotation, this interrupt function is run
   rpmcount++;
   digitalWrite(RPM_TEST,!digitalRead(RPM_TEST));
 }
 
 int multiMap(int val, int* _in, int* _out, uint8_t size)
{
  // take care the value is within range
  // val = constrain(val, _in[0], _in[size-1]);
  if (val <= _in[0]) return _out[0];
  if (val >= _in[size-1]) return _out[size-1];

  // search right interval
  uint8_t pos = 1;  // _in[0] allready tested
  while(val > _in[pos]) pos++;

  // this will handle all exact "points" in the _in array
  if (val == _in[pos]) return _out[pos];

  // interpolate in the right segment for the rest
  return (val - _in[pos-1]) * (_out[pos] - _out[pos-1]) / (_in[pos] - _in[pos-1]) + _out[pos-1];
}
