/******************************************************************
 * PID Simple Example (Augmented with Processing.org Communication)
 * Version 0.3
 * by Brett Beauregard
 * License: Creative-Commons Attribution Share-Alike
 * April 2011
 ******************************************************************/

#include <PID_v1.h>

//#define DEBUG 1

/* Definicion de pines */
#define INPUT_PIN                 0                //Referncia Analogica de valor deseado
#define TRIAC_CONTROL    5   // Triac control - pin
#define ZERO_DETECT        2       //Zero detect   - pin
#define  RPM_SENSOR        3       //RPM sensor - pin

/* Definiciones para calculo de RPM */
#define MARCAS_SENSOR           8
#define RPM_MAX                    35000
#define REF_MAX                       1023

 // when using values in the main routine and IRQ routine must be volatile value
volatile byte zeroBit = LOW; // declare IRQ flag
 // HIGH = 1, LOW = 0
  
unsigned long rpmcount;
unsigned long rpm;
unsigned long time;
unsigned long timeold;
float impuls_time;

unsigned long timeMark = 0;
int rpm2=0;
  
//unsigned int retraso;
float retraso;
float maxDelay = .6*(1000000/(float)120); //Retraso maximo: Perido de media señal senoidal en Microsegundos escalado al 90%


//Define Variables we'll be connecting to
double Setpoint, Input, Output;

    //Define the aggressive and conservative Tuning Parameters
    //double consKp=4, consKi=0.2, consKd=1;
    //double consKp=0.4, consKi=0.001, consKd=1;
    //double consKp=4, consKi=0.2, consKd=1;
    double consKp=1, consKi=0.4, consKd=0.01; //100% work
    //double consKp=0.4, consKi=0.001, consKd=1;
    //double consKp=0.1, consKi=0.5, consKd=0.05;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,consKp, consKi, consKd, REVERSE);

unsigned long serialTime; //this will help us know when to talk with processing

void setup()
{
  //initialize the serial link with processing
  Serial.begin(115200);
//  Serial.begin(9600);
  
  //initialize the variables we're linked to
  Input = analogRead(INPUT_PIN);
  Setpoint = 100;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

//Triac control setup  
  pinMode(TRIAC_CONTROL, OUTPUT);  
  digitalWrite(TRIAC_CONTROL, 0); // LED off
  
  //Zero detect  
  pinMode(ZERO_DETECT, INPUT);
  digitalWrite(ZERO_DETECT, 1); // pull up on
  attachInterrupt(0, zero_fun, FALLING);  // interrupt 0 digital pin 2 connected ZC circuit
// Hall sensor  
  pinMode(RPM_SENSOR, INPUT);
  digitalWrite(RPM_SENSOR, 1); // pull up on
  attachInterrupt(1, rpm_fun, FALLING);  // interrupt 1 digital pin 3 connected hall sensor

  Serial.print("Iniciado ");
}

void loop(){

//send-receive with processing if it's time

#ifndef DEBUG
  if(millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }
#endif
  
//  Setpoint    = (analogRead(INPUT_PIN)*35000)/(float)REF_MAX;
    Setpoint    = analogRead(INPUT_PIN);
if(Setpoint<255)return;
//TRIAC delay control      
  if (zeroBit == 1){
    
    //PID   
    time = micros() - timeold;
    float time_in_sec  = (float)time / 1000000;
    impuls_time = (float)rpmcount / time_in_sec;
    rpm                = impuls_time*60 / MARCAS_SENSOR;

    timeold  = micros(); //set time
    rpmcount = 0; //reset
  
    Input = (rpm/(float)RPM_MAX)*REF_MAX;
  
    myPID.Compute();

//    retraso = (Output*(float)maxDelay)/255;
    retraso = (Output*(float)maxDelay)/255.0;
    
    if(retraso>0)
      delayMicroseconds((int)retraso);
    digitalWrite(TRIAC_CONTROL, 1); //triac on 
    delayMicroseconds(100); 
    digitalWrite(TRIAC_CONTROL, 0);  //triac off 
    zeroBit = 0; // clear flag

    #ifdef DEBUG
      Serial.print(rpmcount);
      Serial.print(" marcas en ");
      Serial.print(time);
      Serial.print("us ( ");
      Serial.print(impuls_time);
      Serial.print(") RPM: ");
      Serial.print(rpm);
      Serial.print(" vs  ");
      Serial.print(rpm2);
      Serial.print("   Retraso: ");
      Serial.println(retraso);
    #endif
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
    myPID.SetTunings(p, i, d);            //
    
    if(Auto_Man==0) myPID.SetMode(MANUAL);// * set the controller mode
    else myPID.SetMode(AUTOMATIC);             //
    
    if(Direct_Reverse==0) myPID.SetControllerDirection(DIRECT);// * set the controller Direction
    else myPID.SetControllerDirection(REVERSE);          //
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
  Serial.print(myPID.GetKp());   
  Serial.print(" ");
  Serial.print(myPID.GetKi());   
  Serial.print(" ");
  Serial.print(myPID.GetKd());   
  Serial.print(" ");
  if(myPID.GetMode()==AUTOMATIC) Serial.print("Automatic");
  else Serial.print("Manual");  
  Serial.print(" ");
  if(myPID.GetDirection()==DIRECT) Serial.print("Direct");
  else Serial.print("Reverse");
    Serial.print("  ");
    Serial.println(retraso);
}

void zero_fun(){
    zeroBit = 1;
    // if zero detect set bit == 1
  } 

void rpm_fun(){
   unsigned long timeT = micros();
   rpm2 =(1000000/(8.0*(timeT-timeMark)))*60;
   rpmcount++;
   timeMark = timeT;
   //Each rotation, this interrupt function is run
 }
