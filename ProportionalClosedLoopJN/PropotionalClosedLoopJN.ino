/* Example of driving servomotor and reading encoder signals in various ways */

#include <avr/io.h>  /* Needed to set up counter on pin 47 */
#include <SPI.h>     /* Needed to communicate with LS7366R (Counter Click) */

/* Serial input aspects are based closely upon: 
   http://forum.arduino.cc/index.php?topic=396450
   Example 4 - Receive a number as text and convert it to an int
   Modified to read a float */

/* LS7366R aspects very loosely based on concepts used in controlling
   the Robogaia 3-axis encoder shield though implementation is very different
   https://www.robogaia.com/3-axis-encoder-conter-arduino-shield.html */

/* Counting using Timer 5 (external counter input) based loosely on code from 
  https://forum.arduino.cc/index.php?topic=59396.0 written by bubuldino */

/* Pins used for L298 driver */
const int enA=13;      /* PWM output, also visible as LED */
const int in1=8;       /* H bridge selection input 1 */
const int in2=9;       /* H bridge selection input 2 */
const float minPercent=-100.0;
const float maxPercent=100.0;

/* Encoder input pins (used for state machine and interrupts)
   Not really needed but may be used just to provide pullup resistors */
const int channelA=2;
const int channelB=3;

/* Used to to initiate SPI communication to LS7366R chip (Counter click) */
const int chipSelectPin=10;

/* Size of buffer used to store received characters */
enum {numChars=32};

/* Intervals in milliseconds for user-defined timed loops */
const int printInterval=1000;           

/* Global variables used in serial input */ 
char receivedChars[numChars];   // an array to store the received data
float dataNumber = 0;             // new for this version
boolean newData = false;

/* Global variables used for motor control and encoder reading */
double percentDutyCycle;
double measuredPosnFromEncoder;

/* Global variables used for loop timing */
unsigned long prevMillisPrint = 0;        /* stores last time values were printed */

/* Overlapping regions of memory used to convert four bytes to a long integer */
union fourBytesToLong
{
  long result;
  unsigned char bytes [4];
};

//control loop global varables
const long controlInterval = 20;
unsigned long prevControlMillis = 0;
double encoderPosition;
int Kp = 0.02; //proportional gain of a PID

float positionSetPoint;

//-------------------------------------------------SETUP-----------------------------------------------------
void setup() 
{
  Serial.begin(9600);
  Serial.println("Enter PWM duty cycle as a percentage (positive for forward, negative for reverse");

  /* Set encoder pins as input but with pullup resistors to be compatible with various encoders 
     Note: not strictly needed if state machine input is not being used, but retained in case we need
     easy access to pull-up resistors for encoders that do not include them */
  pinMode(channelA, INPUT_PULLUP);
  pinMode(channelB, INPUT_PULLUP);
  
  /* Set up and initialise pin used for selecting LS7366R counter: hi=inactive */
  pinMode(chipSelectPin, OUTPUT);   
  digitalWrite(chipSelectPin, HIGH);

  SetUpLS7366RCounter();

  delay(100);

  /* Configure control pins for L298 H bridge */
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  /* Set initial rotation direction */
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}


//------------------------------------------------ - LOOP - -----------------------------------------------------
void loop() 
{
  unsigned long currentMillis = millis();

  //call control loop function
  controlLoop();

  if (currentMillis - prevMillisPrint >= printInterval) 
  {
    // save the last time you printed output
    prevMillisPrint = currentMillis;
    printLoop();
  }
  
  /* Don't change these two lines, they decide when there is a new value just read from the
   *  serial input and allow you to take action based on this new value  */
  recvWithEndMarker();
  if(convertNewNumber())
  /*  Update value read from serial line  */
  {
     //serial.read()-> rc -> recievedChars array -> data number
     positionSetPoint=dataNumber; //desired position
  }
}


//-----------------------------------FUNCTIONS----------------------------------------------------------------
void driveMotorPercent(double percentDutyCycle)
/* Output PWM and H bridge signals based on positive or negative duty cycle % */
{
      percentDutyCycle = constrain(percentDutyCycle, -100, 100);
      int regVal = map(percentDutyCycle, -100, 100, -255, 255);
      analogWrite(enA, (int)abs(regVal));
      digitalWrite(in1, regVal>0);
      digitalWrite(in2, !(regVal>0));
}

//loop to control encoder
void controlLoop()
{
  unsigned long currentMillis = millis(); //update current ime 
  double error; //error passed to controller
  
  if (currentMillis - prevControlMillis >= controlInterval)
  {
    prevControlMillis = currentMillis; //saves the last time the if statement was used
    encoderPosition = readEncoderCountFromLS7366R(); //recieve current encoder position
    
    error = positionSetPoint - encoderPosition; //calculate closed loop error
    percentDutyCycle = Kp*error; //corrective action by controller

    driveMotorPercent(percentDutyCycle); //set motor speed
  }
}

/* Print count and control information */
void printLoop()
{
   double error;
   /* Sample all counters one after the other to avoid delay-related offsets */
   measuredPosnFromEncoder = readEncoderCountFromLS7366R();
   error = positionSetPoint - measuredPosnFromEncoder;
   Serial.print("Desired position = ");
   Serial.print(positionSetPoint);
   Serial.print("\t  ");
   Serial.print("Count from LS7366R = ");
   Serial.print(measuredPosnFromEncoder);
   Serial.print("\t");
   Serial.print("Error = ");
   Serial.print(error);
   Serial.print("\r\n");
}
 
long readEncoderCountFromLS7366R()
/* Reads the LS7366R chip to obtain up/down count from encoder.  Reads four
   bytes separately then concverts them to a long integer using a union */
{
    fourBytesToLong converter; /* Union of four bytes and a long integer */
    
    digitalWrite(chipSelectPin,LOW); /* Make LS7366R active */
    
    SPI.transfer(0x60); // Request count
    converter.bytes[3] = SPI.transfer(0x00); /* Read highest order byte */
    converter.bytes[2] = SPI.transfer(0x00); 
    converter.bytes[1] = SPI.transfer(0x00); 
    converter.bytes[0] = SPI.transfer(0x00); /* Read lowest order byte */
    
    digitalWrite(chipSelectPin,HIGH); /* Make LS7366R inactive */
   
    return converter.result;
}

void SetUpLS7366RCounter(void)
/* Initialises LS7366R hardware counter on Counter Click board to read quadrature signals */
{
    /* Control registers in LS7366R - see LS7366R datasheet for this and subsequent control words */
    unsigned char IR = 0x00, MRD0=0x00;
    
    // SPI initialization
    SPI.begin();
    //SPI.setClockDivider(SPI_CLOCK_DIV16);      // SPI at 1Mhz (on 16Mhz clock)
    delay(10);

   /* Configure as free-running 4x quadrature counter */
   digitalWrite(chipSelectPin,LOW); /* Select chip and initialise transfer */
   /* Instruction register IR */
   IR |= 0x80;   /* Write to register (B7=1, B6=0) */
   IR |= 0x08;   /* Select register MDR0: B5=0, B4=0, B3=1 */
   SPI.transfer(IR); /* Write to instruction register */ 
   /* Mode register 0 */
   MRD0 |= 0x03;    /* 4x quadrature count: B0=1, B1=1 */
   /* B2=B3=0: free running.  B4=B5=0: disable index. */
   /* B6=0: asynchronous index.  B7: Filter division factor = 1. */
   SPI.transfer(MRD0);
   digitalWrite(chipSelectPin,HIGH); 

   /* Clear the counter i.e. set it to zero */
   IR = 0x00; /* Clear the instructino register IR */
   digitalWrite(chipSelectPin,LOW); /* Select chip and initialise transfer */
   IR |= 0x20; /* Select CNTR: B5=1,B4=0,B3=0; CLR register: B7=0,B6=0 */
   SPI.transfer(IR); /* Write to instruction register */ 
   digitalWrite(chipSelectPin,HIGH); 
}

void recvWithEndMarker() 
/* Receive data from serial port finishing with "newline" character. 
   Based on http://forum.arduino.cc/index.php?topic=396450 Example 4 */
{
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;

    //serial.available = Get the number of bytes available for reading from the serial port
    if (Serial.available() > 0) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}

bool convertNewNumber() 
/* Converts character string to floating point number only if there are new
       data to convert, otherwise returns false */
{
    if (newData) {
        dataNumber = 0.0;             
        dataNumber = atof(receivedChars);   
        newData = false;
        return true;
    }
    else
    {
       return false;
    }
}
