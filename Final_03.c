  /* Ping Sensor & ActivityBot program to demonstrate a simple proportional feedback control system to
 * allow the ActivityBot to navigate by using echo location to sense the distance to an adjacent wall
 * and to maintain a constant distance between the Ping sensor and the wall while traveling.
 * 
 * This program also sets the orientation of a servomotor attached to pin BALL_KNOCKER_MOTOR.
 * The position is static and held until the fixed travel time elapses, then it returns to the original orientation.
 * 
 * This version of the program is incomplete, and does not fully implement: 
 * Ping reading outlier control
 * Missing wall section detection or mitigation
 * detection of obstacles other than walls
 * use of a full PID control algorithm.
 * 
 * Test the PING))) sensor before using it to navigate with the ActivityBot.
 *
 *  http://learn.parallax.com/activitybot/build-and-test-ping-sensor-circuit
 *
 * IBM’s C library reference: https://www.ibm.com/support/knowledgecenter/ssw_ibm_i_71/rtref/sc41560702.htm  */


//Libraries
#include "simpletools.h"                      // Include simpletools header
#include "abdrive360.h"
#include "ping.h"                             // Include ping header
#include "fdserial.h" // added from LFR approach feedback 
#include "wifi.h"
#include "adcACpropab.h"      // Include Analog to Digital Converter Library

// Constant values used in the system control / geometric parameters of the robot
#define TRAVELSPEED          200 
#define SIDE_SETPOINT_MM     110 
#define KP_HEADING           1.5  // scaled for Ping data in mm units
#define MM_PER_TICK         3.25
#define SETPOINT_TOLERANCE    5
#define SERVO_OFFSET          20 //47?
#define RETRACTED_ANGLE        0  // Initial angle for servomotor
#define KNOCKING_ANGLE        40  // Angle to knock balls from pegs
#define CAPTURE_ANGLE         85  // Angle to capture balls
#define DISABLE                0  // Used to turn off the servo feedback
#define WHEEL_CIRCUMFERENCE_MM 208
#define FORWARD_MM_PER_SECOND 50

// Pin assignments
#define VOLT_REG_ENABLE       00   // Pin to enable line of the 3.3 V power regulator
#define LED_PULSE             01   // Pin to the MOSFET gate that controls the LED light output
#define RIGHT_WHEEL_FEEDBACK  15
#define LEFT_WHEEL_FEEDBACK   14
#define LEFT_WHEEL            12
#define RIGHT_WHEEL           13
#define BALL_KNOCKER_MOTOR    16
#define BALL_KNOCKER_FEEDBACK 17
#define PING_BACK_PIN         10
#define PING_FRONT_PIN        11
#define SDA_PIN               04   // Set SDA of Gyroscope, orange wire
#define SCL_PIN               05   // Set SCL of Gyroscope, yellow wire
#define VOLT_REG_ENABLE       00   // Pin to enable line of the 3.3 V power regulator
#define LED_PULSE             01   // Pin to the MOSFET gate that controls the LED light output
#define LED_PULSE             01   // Pin to the MOSFET gate that controls the LED light output
#define LRF_rxPin             02   // Serial input (connects to the LRF's SOUT pin) yellow wire
#define LRF_txPin             03   // Serial output (connects to the LRF's SIN pin) orange wire


// LFR response feedback
#define rxPin    02  // Serial input (connects to the LRF's SOUT pin)
#define txPin    03  // Serial output (connects to the LRF's SIN pin)
#define BAUD  38400  // Baud rate for Laser Range Finder
#define FORWARD_SETPOINT_MM 254
#define DEADBAND 26

//Gyroscope definitions
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x28
#define CTRL_REG6 0x29
#define CTRL_REG7 0x2A
#define CTRL_REG8 0x2B
#define CTRL_REG9 0x2C
#define CTRL_REGA 0x2D
#define STATUS    0x27
#define INERTIAL_OFFSET_ANGLE_DEGREES 5
#define GYROSCOPE_SCALE_FACTOR 110.0
#define ROTATION_DIRECTION_LEFT -1    
#define ROTATION_DIRECTION_RIGHT 1

// Global varibles from gyro
i2c *bus_Id;                                   // create a variable for the I2C bus ID
int rotationalVel, z, zOffset, t;
int Addr = 105;           // I2C address of Gyroscope
float zSum, degreesZ, zTotal;
static volatile int og_diff, og_sum, og_sum_max, Dwhite, GrnLDF, LtPink, RedLDF, Wwhite;
static volatile float og_ratio;

// Global variables for cogs
static volatile int distance_mm, heading_mm, state = 1, Front_mm, Back_mm;        
static unsigned int Dstack[40 + 128]; // stacks for cogs, add more memory if you add variables or code.
static volatile int sideError[2], controlSignal;
static volatile int object;

// Declare Functions
void initialize();
void getMeasurement();  // function to get a new LRF measurement
void getPing();
void gyroSetup();
void gyroMeasure();
void Navigation();
void ballRec();
void appleHarvesting();

//Global variables for ping sensor
int Right_mm, Left_mm;
int start = 1; // Start navigation
float oldHeading = 0.0, relativeHeading = 0.0;

// Global variable for LRF
fdserial *lrf;
static volatile int iLRF, distance;
char LRF_in1[1];
static unsigned int stack[40+200];// memory for LRF cog
 
static volatile int Forward_Error, Control_Signal;

//Ball recgonition Stack
static unsigned int Cstack[40+200];// memory for LRF cog 

// Apple Harvesting stack
static unsigned int Estack[40 + 128]; // stacks for cogs, add more memory if you add variables or code.

//navigation

//Color
static int adcVal[4];                      // 4 element array for storing analog values


int main(){                                // main function
    
  initialize();
  gyroSetup(); // Call function to set the gyroscope
  cogstart(getPing, NULL, Dstack, sizeof(Dstack)); // start measuring side distance
  cogstart(getMeasurement, NULL, stack, sizeof(stack)); //start measuring forward distancce
  pause(1000);
  cogstart(ballRec, NULL, Cstack, sizeof(Cstack)); //start recognizing the colors
  pause(2000);
  cogstart(appleHarvesting, NULL, Estack, sizeof(Estack)); //knock the balls
  //print("og_diff og_sum og_ratio\n"); 
  
  while(1) {        // Travelling straight, tracking wall 

    Navigation();
 }
}

void getPing() { // separate cog function for continuous Ping distance measurements
  int echoTimeFront, echoTimeBack;
  while(state > -2) { // state variable allows you to turn off the Ping sensors
    pause(5);
    echoTimeFront = ping(PING_FRONT_PIN);  // Get object echo time from front Ping sensor
    pause(5);
    echoTimeBack  = ping(PING_BACK_PIN);   // Get object echo time from front Ping sensor
    Front_mm = (echoTimeFront*1713 - 38395 + 5000)/10000; // Regression equation for echo time to mm
    Back_mm  = (echoTimeBack*1713 - 38395 + 5000)/10000;  // Regression equation for echo time to mm
    heading_mm = Front_mm - Back_mm;                      // Calculate the relative heading
    distance_mm = (Front_mm + Back_mm +1) / 2;            // Calculate the average distance to object
   }  
}  

void initialize() { //simple commands that need to be executed
  
  servo360_connect(BALL_KNOCKER_MOTOR, BALL_KNOCKER_FEEDBACK);    // setup to use the servomotor 
  servo360_connect(LEFT_WHEEL, LEFT_WHEEL_FEEDBACK);    // setup to use the wheel servomotors  
  servo360_connect(RIGHT_WHEEL, RIGHT_WHEEL_FEEDBACK);  
  //wifi_start(31, 30, 115200, WX_ALL_COM); // use WiFi for programming and terminal communication

  // Setup serial communication with the LRF
  lrf = fdserial_open(rxPin, txPin, FDSERIAL_MODE_NONE, BAUD);  //Open a full duplex serial connection. 

  pause(50); // Wait for LRF to turn on.
  
  fdserial_txChar(lrf, 'U');
  
  // When the LRF has initialized and is ready, it will send a ':' character, 

  while((iLRF = fdserial_rxCount(lrf)) < 2); // wait here for 3 characters to be transmitted.
  
  pause(10); // delay in case of a warm start, 3 character, response.
  
  iLRF = fdserial_rxCount(lrf); // get number of characters in the buffer.
  
  while(iLRF > 0) {
      LRF_in1[0] = fdserial_rxChar(lrf);
//      putchar(LRF_in1[0]);          // Display result
      iLRF = fdserial_rxCount(lrf); // get number of characters in the buffer.
      pause(1);
    }
    
          adc_start(19, 18, 20, 21,                // CS=21, SCL=20, DO=19, DI=18 
                0b0011,                        // Ch3 off, 2 off, 1 on, 0 on 
                adcVal);
      pause(1);        
}  // End of initialize.

void getMeasurement(){

 while(1){
    
      fdserial_txChar(lrf, 'B');              // Send LRF command to take a measurement
      while((iLRF = fdserial_rxCount(lrf)) < 4); // wait here for LRF reply
  
        LRF_in1[0] = fdserial_rxChar(lrf);   // get 1st byte of measurement
        distance = LRF_in1[0] << 8;          // shift to upper byte position
        iLRF--;                                 // decrement i
        LRF_in1[0] = fdserial_rxChar(lrf);   // get 2nd byte of measurement
        distance = distance | LRF_in1[0];    // concatinate the 2 bytes
  
        iLRF--;                                 // decrement i
      while(iLRF > 0) {
         LRF_in1[0] = fdserial_rxChar(lrf);  // read remaining characters in message
  //      print("%c", LRF_in1[0]);           // Optional, Display result
         iLRF--;
         }
        
        }
  
}

void Navigation(){
  float rotationalVel = (int) ( (360.0*((float) FORWARD_MM_PER_SECOND)/((float) WHEEL_CIRCUMFERENCE_MM)) +0.5);
      //printi("og_diff %d, og_sum %d, og_ratio %d, Object %d\n", og_diff, og_sum, og_ratio, object);
      
      if(start == 1){
        
        switch(state){
      
      case 1:
      
        if ((distance > 100) && (distance <2400)){ // trusty range of measurements

          if (distance<=620 && distance>550){ // robot reached the end of the row (last stick)
 
            servo360_speed(LEFT_WHEEL, 0);
            servo360_speed(RIGHT_WHEEL, 0);
            controlSignal = 0; 
            //pause(1000);
            state = 2;  //chechink state
 
          }  
          else{ //proportional control for traveling
  
              //appleHarvesting();
              sideError[0] = (SIDE_SETPOINT_MM - distance_mm) - heading_mm; // 
              if(Front_mm < 45) controlSignal = 50;  // Risk of crashing when Front < 30 mm.
              
              else {
                                  
               if(sideError[0] > 0)  controlSignal = (int) (KP_HEADING * (float) (sideError[0] +SERVO_OFFSET) +0.5);
               if(sideError[0] < 0)  controlSignal = (int) (KP_HEADING * (float) (sideError[0] -SERVO_OFFSET) +0.5);
               if(controlSignal > 300) controlSignal = 300;
               if(controlSignal < -300) controlSignal = -300;
              } 
             
            if(abs(sideError[0]) > SETPOINT_TOLERANCE) {
                servo360_speed(LEFT_WHEEL, TRAVELSPEED +controlSignal);
                servo360_speed(RIGHT_WHEEL  , -TRAVELSPEED +controlSignal);
                }
            else {
                  servo360_speed(LEFT_WHEEL, TRAVELSPEED);
                  servo360_speed(RIGHT_WHEEL, -TRAVELSPEED);    
              }   

            }
            break; //end of case 1      
       
      case 2: //checking step to make sure the robot did not get anything wrong
      
        if(distance>=620){
          state = 1; //It corrects the fact that the robot might have wrongly executed a measurement
        }          
        
        if ((distance > 100) && (distance <2400)){ // trusty range of measurements
   
          if (distance<=250 && distance>0){ // time to turn
 
            servo360_speed(LEFT_WHEEL, 0); //first stop the robot in order to make a nice and smooth turning operation
            servo360_speed(RIGHT_WHEEL, 0);
            controlSignal = 0; 
            state = 3;  //turning
 
          }  
          else{ // if it everything was correct, just keep moving forward
              
             servo360_speed(LEFT_WHEEL, TRAVELSPEED);
             servo360_speed(RIGHT_WHEEL, -TRAVELSPEED);    
       

            } //end of else
           
        }  //end of if
        break;  
                  
      case 3: //turning state
      
         servo360_speed(LEFT_WHEEL, ROTATION_DIRECTION_RIGHT * rotationalVel);
         servo360_speed(RIGHT_WHEEL, ROTATION_DIRECTION_RIGHT * rotationalVel);
         
         t = CNT;
         zSum=0.0; 
         zTotal=0.0;
         while( fabs(zTotal) < fabs((87) - INERTIAL_OFFSET_ANGLE_DEGREES)) //the robot needs to turn 90 degrees, however, just adjusting the INERTIAL_OFFSET_ANGLE did not seem to work, this the "why" of 87 degrees
         { 
           gyroMeasure();
         } // End of while loop  
        
        // Stop the servomotors
         servo360_speed(LEFT_WHEEL, 0);
         servo360_speed(RIGHT_WHEEL, 0);
         
           state = 4; // after turning move forward (unilt you have to turn again)
           
           break;
           
      case 4:
      
            servo360_speed(LEFT_WHEEL, TRAVELSPEED);//start moving
            servo360_speed(RIGHT_WHEEL, -TRAVELSPEED);
            pause(65000); // don't do anything for 6.5 s, just move until it's time for the next turn
            servo360_speed(LEFT_WHEEL, 0);
            servo360_speed(RIGHT_WHEEL,0);
            state = 5;
            
            break;
                        
      case 5: //second turning
        
         servo360_speed(LEFT_WHEEL, ROTATION_DIRECTION_RIGHT * rotationalVel);
         servo360_speed(RIGHT_WHEEL, ROTATION_DIRECTION_RIGHT * rotationalVel);
         
         t = CNT; // record starting system clock
         zSum=0.0; 
         zTotal=0.0;
         while( fabs(zTotal) < fabs((87) - INERTIAL_OFFSET_ANGLE_DEGREES)){ //same explanation
 
           gyroMeasure();
         } // End of while loop  
        
            servo360_speed(LEFT_WHEEL, TRAVELSPEED); //after turning move again
            servo360_speed(RIGHT_WHEEL, -TRAVELSPEED);
            pause(2500);
            servo360_speed(LEFT_WHEEL, 0);
            servo360_speed(RIGHT_WHEEL,0);
            controlSignal = 0;
            state = 6; //restart navigation process
            
            break;
      
      case 6: //same thing as case 1 but after it's run it ends up the program (instead of going to state 2)

        if ((distance > 100) && (distance <2400)){ 
  
          if (distance<=620 && distance>550){ // 
       
            servo360_speed(LEFT_WHEEL, 0);
            servo360_speed(RIGHT_WHEEL, 0);
            controlSignal = 0; 
 
          }  
          else {

              sideError[0] = (SIDE_SETPOINT_MM - distance_mm) - heading_mm; // 
              if(Front_mm < 45) controlSignal = 50;  // Risk of crashing when Front < 30 mm.
              else {
                                  
               if(sideError[0] > 0)  controlSignal = (int) (KP_HEADING * (float) (sideError[0] +SERVO_OFFSET) +0.5);
               if(sideError[0] < 0)  controlSignal = (int) (KP_HEADING * (float) (sideError[0] -SERVO_OFFSET) +0.5);
               if(controlSignal > 300) controlSignal = 300;
               if(controlSignal < -300) controlSignal = -300;
              } 
             
            if(abs(sideError[0]) > SETPOINT_TOLERANCE) {
                servo360_speed(LEFT_WHEEL, TRAVELSPEED +controlSignal);
                servo360_speed(RIGHT_WHEEL  , -TRAVELSPEED +controlSignal);
                }
            else {
                  servo360_speed(LEFT_WHEEL, TRAVELSPEED);
                  servo360_speed(RIGHT_WHEEL, -TRAVELSPEED);    
              } //end of else  
             } //end of first if (620 && 550)
           } //end of if (d>100 && d<250)

          break;
        }          
        } // end of switch          
       } // end of if(start)          
      }  //end of navigation                               

void gyroSetup() { // function to setup the operating parameters for the gyroscope
  int j, dataNotReady;
  
  char testStr1[1] = {0};   // I2C data string 1  
  char testStr2[1] = {0};   // I2C data string 2 
  char testStr3[1] = {0};   // I2C data string 3 
  unsigned char dataout[1]; // Setup single character varible for I2C communication  

  bus_Id = i2c_newbus(SCL_PIN,  SDA_PIN,   0);      // Set up I2C bus, and store the bus ID

                                                    // Use bus_Id to write to gyroscope
  dataout[0] = 0x08;                                // Control ready signal
  while( i2c_busy(bus_Id, Addr) );                  // Wait for i2c bus to become available.
  i2c_out(bus_Id, Addr, CTRL_REG3, 1, dataout, 1);  // Enable control ready signal

  dataout[0] = 0x80;
  while( i2c_busy(bus_Id, Addr) );                  // Wait for i2c bus to become available.
  i2c_out(bus_Id, Addr, CTRL_REG4, 1, dataout, 1);  // Set scale (250 deg/sec)
  // For 500 deg/sec use 0x90, for 2000 dps use 0xA0.
  
  // Write to CTRL_REG1 last
  dataout[0] = 0x1F;
  while( i2c_busy(bus_Id, Addr) );   // Wait for i2c bus to become available.
  i2c_out(bus_Id, Addr, CTRL_REG1, 1, dataout, 1);  // Turn on all axes, disable power down
  // Output Data Rate = 100 Hz, Cut-off = 25
  // to only use the Z axis the value would be 1C

pause(100);  // wait to synchronize

// Determine the offset value for the z-axis of the Gyroscope when the robot is stationary.
  for(j=0; j<128; j++) { // loop 128 (2**7) times   
    testStr1[0] = '0';   // Clear the data string 1
    testStr2[0] = '0';   // Clear the data string 2
/* Z axis data */
    while( i2c_busy(bus_Id, Addr) ); // Wait for i2c bus to become available.
    dataNotReady = 1;
    while(dataNotReady != 4) {  
     i2c_in(bus_Id, Addr, STATUS, 1, testStr3, 1);
     dataNotReady = testStr3[0] & 0x04;
    }   
    i2c_in(bus_Id, Addr,                  // with I2C address Addr,
         CTRL_REG9, 1, testStr1, 1);      // send address CTRL_REG9 (1 byte)
                                          // data in to testStr1 (1 byte)
    while( i2c_busy(bus_Id, Addr) );      // Wait for i2c bus to become available.
    i2c_in(bus_Id, Addr,                  // with I2C address Addr,
         CTRL_REGA, 1, testStr2, 1);      // send address CTRL_REGA (1 byte)

    z = ((testStr2[0] << 24 ) + (testStr1[0] << 16) ) >> 16; // Concatenate the data
    zOffset += z;                                            // Sum the 128 samples
   }
  zOffset = (zOffset + 64) >> 7; // find average offset value.

} // End of gyroSetup

void gyroMeasure() { // function to take a gyroscope measurement
   int dataNotReady;
   float elapsedTime;
   char testStr1[1] = {0};   // I2C data string 1  
   char testStr2[1] = {0};   // I2C data string 2 
   char testStr3[1] = {0};   // I2C data string 3 
   unsigned char dataout[1]; // Setup single character varible for I2C communication 
    
   testStr1[0] = '0';   // Clear the data string 1
   testStr2[0] = '0';   // Clear the data string 2
 /* Z axis data */
  while( i2c_busy(bus_Id, Addr) ); // Wait for i2c bus to become available.
  dataNotReady = 1;
  while(dataNotReady != 4) {  
     i2c_in(bus_Id, Addr, STATUS, 1, testStr3, 1);
     dataNotReady = testStr3[0] & 0x04;
   }   
  elapsedTime = CNT - t;  //  Get elapsed system clock count.
  elapsedTime /= CLKFREQ; //  calculate elapsed seconds.
  
  t = CNT;         //  reset time counter
  i2c_in(bus_Id, Addr,                // with I2C address Addr,
         CTRL_REG9, 1, testStr1, 1);  // send address CTRL_REG6 (1 byte)
                                      // data in to testStr2 (1 byte)
  while( i2c_busy(bus_Id, Addr) );    // Wait for i2c bus to become available.
  i2c_in(bus_Id, Addr,                // with I2C address Addr,
         CTRL_REGA, 1, testStr2, 1);  // send address CTRL_REG5 (1 byte)

  z = ((testStr2[0] << 24 ) + (testStr1[0] << 16) ) >> 16; // Concatenate the data
  z -= zOffset;          // subtract offset value.
  degreesZ = z * elapsedTime;   // calculate the degrees travelled since last gyroscope measurement.
  zSum += degreesZ;      // calulate the total number of degrees travelled.
  zTotal = zSum / GYROSCOPE_SCALE_FACTOR;  
}

void ballRec(){
      int t, green[6], orange[6], temp[2], led;   
      //float og_ratio;
      low(LED_PULSE);
      high(VOLT_REG_ENABLE);                   // Energize the 3.3 V regulator
      pause(2);
      set_pause_dt(CLKFREQ/10000);             // Change the default time from 1 ms to 100 us.
      
      while(1){
        
    green[0]  = 0;
    orange[0] = 0;   
    led = 0;
    t = CNT; // record starting system clock
    while( (CNT - t)/(CLKFREQ/1000) < 8) {
       if(led == 0 && (CNT - t)/(CLKFREQ/1000) < 1) {led = 1; high(LED_PULSE); pause(5);} // turn LED on
       if(led == 1 && (CNT - t)/(CLKFREQ/1000) > 4) 
        { 
         led = 0;  
         low(LED_PULSE); // turn LED off
         green[1] = green[0];
         orange[1] = orange[0];
         pause(2);
        } 

       temp[0] = adcVal[0];
       temp[1] = adcVal[1];

       if(led) {
         if(green[0] < temp[0] )  green[0]  = temp[0];
         if(orange[0] < temp[1] ) orange[0] = temp[1];
        }

       else {
         if(green[1] > temp[0] )  green[1]  = temp[0];
         if(orange[1] > temp[1] ) orange[1] = temp[1];
        }

       pause(2);
      }            
     og_diff = ((orange[0] - orange[1]) - (green[0] - green[1]));
     og_sum = ((orange[0] - orange[1]) + (green[0] - green[1]));

     if(green[0] != green[1]) og_ratio = (  ((float) (orange[0] - orange[1]) ) / ((float) (green[0] - green[1])) );
     else og_ratio = 10;
     //printi("%d %d %d\n",og_diff, og_sum, og_ratio);
     /* Linear Disc. Functions determined from machine learning */
     Dwhite = -62.40841 -0.02041*og_diff +0.00666*og_sum +12.68754*og_ratio;  // Dwhite is the case where the sensor is looking over the 4 inch wall
     GrnLDF = -68.87080 -0.01166*og_diff +0.03478*og_sum +3.53027*og_ratio;  // GrnLDF is the LDF of a green ball
     LtPink = -79.75695 +0.04191*og_diff +0.02637*og_sum +2.62230*og_ratio;  // LtPink is the LDF of a light pink ball
     RedLDF = -109.95571 +0.07278*og_diff +0.01773*og_sum +4.66094*og_ratio;  // RedLDF is the LDF of a red ball
     Wwhite = -37.99848 +0.02988*og_diff +0.01719*og_sum +2.74962*og_ratio;  // Wwhite is the LDF of the 7" white wall
                                  
    if( (Dwhite > GrnLDF && Dwhite > LtPink && Dwhite > RedLDF) || (Wwhite > GrnLDF && Wwhite > LtPink && Wwhite > RedLDF) ) object = 0;
     else 
       if( GrnLDF > LtPink && GrnLDF > RedLDF ) object = 1; // green ball
       else
         if( LtPink > GrnLDF && LtPink > RedLDF ) object = 2; // pink ball
         else object = 3; // red ball
             
}  
 
} 

void appleHarvesting(){
   while(1){
        if(object==1){
          servo360_angle(BALL_KNOCKER_MOTOR, RETRACTED_ANGLE); // Move the knocker to the capture angle
        pause(4000);
        }          
        else if(object == 3){
          servo360_angle(BALL_KNOCKER_MOTOR, KNOCKING_ANGLE); // Move the knocker to the capture angle
        pause(4000);
        } 
        else if(object == 2){
          servo360_angle(BALL_KNOCKER_MOTOR, CAPTURE_ANGLE); // Move the knocker to the capture angle   
        pause(4000);
        }
      }          
}  