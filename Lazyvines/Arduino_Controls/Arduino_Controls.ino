/*  ECE496 Scanner Project: Phrase I
 *  Date: 12/31/2015 Rev. 3.1
 *  Author: S. Yang 
 *  
 *  This revision of the code is improved from Rev 2. to use Stepper motor opposed to the exisiting Servo motor.
 *    
 *  This Ardruino implementation is designed to be preloaded into the Arduino Hardware while using the main executable program.
 *  The code downloaded into the Arduino Nano chip will control the fine position of the motor. 
 *  The ROTATE_READY bit dictates which of the system executes and which one waits. This is an acknowlegment bit 
 *    communicated using serial port between the executing PC and the Arduino hardware. 
 *    
 *  This Calibration Data is specific to BED Driver and ROB10846 Stepper
 *  Calibration Data 
 *    Step Size     Steps per Revolution    Trials    Driver Heat 
 *    40            160                     3         Warm
 */

 

// GLOBAL CONSTANTS 

#define MOTOR_DIR   11
#define MOTOR_STEP  12
#define LED_STEP    2
#define OPTIC_SWT   3

#define STEP_SIZE   40
#define WARM_UP_TM  5
#define POLL_FREQ   100

#define ACK_READY   '1'
#define ACK_COMPL   '0'
#define ACK_FAIL    '1'

// Setup Code
void setup() {        
      
  // Start serial port at 9600 bits per second:
  Serial.begin(9600);

  // Configure Arduino Ports and Wirings 
  pinMode(MOTOR_DIR, OUTPUT);     
  pinMode(MOTOR_STEP, OUTPUT);
  pinMode(LED_STEP, OUTPUT);
  pinMode(OPTIC_SWT, INPUT); 

  // Initialize Ports 
  digitalWrite(MOTOR_DIR, LOW);
  digitalWrite(MOTOR_STEP, LOW);
  digitalWrite(LED_STEP, LOW);  
  
}

// Hardware Execution Code 
void loop() {
  
  // Waiting for acknowledge bit from USB
  while (Serial.available() <= 0) {
    delay (POLL_FREQ); 
  }

  // Clears the incoming serial stream
  char inbyte = Serial.read();

  // Sanity Check on USB Signal
  if (inbyte == ACK_READY){
    
    digitalWrite(LED_STEP, HIGH);  

    // Step Motor by Motor Step
    int counter = 0; 
    for (; counter < STEP_SIZE; counter++){
      digitalWrite(MOTOR_STEP, HIGH); 
      delay(5); 
      digitalWrite(MOTOR_STEP, LOW); 
      delay(5); 
    }

    digitalWrite(LED_STEP, LOW);  
    
    // Acknowledge Success
    Serial.print(ACK_COMPL);
  }
  else {
    // NACK 
    Serial.print(ACK_FAIL);
  }
  
}

