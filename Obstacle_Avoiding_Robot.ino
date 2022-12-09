#include<NewPing.h>  //library for Ultrasonic sensor
#include<Servo.h>    //library for Servo motor

#define en1 5//Enable1 L293D Pin en1
#define in1 6 //Motor1  L293D Pin in1
#define in2 7 //Motor1  L293D Pin in2       
#define in3 9 //Motor2  L293D Pin in3
#define in4 10 //Motor2  L293D Pin in4                                              
#define en2 8 //Enable2 L293D Pin en2
// Initializing sensor pins
#define TRIGGER_PIN A1 //Sensor trigger pin A1
#define ECHO_PIN A0 //Sensor echo pin A0
#define MAX_DISTANCE 200 // Setting maximum distance as 200 cm

boolean goesForward = false;
int distance = 100; 
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);  
Servo servo_motor; //servo name

void setup(){
//assigning the pins as inputs and outputs
Serial.begin(9600);
//setting pin modes of motor1
pinMode(en1, OUTPUT); 
pinMode(in1, OUTPUT);
pinMode(in2, OUTPUT);
//setting pin modes of motor2
pinMode(in3, OUTPUT);
pinMode(in4, OUTPUT); 
pinMode(en2, OUTPUT);

// setting motor1 and motor2 RPM as 100%
//digitalWrite(en1, HIGH);
//digitalWrite(en2, HIGH);

// setting motor1 and motor2 RPM as 50%
analogWrite(en1, 128);
analogWrite(en2, 128);

  servo_motor.attach(3);          //servo pin assign to 3 rd pin of arduino
  servo_motor.write(115);         //servo angle = 115
  delay(2000);
  distance = readPing();          //ping the ultrasonic sensor
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100); 
}
void loop(){
  int distanceRight = 0;        //initializing variable
  int distanceLeft = 0;         //initializing variable
  delay(50);
   if (distance <= 20){         //if the distance is < or = to 20cm 
    moveStop(); // obstacle probably on the route forward, so stop
    delay(300);
    moveBackward();  // obstacle probably on the closest to the sensor, so backward
    delay(400);
    moveStop(); // stop the robot 
    delay(300);
    distanceRight = lookRight(); //look right to scan 
    delay(300);
    distanceLeft = lookLeft(); //look left to scan
    delay(300);
    if (distance >= distanceLeft){   // calculate in which direction the obstacle is more far then move to that direction
      turnRight(); //distance right is less than distance left ,so turn right
      moveStop(); 
    }
    else{
      turnLeft(); //distance left is less than distance right ,so turn left
      moveStop();
    }
  }
  else{
    moveForward(); // move forward if > 20 cm
  }
    distance = readPing();
}
int lookRight(){  // scan to the right side
  servo_motor.write(50);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
}

int lookLeft(){ // scan to the left side 
  servo_motor.write(170);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
  delay(100);
}

int readPing(){ //ping the ultrasonic sensor
  delay(70);
  int cm = sonar.ping_cm();
  if (cm==0){
    cm=250;
  }
  return cm;
}

void moveStop(){ // robot stop 
digitalWrite(in1, LOW); 
digitalWrite(in2, LOW); 

digitalWrite(in3, LOW);  
digitalWrite(in4, LOW); 
}

void moveForward(){ //robot moves forward
  if(!goesForward){
  goesForward=true;
digitalWrite(in1, LOW); 
digitalWrite(in2, HIGH); 

digitalWrite(in3, LOW);  
digitalWrite(in4, HIGH); 
}
}

void moveBackward(){ //robot moves backward
  goesForward=false;
digitalWrite(in1, HIGH); 
digitalWrite(in2, LOW); 

digitalWrite(in3, HIGH);  
digitalWrite(in4, LOW); 
}

void turnRight(){ //robot moves right
digitalWrite(in1, LOW); 
digitalWrite(in2, HIGH); 

digitalWrite(in3, HIGH);  
digitalWrite(in4, LOW); 

delay(800);

digitalWrite(in1, LOW); 
digitalWrite(in2, LOW); 

digitalWrite(in3, LOW);  
digitalWrite(in4, LOW);  
}

void turnLeft(){ //robot moves left
digitalWrite(in1, HIGH); 
digitalWrite(in2, LOW); 

digitalWrite(in3, LOW);  
digitalWrite(in4, HIGH);
 
delay(800);

digitalWrite(in1, LOW); 
digitalWrite(in2, LOW); 

digitalWrite(in3, LOW);  
digitalWrite(in4, LOW);  
}
