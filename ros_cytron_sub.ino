  
#define LED_BOARD 13
#include <SoftwareSerial.h>
#include <Cytron_G15Shield.h>

Cytron_G15Shield g15(10, 11, 3);
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/UInt16.h>

word ERROR=0;
byte DATA[10]; 
word STATUS;
word post=0;
#define G15_4 4
#define G15_1 1
#define G15_2 2
#define G15_3 3
ros::NodeHandle nh;


double targets[4];
double positions[4];






void servo_cb(const sensor_msgs::JointState& cmd_msg)
{
//delay(25000);
  double p[4];
  for(int i=0;i<4;i++)
  {
  p[i] = radiansToDegrees(cmd_msg.position[i]);
  double a = p[i]+360.0;
  if(a>360)
  {
    a = a-360.0;
  }
  p[i]=a;
  }
  double pitch = p[0];
  double roll = p[1];
 if(roll>1)
  { 
    if(roll<359)
  {
  roll = roll +180.0;
  if(roll>360.0)
  {
    roll = roll-360;
  }
  }
  else
  {
    roll =0;
  }
  }
  else{
    roll = 0;
  }
  double yaw = p[2];
  double e_pitch = p[3];
  Serial.print("pitch = ");
  Serial.println(pitch);
  Serial.print("roll = ");
  Serial.println(roll);
  Serial.print("yaw = ");
  Serial.println(yaw);
  Serial.print("e_pitch = ");
  Serial.println(e_pitch);
 // control(pitch,roll,yaw,e_pitch);

//}
//void control(double pitch,double roll,double yaw,double e_pitch)
//{
   double pitch_angle=angletoposition(pitch);
   Serial.println(pitch);
   double roll_angle=angletoposition(roll);
   double yaw_angle=angletoposition(yaw);
   double e_pitch_angle=angletoposition(e_pitch);
   //double end_angle=angletoposition(end_);
   int pitch1,pitch2,roll1,roll2,yaw1,yaw2,e_pitch1,e_pitch2;
   pitch1=readangle4();
   Serial.println(pitch1);
   pitch2=pitch_angle;
   roll1=readangle3();
   roll2=roll_angle;
   yaw1=readangle2();
   yaw2=yaw_angle;
   e_pitch1=readangle1();
   e_pitch2=e_pitch_angle;
  // delay(2000);
  if(pitch1<=544 && pitch2<=544 && (pitch1-pitch2)>0)
  {
    if(pitch2>529)
    {
  g15.setLED(G15_4,ON);
  g15.setSpeed(G15_4,250);
  g15.rotateCW(G15_4,529);
  }
  else
  {
  g15.setLED(G15_4,ON);
  g15.setSpeed(G15_4,250);
  g15.rotateCW(G15_4,pitch2);
  }
  }
  else if(pitch1<=544 && pitch2<=544 && (pitch1-pitch2)<0)
  {
    if(pitch2>529)
    {
  g15.setLED(G15_4,ON);
  g15.setSpeed(G15_4,250);
  g15.rotateCCW(G15_4,529);
    }
  else{
   g15.setLED(G15_4,ON);
  g15.setSpeed(G15_4,250);
  g15.rotateCCW(G15_4,pitch2); 
  }
  }
  else if(pitch1<=544 && pitch2>544 && (pitch1-pitch2)<0)
  {
    if(pitch2<816)
    {
  g15.setLED(G15_4,ON);
  g15.setSpeed(G15_4,250);
  g15.rotateCW(G15_4,816); 
    }
    else
    {
   g15.setLED(G15_4,ON);
  g15.setSpeed(G15_4,250);
  g15.rotateCW(G15_4,pitch2); 
  }
  }
  else if(pitch1>544 && pitch2<=544 && (pitch1-pitch2)>0)
  {
    if(pitch2>529)
    {
  g15.setLED(G15_4,ON);
  g15.setSpeed(G15_4,250);
  g15.rotateCCW(G15_4,529);
    }
    else{
   g15.setLED(G15_4,ON);
  g15.setSpeed(G15_4,250);
  g15.rotateCCW(G15_4,pitch2);
  }
  }
   else if(pitch1>544 && pitch2>544 && (pitch1-pitch2)<0)
  {
    if(pitch2<574)
    {
      g15.setLED(G15_4,ON);
  g15.setSpeed(G15_4,250);
  g15.rotateCCW(G15_4,574);
    }
    else
    {
   g15.setLED(G15_4,ON);
  g15.setSpeed(G15_4,250);
  g15.rotateCCW(G15_4,pitch2); 
  }
  }
  else if(pitch1>544 && pitch2>544 && (pitch1-pitch2)>0)
  {
  if(pitch2<574)
  {
  g15.setLED(G15_4,ON);
  g15.setSpeed(G15_4,250);
  g15.rotateCW(G15_4,574);
  }
  else
  {
  g15.setLED(G15_4,ON);
  g15.setSpeed(G15_4,250);
  g15.rotateCW(G15_4,pitch2); 
  }
  }



   if(roll1<=544 && roll2<545 && (roll1-roll2)>0)
  {
    if(roll2>484)
    {
  g15.setLED(G15_3,ON);
  g15.setSpeed(G15_3,250);
  g15.rotateCW(G15_3,484);
    }
    else
    {
  g15.setLED(G15_3,ON);
  g15.setSpeed(G15_3,250);
  g15.rotateCW(G15_3,roll2); 
  }
  }
  else if(roll1<=544 && roll2<545 && (roll1-roll2)<0)
  {
    if(roll2>=484)
    {
  g15.setLED(G15_3,ON);
  g15.setSpeed(G15_3,250);
  g15.rotateCCW(G15_3,484);
    }
    else{
   g15.setLED(G15_3,ON);
  g15.setSpeed(G15_3,250);
  g15.rotateCCW(G15_3,roll2); 
  }
  }
  else if(roll1<=544 && roll2>545 && (roll1-roll2)<0)
  {
    if(roll2<1080 && roll1<250)
    {
  g15.setLED(G15_3,ON);
  g15.setSpeed(G15_3,250);
  g15.rotateCW(G15_3,0); 
    }
    else if(roll1>=250 && roll2>484 && roll2<1088)
    {
  g15.setLED(G15_3,ON);
  g15.setSpeed(G15_3,250);
  g15.rotateCCW(G15_3,484);
    }
    else
    {
   g15.setLED(G15_3,ON);
  g15.setSpeed(G15_3,250);
  g15.rotateCCW(G15_3,roll2); 
    }
  }
  else if(roll1>=544 && roll2<545 && (roll1-roll2)>0)
  {
    if(roll2>=484)
    {
   g15.setLED(G15_3,ON);
  g15.setSpeed(G15_3,250);
  g15.rotateCCW(G15_3,484); 
    }
    else if(roll2<10 && roll1>1070)
    {
   g15.setLED(G15_3,ON);
  g15.setSpeed(G15_3,250);
  g15.rotateCCW(G15_3,0); 
  }
  else
  {
    g15.setLED(G15_3,ON);
  g15.setSpeed(G15_3,250);
  g15.rotateCCW(G15_3,roll2); 
  }
  }
   else if(roll1>544  && roll2>545 && (roll1-roll2)<0)
  {
    if(roll2<1080)
    {
  g15.setLED(G15_3,ON);
  g15.setSpeed(G15_3,250);
  g15.rotateCCW(G15_3,0);
    }
    else{
   g15.setLED(G15_3,ON);
  g15.setSpeed(G15_3,250);
  g15.rotateCCW(G15_3,0); 
  }
  }

  if(yaw1<=544 && yaw2<545 && (yaw1-yaw2)>0)
  {
    if(yaw2>529)
    {
   g15.setLED(G15_2,ON);
  g15.setSpeed(G15_2,250);
  g15.rotateCW(G15_2,529); 
    }
    else
    {
  g15.setLED(G15_2,ON);
  g15.setSpeed(G15_2,250);
  g15.rotateCW(G15_2,yaw2); 
  }
  }
  else if(yaw1<=544 && yaw2<545 && (yaw1-yaw2)<0)
  {
    if(yaw2>529)
    {
  g15.setLED(G15_2,ON);
  g15.setSpeed(G15_2,250);
  g15.rotateCCW(G15_2,529); 
    }
    else
    {
  g15.setLED(G15_2,ON);
  g15.setSpeed(G15_2,250);
  g15.rotateCCW(G15_2,yaw2); 
  }
  }
  else if(yaw1<=544 && yaw2>545 && (yaw1-yaw2)<0)
  {
    if(yaw2<574)
    {
  g15.setLED(G15_2,ON);
  g15.setSpeed(G15_2,250);
  g15.rotateCW(G15_2,574);  
    }
    else
    {
  g15.setLED(G15_2,ON);
  g15.setSpeed(G15_2,250);
  g15.rotateCW(G15_2,yaw2); 
  }
  }
  else if(yaw1>=544 && yaw1<545 && (yaw1-yaw2)>0)
  {
    if(yaw2>529)
    {
  g15.setLED(G15_2,ON);
  g15.setSpeed(G15_2,250);
  g15.rotateCCW(G15_2,529); 
    }else
    {
  g15.setLED(G15_2,ON);
  g15.setSpeed(G15_2,250);
  g15.rotateCCW(G15_2,yaw2); 
  }
  }
   else if(yaw1>=544 && yaw2>545 && (yaw1-yaw2)<0)
  {
    if(yaw2<574)
    {
  g15.setLED(G15_2,ON);
  g15.setSpeed(G15_2,250);
  g15.rotateCCW(G15_2,574); 
    }
    else
    {
  g15.setLED(G15_2,ON);
  g15.setSpeed(G15_2,250);
  g15.rotateCCW(G15_2,yaw2); 
  }
  }
  else if(yaw1>=544 && yaw2>545 && (yaw1-yaw2)>0)
  {
    if(yaw2<574)
    {
   g15.setLED(G15_2,ON);
  g15.setSpeed(G15_2,250);
  g15.rotateCW(G15_2,574); 
    }
    else
    {
  g15.setLED(G15_2,ON);
  g15.setSpeed(G15_2,250);
  g15.rotateCW(G15_2,yaw2); 
  }
  }



  if(e_pitch1<=544 && e_pitch2<545 && (e_pitch1-e_pitch2)>0)
  {
    if(e_pitch2>272)
    {
  g15.setLED(G15_1,ON);
  g15.setSpeed(G15_1,250);
  g15.rotateCW(G15_1,272); 
    }
    else
    {
  g15.setLED(G15_1,ON);
  g15.setSpeed(G15_1,250);
  g15.rotateCW(G15_1,e_pitch2); 
  }
  }
  else if(e_pitch1<=544 && e_pitch2<545 && (e_pitch1-e_pitch2)<0)
  {
    if(e_pitch2>272)
    {
  g15.setLED(G15_1,ON);
  g15.setSpeed(G15_1,250);
  g15.rotateCCW(G15_1,272); 
    }
    else
    {
   g15.setLED(G15_1,ON);
  g15.setSpeed(G15_1,250);
  g15.rotateCCW(G15_1,e_pitch2); 
  }
  }
  else if(e_pitch1<=544 && e_pitch2>545 && (e_pitch1-e_pitch2)<0)
  {
    if(e_pitch2<755)
    {
  g15.setLED(G15_1,ON);
  g15.setSpeed(G15_1,250);
  g15.rotateCW(G15_1,755);  
    }
    else
    {
   g15.setLED(G15_1,ON);
  g15.setSpeed(G15_1,250);
  g15.rotateCW(G15_1,e_pitch2); 
  }
  }
  else if(e_pitch1>=544 && e_pitch1<545 && (e_pitch1-e_pitch2)>0)
  {
    if(e_pitch2>272)
    {
      g15.setLED(G15_1,ON);
  g15.setSpeed(G15_1,250);
  g15.rotateCCW(G15_1,272); 
    }
    else
    {
   g15.setLED(G15_1,ON);
  g15.setSpeed(G15_1,250);
  g15.rotateCCW(G15_1,e_pitch2); 
  }
  }
   else if(e_pitch1>=544 && e_pitch2>545 && (e_pitch1-e_pitch2)<0)
  {
    if(e_pitch2<755)
    {
  g15.setLED(G15_1,ON);
  g15.setSpeed(G15_1,250);
  g15.rotateCCW(G15_1,755);
    }
    else
    {
  g15.setLED(G15_1,ON);
  g15.setSpeed(G15_1,250);
  g15.rotateCCW(G15_1,e_pitch2); 
  }
  }
  else if(e_pitch1>=544 && e_pitch2>545 && (e_pitch1-e_pitch2)>0)
  {
    if(e_pitch2<755)
    {
  g15.setLED(G15_1,ON);
  g15.setSpeed(G15_1,250);
  g15.rotateCW(G15_1,755);
    }
    else{
   g15.setLED(G15_1,ON);
  g15.setSpeed(G15_1,250);
  g15.rotateCW(G15_1,e_pitch2); 
  }
  }
}
ros::Subscriber<sensor_msgs::JointState> sub("joint_states", servo_cb);
void setup()
{
  //Serial.begin(57600);
 // G15ShieldInit(19200,3,8); 
  //Serial.begin(19200);
  nh.initNode();
  nh.subscribe(sub);
  

  g15.begin(19200);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  delay(500);
  digitalWrite(LED, HIGH);

  // Make sure G15 in position mode
  g15.exitWheelMode(G15_1);
  g15.exitWheelMode(G15_2);
  g15.exitWheelMode(G15_3);
  g15.exitWheelMode(G15_4);

}





void loop()
{

  nh.spinOnce();

}

int readangle1()
{
  word error = 0;
byte data[10];
word position = 0;
    error = g15.getPos(G15_1, data); // Get G15 ID1 knob position
  if(error == 0) // No error
  {
    digitalWrite(LED, LOW);
    position = data[0];
    position = position | (data[1] << 8);
    Serial.print(position); // Print position
    Serial.print("   ");
    return position;
    //Serial.println(ConvertPosToAngle(position)); // Print angle
  
}
}
int readangle2()
{
  word error = 0;
byte data[10];
word position = 0;
    error = g15.getPos(G15_2, data); // Get G15 ID1 knob position
  if(error == 0) // No error
  {
    digitalWrite(LED, LOW);
    position = data[0];
    position = position | (data[1] << 8);
    Serial.print(position); // Print position
    Serial.print("   ");
    return position;
   // Serial.println(ConvertPosToAngle(position)); // Print angle
  }
}
int readangle3()
{
  word error = 0;
byte data[10];
word position = 0;
    error = g15.getPos(G15_3, data); // Get G15 ID1 knob position
  if(error == 0) // No error
  {
    digitalWrite(LED, LOW);
    position = data[0];
    position = position | (data[1] << 8);
    Serial.print(position); // Print position
    Serial.print("   ");
    return position;
    //Serial.println(ConvertPosToAngle(position)); // Print angle
  }
}
int readangle4()
{
  word error = 0;
byte data[10];
word position = 0;
   error = g15.getPos(G15_4, data); // Get G15 ID1 knob position
  if(error == 0) // No error
  {
    digitalWrite(LED, LOW);
    position = data[0];
    position = position | (data[1] << 8);
    Serial.print(position); // Print position
    Serial.print("   ");
    return position;
    //Serial.println(ConvertPosToAngle(position)); // Print angle
  }
}

double angletoposition(double angle)
{
  double po=((1088*angle)/360.0);
  return po;
}

double radiansToDegrees(float position_radians)
{

  //position_radians = position_radians + 1.6;

  return position_radians * 57.2958;

}
