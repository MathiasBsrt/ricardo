/***
 * This file is the main program for the walking dog robot called "Ricardo"
 */
//Import
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
//Array index for each servo
#define frontLeftKnee 0
#define frontLeftShoulder 1
#define frontRightKnee 2
#define frontRightShoulder 3
#define backLeftKnee 4
#define backLeftShoulder 5
#define backRightKnee 6
#define backRightShoulder 7
#define SERVOMIN 150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 600 // this is the 'maximum' pulse length count (out of 4096)
#define nbPos 10 //number of position from target to target pos (both include)
#define nbServo 8            // Number of servo
#define differenceBtwSteps 7 // Difference between the start of movement steps

//Joint pins Declaration
int numPins[] = {15, 14, 8, 9, 0, 1, 7, 6}; //In the define order (0 index => frontleftKnee)
//Rotations are setup as clock direction : 1, anti-clock direction : -1
int rotationDirections[8] = {1, -1, -1, 1, 1, -1, -1, 1}; //In the define order (0 index => frontleftKnee)

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

//Array with all the position to do : init pos => target pos
//Set of positions for each servo
//Use the array index define to access to the desired set of position to walk
int positionsWalk[nbServo][nbPos] = {};

//Position to stand up
int positionsStandUp[nbServo][nbPos] = {{-1, -2, -3, -4, -5, -6, -7, -8, -9, -10},
                                        {-1, -2, -3, -4, -5, -6, -7, -8, -9, -10},
                                        {-1, -2, -3, -4, -5, -6, -7, -8, -9, -10},
                                        {-1, -2, -3, -4, -5, -6, -7, -8, -9, -10},
                                        {-1, -2, -3, -4, -5, -6, -7, -8, -9, -13},
                                        {-1, -2, -3, -4, -5, -6, -7, -8, -9, -10},
                                        {-1, -2, -3, -4, -5, -6, -7, -8, -9, -13},
                                        {-1, -2, -3, -4, -5, -6, -7, -8, -9, -10}};

int stepWalk = 1;    //Incrementation value for walk(speed of the robot)
float heightGarot = 70;
float lFemur = 48;
float lTibia = 55;
float footProgress = 0;
float footRise= 0;

int angleCycle[] = {180,180,0,0,0,0,180,180};

void setup()
{
  Serial.begin(115200);
  Serial.println("16 channel Servo test!");
  pwm.begin();

  pwm.setPWMFreq(60); // Analog servos run at ~60 Hz updates

  //Init servo motor
  for (int servonum = 0; servonum < 16; servonum++)
  {
    pwm.setPWM(servonum, 160, 4096);
    delay(50);
    writeAngle(0, servonum);
    delay(40);
  }
  delay(1000);

}
/***
 * This function set the pwm of the provided servo according to the provided angle 
 */
void writeAngle(int angle, int servonum)
{
  if (rotationDirections[servonum] == -1)
  {
    angle = -1 * angle;
  }
  int duty = 0;
  duty = map(angle, -90, 90, SERVOMIN, SERVOMAX);
  pwm.setPWM(numPins[servonum], 0, duty);
}

int mode = 0; // 0 = standup, 1 = walk
int standup = 1; // equal 0 if already stand up

void loop()
{
  
  if(Serial.available()){
    char in = Serial.read();
    Serial.println(in);
    if(in=='w'){
      mode = 1;
    }
    else if(in == 's'){
      mode = 0;
    }
  }
  if(mode==1){
    standup = 1;
      Serial.println("WALK");
    walk();
  }
  else if(mode==0 && standup==1){
      Serial.println("STAND UP");
      standUp();
      standup = 0; // Set to already standup
  }
}

/****
 * This function set the robot in standup position
 */
void standUp()
{
  for (int i = 0; i < nbPos; i++)
  {
    writeAngle(positionsStandUp[frontLeftKnee][i], frontLeftKnee);
    writeAngle(positionsStandUp[backRightKnee][i], backRightKnee);
    writeAngle(positionsStandUp[frontLeftShoulder][i], frontLeftShoulder);
    writeAngle(positionsStandUp[backRightShoulder][i], backRightShoulder);
    writeAngle(positionsStandUp[frontRightKnee][i], frontRightKnee);
    writeAngle(positionsStandUp[backLeftKnee][i], backLeftKnee);
    writeAngle(positionsStandUp[frontRightShoulder][i], frontRightShoulder);
    writeAngle(positionsStandUp[backLeftShoulder][i], backLeftShoulder);
    delay(50);
  }
}

void walk()
{

  for (int i = 0; i <= 6; i = i+2) //Pour chaque pate
  {
    Serial.println(i);
    //TODO : correct this equation
    //On est bon, juste au retour des pates arrière, on tape les genoux arrière au sol, ça racle
    float x = heightGarot / 2.5 * sin(angleCycle[i] / 180. * M_PI) + lTibia / 4.0;
    float y = lTibia / 8. * cos(angleCycle[i] / 180. * M_PI);
    footProgress = x;
    footRise = y;
    //Calcul angles
    /* Give both angles according to a height and advance of the foot in relation 
    to the position of the hip according to the length of the femur and tibia.*/
    
    float heightHipFoot = heightGarot-footRise;

    float legSpan = sqrt(pow(heightHipFoot,2) + pow(footProgress,2));

    //angles du triangle Hanche - Genou - Pied
    float anglePHG = acos((-pow(lTibia,2) + pow(lFemur,2) + pow(legSpan,2)) / (2 * legSpan * lFemur));
    float angleGPH = acos((pow(lTibia,2) - pow(lFemur ,2) + pow(legSpan,2)) / (2 * legSpan * lTibia));
    float angleHGP = M_PI - anglePHG - angleGPH;

    // angle entre la verticale et le segment hanche - genou (fémur)
    //ATAN : float hipAngle = anglePHG + atan(footProgress / heightHipFoot);
    float hipAngle = anglePHG + atan(footProgress/heightHipFoot); //with atan2

    //angle entre le segment hanche - genou et le segment genou - pied (tibia)
    float kneeAngle= M_PI - angleHGP;

    /****** TRY NOT SUCESS BUT 
    //stuff for calculating th2
    float r_2 = pow(x,2) + pow(y,2);
    float l_sq = pow(lTibia,2)+ pow(lFemur,2);
    float term2 = (r_2 - l_sq)/(2*lFemur*lTibia);
    float term1 = (pow(1 - pow(term2,2),0.5))*-1;
    //calculate th2
    float th2 = atan2(term1, term2);

    //Stuff for calculating th2
    float k1 = lTibia + lFemur*cos(th2);
    float k2 = lFemur*sin(th2);
    float r  = pow(pow(k1,2) + pow(k2,2),0.5);
    float gamma = atan2(k2,k1);
    //calculate th1
    float th1 = atan2(y,x) - gamma ;

    ****/
    
    //Radius to degrees
    kneeAngle = kneeAngle *180 / 3.14 - 90;
    hipAngle = hipAngle *180 / 3.14 - 90;

    //Correction à l'arrache pour eviter la butée
    if(hipAngle<=-45.0){
      hipAngle = -45.0;
    }

    
    
    
    Serial.print(kneeAngle);
    Serial.print("-");
    Serial.println(hipAngle);
    Serial.println(" \n  \n");
    angleCycle[i] = (angleCycle[i] + 180 / 45) % 360;

    writeAngle(kneeAngle, i);
    writeAngle(hipAngle, i+1);


  }
  delay(10);
}
