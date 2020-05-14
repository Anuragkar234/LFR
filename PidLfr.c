//This is a program to control a simple PID implemented Line_follower
//Now defining the adruino pins connected to motors

int L1 = 2;    //Left Motor
int L2 = 3; 

int R1 = 4;    //Right Motor
int R2 = 5;

int En1 = 9;    //Enable 1
int En2 = 10;   //Enable 2

//Declaring an array to store the values of the sensor readings(inputs)

int IR[5] = {0,0,0,0,0};     //Taking array if 5 IR(can be changed later)

//Declaring the constants for memory of PID

int LastProportional = 0;
int Integral = 0;

//Declaring a function prototype for different Operations

char SelectTurn(unsigned char FoundLeft, unsigned char FoundRight, unsigned char FoundStraight); 

int Mod(int v);

int SetMotors(int a, int b);

void Turn(char Direction);

void PID();

//Decalring and Initializing the variables Left and Right for Quantities
int Right = 0;
int Left = 0;

//void setup defination

void setup()
{
  pinMode(L1, OUTPUT);  //Left Motor
  pinMode(L2, OUTPUT);

  pinMode(R1, OUTPUT);  //Right Motor
  pinMode(R2, OUTPUT);

  pinMode(En1, OUTPUT);  //Enable pins
  pinMode(En2, OUTPUT);

  Serial.begin(9600);
}

//Completion of setup function

//void loop defination

void loop()
{
  PID();  //calling the PID function first to check the error 

  unsigned char FoundLeft = 0;
  unsigned char FoundRight = 0;
  unsigned char FoundStraight = 0;

  readline();    //Calling readline function to read  sensor values

  //Here Checking the condition for the sensors

  if(a[0] == HIGH)  //condition check for left sensor is high or not
  {
    FoundLeft = 1;
  }

  else if (a[0] == HIGH && a[1] == HIGH && a[2] == HIGH) //condition check for left sensor is high or not
  {
    FoundLeft = 1;
  }

  else if(a[4] == HIGH && a[3] == HIGH)  //condition check for right sensor is high or not
  {
    FoundRight = 1;  
  }

  if(a[2] == HIGH)  //condition check for middle sensor is high or not
  {
    FoundStraight = 1;
  }
  //Implementing the Direction and calling function for the final movement of the LFR
  unsigned char Direction;
  Direction = SelectTurn(FoundLeft, FoundRight, FoundStraight);
  Turn(Direction);
}

//completion of void loop


//Implementing the PID Function

void Pid()
{
  //Decalring the variables 

  int i;   //iterable variable
  int PowerDifference = 0;
  float P, I ,D; //Proportion, derivative and Integration constants
  unsigned int Position;
  int Derivative;
  int Proportional;

  //Calling the PID loop
  while(1)
  {
    position = readline();         //Reading the sensor Value and storing in variable position
    Serial.println("position = "); //Here we have to check the set point of the LFR
    Serial.println(position);

    //Replace value 2000 with your position by placing your robot at the dead centre and read the value

    Proportional = ((int)Position - 2000);
    Derivative = Proportional - LastProportional;
    Integral = Integral + Proportional;
    LastProportional = Proportional;

    //Randomly putting the values of the PID constants now :)...can be changed later

    P = 0.08;    //Proportion constant
    I = 0.0002;  //Integration constant
    D = 1.0;     //Derivative constant
      
    //Formula for PID to Calculate error(i.e Power Difference)

    PowerDifference = Proportional * P + Integral * I + Derivative * D;
    
    const int Max = 180; //Setting the maximum speed of motors

    if(PowerDifference > Max)
      PowerDifference = Max;
    if(PowerDifference < -Max)
      PowerDifference = (-1 * Max);
      if(PowerDifference < 0)
      {

        //Here function SetMotors will be called

        SetMotors(Max + PowerDifference, Max)

      }

      else
      {
        //Here Function SetMotors will be called

        SetMotors(Max, Max - PowerDifference);
      }

      // In above the functions can be interchanged depending upon the values we will get

      readline();

      if(a[0] == LOW && a[1] && a[2] == LOW && a[3] == LOW && a[4] == LOW)
      return ;
      else if(a[0] == HIGH || a[4] == HIGH)
      return;
        

  }
  
}

//Defination of readline function
//This will be used to get the set point of Line_follower....need to be changed when required

int readline()
{
    //Reading the values of the sensor

    a[0] = digitalRead(6);
    a[1] = digitalRead(7);
    a[2] = digitalRead(11);
    a[3] = digitalRead(12);
    a[4] = digitalRead(13);

    int v;

    //Calculating the average values of the sensors

    v = (4000 * a[0] + 3000 * a[1] + 2000 * a[2] + 1000 * a[3] + 0 * a[4]) / a[0] + a[1] + a[2] + a[3] + a[4]);

    Serial.println(v);  //Checking the set point of the LFR

    return v;

}

//Definfing function for the Turns of the LFR

char SelectTurn(unsigned char FoundLeft, unsigned char FoundRight, unsigned char FoundStraight)

{
    if(FoundLeft == 1)      //For Left Turn
    return 'L';

    if(FoundStraight == 1)  //For moving Straight
    return 'S';

    if(FoundRight == 1)     //For Right Turn
    return 'R';

    else
    return 'B';             //For Moving Backward

}

//Definfing the function for direction change and implement the Turns

void Turn(char Direction)
{
    switch(Direction)      //Checking the Direction got wih Predefined options
    {
        case 'L':           //Making the LFR move Left
        SetMotors(0,150);
        delay(350);
        break;
        
        case 'R'            //Making the LFR move Right
        SetMotors(150,0);
        delay(350);
        break;

        case 'B':           //Making the LFR move Backward
        SetMotors(150,150);
        delay(200);
        break;
        
        case 'S':           //Making the LFR move Stop
        break;

    }

}

//Defination of th SetMotors function to manipulate the motors according to movemnt desired

int SetMotors(int L, int R)
{
    Serial.println(r);
    Serial.println(l);

    if(l >= 0 && r >= 0)
    {
        analogWrite(En1, Mod(L));
        analogWrite(En2, Mod(R));

        digitalWrite(L1, HIGH);
        digitalWrite(L2, LOW);
        digitalWrite(R1, HIGH);
        digitalWrite(R2, LOW);
    }    

    else if(l <= 0 && r >= 0)
    {
      analogWrite(En1, Mod(L));
      analogWrite(En2, Mod(R));

      digitalWrite(L1, LOW);
      digitalWrite(L2, HIGH);
      digitalWrite(R1, HIGH);
      digitalWrite(R2, LOW);
    }

    else if(L >= 0 && r <= 0)
    {
      analogWrite(En1, Mod(L));
      analogWrite(En2, Mod(R));

      digitalWrite(L1, HIGH);
      digitalWrite(L2, LOW);
      digitalWrite(R1, LOW);
      digitalWrite(R2, HIGH);      
    }

    else if(L >= 0 && r <= 0)
    {
      analogWrite(En1, 0);
      analogWrite(En2, 0);

      digitalWrite(L1, LOW);
      digitalWrite(L2, LOW);
      digitalWrite(R1, LOW);
      digitalWrite(R2, LOW);          
    }
}

//Definfing Mod function for converison DAC 

int Mod(int V)
{
  if(V < 0)
  {
    return -1*V;
  }

  else if( V > 0)
  {
    return V;
  }
}
  

