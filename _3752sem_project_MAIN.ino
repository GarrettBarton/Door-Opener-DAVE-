//Author(s): Garrett Barton and Nicholas Harper
//Course: EE 3752/sec:003 (Tuesday 1030-1230)
//Professor: Dr. J. Trahan
//Title: Semester Lab Project
/*Overview:
   ++This code will take the input from the RF receiver
      (while turning the indicator LED BLUE) and check 
      that input against the saved passwords.
   ++If the code matches a password then the servo will
      flip the door switch, turn the indicator LED GREEN,
      and the buzzer will sound.
   ++Else it will flash the indicator LED RED. 
   ++The wait timer will hold the door to allow entry and
      then the buzzer will sound and the servo will close 
      the door
      
 DISCLOSURE: Code for controlling hardware (LED, servo, 
             etc.) will be based on examples from other 
             courses, books, and other reference material.
             All logic and control flow is the design/work 
             of the author(s).
 Edge Case(s):
    1) Offset entry of character
            This is for when a user enters less than the 
          four characters required. The program will reset 
          the attempted entry to allow the user to enter in
          a new attempt after 2 seconds of not inputting.
          
  Interrupt(s):
    1) Hold door (software)
          There might be a time when one user opens the 
        door with the clicker, and then a second user
        attempts to have the door open (unaware that
        the first user already has the door open.  This
        interrupt allows the second user to input their
        password and have the door stay open. There is
        a byproduct of this interrupt that allows the 
        door to be held open for longer than the standard
        time.  This system is also recursive, allowing
        multiple entries (different attempts) while the 
        door is open.
    2) "Inputs" clear (used to handle edge case #1)

  The code is in the order of flow, with some auxiliary function at the end
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////// LIBRARIES  //////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  #include <Servo.h>         //functions for controlling  servo (instead of using PWM)
  Servo myServo;                  //names the servo for use of the servo library
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////// VARIABLE DEFINITIONS  ///////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  int angle_open = 15;           //angle used to open door
  int angle_close = 179;        //angle used to close door
  int wait_timeS = 1000;       //allows time for servo to move
  int wait_timeD = 10000;     //allows 1/6 minute for entry
  
  int buzzer_pin = 3;
  int buzzer_freq1 = 100;         //in hertz [31:65535]
  int buzzer_freq2 = 500;        //in hertz [31:65535]
  int buzzer_freq3 = 1000;      //in hertz [31:65535]
  int buzzer_time = 500;       //in milliseconds
  
  const int redLEDpin = 11;        //pin that RED is connected to
  const int greenLEDpin = 6;      //pin that GREEN is connected to
  const int blueLEDpin = 10;     //pin that BLUE is connected to
  
  char R,G,B;                     //set up characters used in function indicatorLED
  
  int pinA = 12;                  //sets pins for decoding inputs
  int pinB = 8;
  int pinC = 7;
  int pinD = 4;
  
  String inputs, x;               //used for concatInputs function
  
  int duration;                   //used for Sound_Alarm function
  String which;
  
  int Htest = 0;                  //variable that controls if testing is done
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////// SET-UPS  ////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
 void setup() 
{ 
  myServo.attach(9);              //attaches the servo on pin 9 to the servo object 
  Serial.begin(9600);             //baud rate == 9600 bits/sec
  
  pinMode(greenLEDpin,OUTPUT);    //sets these variables as outputs for digital pins
  pinMode(redLEDpin,OUTPUT);      //
  pinMode(blueLEDpin,OUTPUT);     //
  pinMode(2, INPUT);              //input for Hardware_test button    
  pinMode(pinA, INPUT);           //inputs from Rx
  pinMode(pinB, INPUT);           //
  pinMode(pinC, INPUT);           //
  pinMode(pinD, INPUT);           //
} 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////// PASSWORDS  //////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                          Garrett  Nick   Tyler   Patrick
  String userpasswords[] = {"AAAA   ", "DBCA", "BADC", "DCBA", "quit"};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////// MAIN FUNCTION  ///////////////////////////////   MAIN   ///////////// 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void loop()
{
  Hardware_testing();              //press button on board to test hardware
  
  Serial.print("\n///////////////////////////////////////////////");
  Serial.print("\nInput password: ");
  x = concatInputs();              //ReadInputs -> concatenates inputs -> returns entry as string (HERE FOR DISPLAY ORDER ONLY)
  Serial.print("\nInputs: ");
  Serial.print(x);
  array_checks();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////// CONCATINATE INPUTS //////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 String concatInputs()
  {
    int j = 0;                                             //initializes incriminator
//    for(j=0; j<=3; j++)                                  //iterates 4 times to get 4 character for password entry attempt
    int pwattemptL = inputs.length();
    while(pwattemptL < 4)
    {
      x = ReadInputs();                                    //gets input character by character
      inputs = String(inputs + x);                         //concatenates the current input with the previous inputs
      pwattemptL = inputs.length();
    }
    return inputs;                                         //returns the string that is the inputted 4-character password attempt
  }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////// READ INPUTS /////////////////////////////////////////////////////////  
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 String ReadInputs()
{
  String rc;
  char a,b,c,d;
  int time1 = millis();
  while(1)
  {
    a = digitalRead(pinA);                                //reads value from pin assigned to A
    if(a == HIGH)  {rc = "A"; indicatorLED('B'); break;} //if the pin assigned to A is high -> store A in rc (received character)
    
    b = digitalRead(pinB);                                //reads value from pin assigned to B
    if(b == HIGH)  {rc = "B"; indicatorLED('B'); break;} //if the pin assigned to B is high -> store B in rc (received character)
    
    c = digitalRead(pinC);                                //reads value from pin assigned to C
    if(c == HIGH)  {rc = "C"; indicatorLED('B'); break;} //if the pin assigned to C is high -> store C in rc (received character)
    
    d = digitalRead(pinD);                                //reads value from pin assigned to D
    if(d == HIGH)  {rc = "D"; indicatorLED('B'); break;} //if the pin assigned to D is high -> store D in rc (received character)
    
    Htest = digitalRead(2);                               //always allow for hardware testing
    if(Htest == 1)  {Hardware_testing();}
    
    int time2 = millis();
    if((time2-time1) > 2000)                              //if no other input for 2 seconds, reset
      {
        inputs = "";  
        indicatorLED('R');                                //turn the indicator LED RED
        delay(1000);
        time1 = millis();                                 //resets time1 for Exception 1
        indicatorLED('F');                                //turn the indicator LED OFF
      }  
  }
  delay(500);                                             //delay added to prevent multiple values from being read too quickly
  return rc;                                             //    for the user to let off the button
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
////////////////////////////////////////////////////////// ARRAY_CHECK  ////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void array_checks()
  {
    String quit = "";                                      //initializes password array terminator variable
    int j = 0;                                             //initializes array indexer
    
                                                           //ReadInputs -> concatenates inputs -> returns entry as string into x
    while(quit != "quit")                                  //loop through saved password array until hit terminator string 
      {                                                    //  (allows for easily scalable password lists)
        quit = userpasswords[j+1];                         //gets next element in saved password array(for test for next loop)
        char z = pwcheck(x, userpasswords[j]);             //checks password with whole array
        if(z == 'P')  {break;}                             //exits while loop if find matching, saved password
        j++;                                               //increments through saved password array
      }
    inputs = "";                                           //reset the string holding the inputs for the next iteration
  }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
////////////////////////////////////////////////////////// PASSWORD CHECKS  ////////////////////////////////////////////////////  
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 char pwcheck(String input, String saved)
  {
      if(input == saved)
        {
          inputs = "";
          x = "";
          Serial.print("\t\tOpen!");
          indicatorLED('G');                              //if input matches password indicate success
          OpenDoor();                                    //then open the door
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
          x = concatInputsWait();
          array_checks();
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
          CloseDoor();                                 //then close the door
          indicatorLED('F');                          //turn off the indicator LED
          return 'P';
        }
      else
        {
          Serial.print("\t\tNope!");
          indicatorLED('R');                           //if input NOT matches password indicate failure
          delay(1000);                                //wait long enough to show
          indicatorLED('F');                         //turn off the indicator LED
          return 'F';
        }
  }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////// CONCATINATE INPUTS_WAIT /////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 String concatInputsWait()                                 //if input character just before time limit then the door could stay
  {                                                        //  open for about 4(T-1) seconds, the standard being T (wait_timeD)
    int j = 0;                                             //initializes incriminator                               
    int pwattemptL = inputs.length();                      //gets length of the attempt
    while(pwattemptL < 4)                                  //iterates 4 times to get 4 character for password entry attempt
    {
      x = ReadInputsWait();                                //gets input character by character
      inputs = String(inputs + x);                         //concatenates the current input with the previous inputs
      pwattemptL = inputs.length();
            
      if(x == "")  {break;}                                //if no inputs for 2 seconds break
    }
    return inputs;                                         //returns the string that is the inputted 4-character password attempt
  }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////// READ INPUTS WAIT ////////////////////////////////////////////////////  
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 String ReadInputsWait()
{
  String rc;
  char a,b,c,d;
  int time1 = millis();
  while(1)
  {
    a = digitalRead(pinA);                                //reads value from pin assigned to A
    if(a == HIGH)  {rc = "A"; indicatorLED('B'); break;} //if the pin assigned to A is high -> store A in rc (received character)
    
    b = digitalRead(pinB);                                //reads value from pin assigned to B
    if(b == HIGH)  {rc = "B"; indicatorLED('B'); break;} //if the pin assigned to B is high -> store B in rc (received character)
    
    c = digitalRead(pinC);                                //reads value from pin assigned to C
    if(c == HIGH)  {rc = "C"; indicatorLED('B'); break;} //if the pin assigned to C is high -> store C in rc (received character)
    
    d = digitalRead(pinD);                                //reads value from pin assigned to D
    if(d == HIGH)  {rc = "D"; indicatorLED('B'); break;} //if the pin assigned to D is high -> store D in rc (received character)
    
    Htest = digitalRead(2);                               //always allow for hardware testing
    if(Htest == 1)  {Hardware_testing();}
    
    int time2 = millis();
    if((time2-time1) > wait_timeD)                        //if no other input for wait_timeD seconds, move out of function
      {
        break;
      }  
  }
  delay(500);                                             //delay added to prevent multiple values from being read too quickly
  return rc;                                             //    for the user to let off the button
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////// indicatorLED  ///////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void indicatorLED(char color)
  {  
    if(color == 'R')              //indicatorLED to RED
      {
        setColor(255,0,0);        //R...G...B
      }
    if(color == 'B')               //indicatorLED to BLUE
      {
        setColor(0,0,255);         //R...G...B
      }
     if(color == 'G')              //indicatorLED to GREEN
      {
        setColor(0,255,0);         //R...G...B
      }
     if(color == 'F')              //indicatorLED to OFF
      {
        setColor(0,0,0);           //R...G...B
      }
  }
/////////////////////////////////////////////////////////////////////////
 void setColor(int redVal, int greenVal, int blueVal)
  {
      analogWrite(redLEDpin, redVal);
      analogWrite(greenLEDpin, greenVal);
      analogWrite(blueLEDpin, blueVal);  
  }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////// OPEN DOOR  //////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
 void OpenDoor()
  {
      Sound_Alarm(1,"open");                             //int duration (number of cycles), String which ("open" or "close")
      myServo.write(angle_open);                        //turns servo to specified angle
      delay(wait_timeS);                               //waits for servo to move to angle
  }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////// CLOSE DOOR  /////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void CloseDoor()
   {
      Sound_Alarm(1,"close");                              //int duration (number of cycles), String which ("open" or "close")
      myServo.write(angle_close);                         //turns servo to specified angle
      delay(wait_timeS);                                 //waits for servo to move to angle
      myServo.write(85);                                //turns servo to center
      delay(wait_timeS);                               //waits for servo to move to angle
   }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////// SOUND_ALARM /////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void Sound_Alarm(int duration, String which)
 {
   if(which == "open")
     {
        tone(buzzer_pin, buzzer_freq1, buzzer_time);           //pin, frequency, duration in ms
        delay(750);
        tone(buzzer_pin, buzzer_freq1, buzzer_time);           //pin, frequency, duration in ms
        delay(750);
          
        for (int i = 0; i < duration; i++)                     //alarm for before opening door
          {
            tone(buzzer_pin, buzzer_freq2, buzzer_time);       //pin, frequency, duration in ms
            delay(200);
            
            tone(buzzer_pin, buzzer_freq3, buzzer_time);       //pin, frequency, duration in ms
            delay(200);
                  
            tone(buzzer_pin, buzzer_freq2, buzzer_time);       //pin, frequency, duration in ms
            delay(200);
            
            tone(buzzer_pin, buzzer_freq3, buzzer_time);       //pin, frequency, duration in ms
            delay(200);
            
            i++;
          }
     }
   if(which == "close")
     {
        for (int i = 0; i < duration; i++)                     //alarm for before closing door
          {
            tone(buzzer_pin, buzzer_freq2, buzzer_time);       //pin, frequency, duration in ms
            delay(200);
              
            tone(buzzer_pin, buzzer_freq3, buzzer_time);       //pin, frequency, duration in ms
            delay(200);
                    
            tone(buzzer_pin, buzzer_freq2, buzzer_time);       //pin, frequency, duration in ms
            delay(200);
              
            tone(buzzer_pin, buzzer_freq3, buzzer_time);       //pin, frequency, duration in ms
            delay(200);
              
            i++;
          }
     }
 }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////// Hardware_TESTING  ///////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//cycles through colors on multiLED, turns servo, and sounds buzzer at end of cycle whenever button pressed
 void Hardware_testing()
  {
    Htest = digitalRead(2);
    if(Htest == HIGH)                                    //button in pressed
    {
      indicatorLED('G');                                 //tests multicolored LED
      delay(1000);
      
      indicatorLED('R');
      delay(1000);
      
      indicatorLED('B');
      delay(1000);
      
      tone(buzzer_pin, buzzer_freq1, buzzer_time);       //pin, frequency, duration in ms
      delay(1000);
      
      myServo.write(85);                                 //turns servo to center
      delay(wait_timeS);                                 //waits for servo to move to angle
      OpenDoor();                                        //Test servo
      delay(2000);                                       //
      CloseDoor();                                       //
      myServo.write(85);                                 //turns servo to center
      delay(wait_timeS);                                 //waits for servo to move to angle
      
    for (int i = 0; i < 6; i++)                          //prototyping for alarms
    { 
      tone(buzzer_pin, buzzer_freq2, buzzer_time);       //pin, frequency, duration in ms
      delay(200);
      
      tone(buzzer_pin, buzzer_freq3, buzzer_time);       //pin, frequency, duration in ms
      delay(200);
            
      tone(buzzer_pin, buzzer_freq2, buzzer_time);       //pin, frequency, duration in ms
      delay(200);
      
      tone(buzzer_pin, buzzer_freq3, buzzer_time);       //pin, frequency, duration in ms
      delay(200);
      
      i++;
    }
      indicatorLED('F');                                 //turn off the indicator LED
    }
  }
