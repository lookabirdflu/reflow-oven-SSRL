#include "SPI.h"
#include "PlayingWithFusion_MAX31855_Wcorr.h"
#include "SoftwareSerial.h"
#include "LiquidCrystal_I2C.h" // Library for LCD 

LiquidCrystal_I2C lcd(0x27, 16, 4); // I2C address 0x27, 16 column and 2 rows 
int PWR_Relay = 2;
int Fan_Relay = 3;
int pinA = 6; // Connected to CLK on KY-040
int pinB = 5; // Connected to DT on KY-040
int pinC = 4; // Connected to SW on KY-040
int encoderPosCount = 0;
int pinALast;
int aVal;
bool cVal = 1;
bool cVal_old = 1;
boolean bCW;
int FanCount = 0;
int OvenCount = 0;
int DispCount = 0;
int8_t CS0_PIN = 10;
PWFusion_MAX31855_TC  thermocouple0(CS0_PIN);
unsigned long previousMillis = 0;
const long SampleInterval = 250;

float DutyCycle_percent = .25; //PWM Duty Cycle Percentage from 0-1
int DutyCycle = DutyCycle_percent*255;
double oldTemp;
double newTemp;
double x = 0.25;
double slope;
double calcSlope;
boolean peak = false;
boolean notice;
boolean start = false;
int which = 10000;
int which2 = 0;
int mode = 0;
int choose = 245;
boolean menu = true;


//changing Heat and Fan function
void changeHeatFan(int heat, boolean fan)  {
    
    analogWrite(PWR_Relay, heat);

    if (fan == false)  {
      digitalWrite(Fan_Relay, LOW);
    }
    else  {
      digitalWrite(Fan_Relay, HIGH);
    }
}

void setup()
{
  Serial.begin(9600);
  SPI.begin();                        // begin SPI
  SPI.setDataMode(SPI_MODE1);         // MAX31865 is a Mode 1 device
  pinMode(CS0_PIN, OUTPUT);  // initalize the chip select pin
  //Serial.println("Playing With Fusion: MAX31855-4CH, SEN-30002");
  Serial.println("Time\tTemperature");
  pinMode(PWR_Relay, OUTPUT);
  pinMode(Fan_Relay,OUTPUT);
  pinMode (pinA,INPUT);
  pinMode (pinB,INPUT);
  pinMode (pinC,INPUT);
  pinALast = digitalRead(pinA);
  lcd.init(); //initialize the lcd
  lcd.backlight(); //open the backlight
}



void loop()   {
      //Menu
      if (start == false) {
           //Relay Push Button Controls
           //Fancount is push button
                 cVal_old = cVal;
                 delay(1);
                 cVal = digitalRead(pinC);
                if((cVal != cVal_old) && (cVal == 1)){
                  FanCount++;
                  
                 }
                
           //Knob twistin'
                
                aVal = digitalRead(pinA);
                if (aVal != pinALast){ // Means the knob is rotating
                       // if the knob is rotating, we need to determine direction
                       // We do that by reading pin B.
                      if (digitalRead(pinB) != aVal) { // Means pin A Changed first - We're Rotating Clockwise
                          encoderPosCount +3;
                          bCW = true;
                      }
                      else {// Otherwise B changed first and we're moving CCW
                          bCW = false;
                          encoderPosCount -3;
                      }
                      
                      //CW or CCW 
                      if (bCW){
                          which ++;
                          which2 ++;
                          //Serial.println ("clockwise"); 
                          //Clockwise Rotation Case
                      }
                      else{
                          which --;
                          which2 --;
                          //Serial.println("counterclockwise");
                          //Counter Clockwise Rotation Case
                      }
                      
                }
                pinALast = aVal;   
                //which = debounce(pinA);


          //Menu print and select
          if (menu == true) {
            if (which % 3 == 0) {
                lcd.setCursor(0,2);
                lcd.print("Select:");
  
                
                lcd.setCursor(11,2);
                lcd.print("Norm 245");
                
                
            } else if (which % 3 ==1 ){
                lcd.setCursor(0,2);
                lcd.print("Select:");
  
               
                lcd.setCursor(11,2);
                lcd.print("High 260");
                
              
            } else if (which % 3 ==2 ){
                lcd.setCursor(0,2);
                lcd.print("Select:");
  
               
                lcd.setCursor(11,2);
                lcd.print("Choose  ");
                
            }
          }
          if (FanCount % 2 == 0 && menu == false) {
             start = true;
             lcd.clear();   
          }
          else if (FanCount % 2 == 1){
             if (menu == true)  {
                mode = (which % 3);
                which = 0;
                lcd.clear();
             }
             menu = false;
             if (mode ==2) {
                if (FanCount % 2 ==1)  {
                  choose = choose +which;
                  lcd.setCursor(0,2);
                  lcd.print("Select:");
                  lcd.setCursor(11,2);
                  lcd.print(choose);
                  which = 0;
                }
             }  else  {
                start = true;
                lcd.clear();
             }
          }
                
        
      }
      //Starting Oven process
      else if (start == true)  {
                // Initializing Time for Output
                //Serial.print("Time: ");
                
                unsigned long currentMillis = millis();
                float Seconds = (float) currentMillis/1000;
                int heat = 0;
                boolean fan = false;

                int finalmode = mode; 
                int y = 0;

                if (finalmode == 0) { //med
                   y = 245;
                }
                else if (finalmode == 1)  { //high
                   y = 260;
                }
                else if (finalmode ==2) { //choose 
                   y = choose;
                }
                
                
           
                if(currentMillis - previousMillis >= SampleInterval){ //Reduces the number of read datapoints so its doesn't spam the serial monitor
                        previousMillis = currentMillis;
                        Serial.print(String((int)Seconds) + " "); // prints time since program started
                        
                        static struct var_max31855 TC_CH0 = {0, 0, 0, 3, 0}; //Specifies T-Type
                        double tmp;
                        struct var_max31855 *tc_ptr;
                        // update T.C. Reading
                        tc_ptr = &TC_CH0;
                        thermocouple0.MAX31855_update(tc_ptr);        // Update MAX31855 readings   
                        //Temp Reading
                        tmp = (double)TC_CH0.ref_jcn_temp * 0.0625;  // convert fixed pt # to double
                        if((-100 > tmp) || (150 < tmp)){Serial.println("unknown fault");}
                        tmp = (double)TC_CH0.value * 0.25;           // convert fixed pt # to double
                    
                        
                        //Serial.print("Temp = ");                     // print TC temp heading
                        if(0x00 == TC_CH0.status){Serial.println((String)((int)tmp)); /*Serial.println((String)(x));*/}
                        else if(0x01 == TC_CH0.status){Serial.println("OPEN");}
                        else if(0x02 == TC_CH0.status){Serial.println("SHORT TO GND");}
                        else if(0x04 == TC_CH0.status){Serial.println("SHORT TO Vcc");}
                        else{Serial.println("unknown fault");
                        }
                        lcd.setCursor(1,1);
                        newTemp = tmp;
                        lcd.print((String)newTemp);
                       
                 }
               
                 //Serial.println("newTemp" + (String)newTemp);
                 slope = ((double)newTemp - (double)oldTemp)/0.5;
                 if (newTemp<140 && peak==false) {
                      changeHeatFan( (1.0 * 225.0), false);
                      //Serial.println("Heat Time");
        
                 }
                 else if (newTemp<170 && newTemp >= 140 && peak==false) { //needs to be longer
                      changeHeatFan( (0.5695 * 225.0), true);
                      //Serial.println("Heat Time");
                     // lcd.setCursor(1,0);
                      //lcd.print("Put in Oven");
          
                 }
                 else if (newTemp >= 170 && peak == false)  {
                  if(notice ==false){
                     //Serial.println("Reflow Time");
                     lcd.setCursor(1,0);
                     lcd.print("Keep in oven");
                     notice = true;
        
                  }
                  /*
                  //delay(500);
                  calcSlope = 1.66;
                  if (calcSlope > slope){
                        x+=0.05;
                        if (x>=1) {
                          x=1;
                        }
                        changeHeatFan( (x * 225.0), true);
                  }
                  else if (calcSlope == slope) {
                        changeHeatFan( (x * 225.0), true);
                  }
                  else {
                        x-=0.05;
                        if (x<=0) {
                          x=0;
                        }
                        changeHeatFan( (x * 225.0), true);
                  }
                  */
                  if (newTemp >=y)  {
                      peak = true;
                  }
                  
                  changeHeatFan( (1.0 * 225.0), false);
                    
              }
              else if (peak == true){
                   //Serial.println(analogRead(PWR_Relay));
                   pinMode(PWR_Relay, LOW);
                   analogWrite(PWR_Relay, 0);
                   analogWrite(Fan_Relay, HIGH);
                   digitalWrite(PWR_Relay, 0);

                   digitalWrite(Fan_Relay, HIGH);
                    delay(5000);
                   


                   if (newTemp <= 70) {
                      lcd.clear();
                      lcd.setCursor(1,0);
                      //lcd.print("Done Cooling; Careful-Hot");
                     // Serial.println("Done cooling");
                      //lcd.setCursor(1,0);
                      lcd.print("Unplug->Replug for new cycle");
                      start = false;
                   }
                 
              }
              else  {
                  Serial.println("Exception thrown");
              }
              oldTemp = newTemp;
              
                
      } 
      //Exception
      else  {
          Serial.println("Exception thrown");
      }
  
}



static int debounce(int fanCountPin) {
  static int fanCount = 0;
  static int lastState = LOW;
  static int state = LOW;
  static unsigned long lastDebounceTime = 0;
  static unsigned long debounceDelay = 50;

  int reading = digitalRead(fanCountPin);

  if (reading != lastState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != state) {
      state = reading;
      if (state == HIGH && lastState == LOW) {
        fanCount++;
      } else if (state == LOW && lastState == HIGH) {
        fanCount--;
      }
    }
  }

  lastState = reading;
  return fanCount;
}
