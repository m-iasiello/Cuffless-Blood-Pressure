// PTT calculation program using Sparkfun AD8232 HRM and SI1143 PPG sensor.  BME 525
// Includes:
/* SI114_Pulse_Demo.ino
 * demo code for the Modern Device SI1143-based pulse sensor
 * http://moderndevice.com/product/pulse-heartbeat-sensor-pulse-sensor-1x/
 * Paul Badger 2013 with plenty of coding help from Jean-Claude Wippler
 * Hardware setup - please read the chart below and set the appropriate options
 */
// Michael Iasiello

#include <stdio.h>
#include <stdlib.h>
#include <SI114.h> 
const int SAMPLES_TO_AVERAGE = 5;             // samples for smoothing 1 to 10 seem useful 5 is default
// increase for smoother waveform (with less resolution - slower!) 


 //#define SEND_TOTAL_TO_PROCESSING   // Use this option exclusive of other options
                                      // for sending data to HeartbeatGraph in Processing
// #define POWERLINE_SAMPLING         // samples on an integral of a power line period [eg 1/60 sec]
// #define AMBIENT_LIGHT_SAMPLING     // also samples ambient slight (slightly slower)
// #define PRINT_LED_VALS             // print LED raw values
 #define GET_PULSE_READING          // prints HB and signal size


int binOut;     // 1 or 0 depending on state of heartbeat
int BPM;
unsigned long red;        // read value from visible red LED
unsigned long IR1;        // read value from infrared LED1
unsigned long IR2;       // read value from infrared LED2
unsigned long total;     // all three LED reads added together
int signalSize;          // the heartbeat signal minus the offset
const int portForSI114 = 0;
PortI2C myBus (portForSI114);
PulsePlug pulse (myBus); 





// variables for lowpass (moving average) filter
int sensorValue;
float EMA_a_l = 0.4;      //initialization of EMA alpha
float lowpass = 0;          //initialization of EMA S

//variables for highpass (Exp moving average) filter
float EMA_a = 0.026177; //alpha for EMA eq., cuttoff = 3Hz
float EMA_s = 0;            //initialize EMA s
float highpass = 0;
float sensorVal = 0.0;

float threshold = 90;  

//variables for peak detection
float next_ecg_pt = 0;
float ECG_bpm = 0;
float prev_ecg_pt = 0;
float ECG_time_buff = 0;
unsigned long ECG_max_time;
//static unsigned long lastPeakTime;
boolean QRS_detected = false;
boolean get_ready = false;
unsigned long R_peak = 0;


const int LED_PIN =   13;      // the number of the LED pin (digital)

const int ECG_PIN =   A3;       // the number of the ECG pin (analog)
const int PWM_PIN =   1;       // the number of the PWM pin (analog)

const int GREEN = 2;
const int BLUE = 4;
const int RED = 3;

// timing variables
unsigned long foundTimeMillis = 0;        // time at which last QRS was found
unsigned long old_foundTimeMillis = 0;        // time at which QRS before last was found


// PTT and BP variables
int i =0;
int j=0;
int k =0;
float PTT_avg=0;
float PTT_sum = 0;
float lastBP = 0;
float bpm_ecg;
float BP_total =0;
float BP_avg=0;
float BP_val=0;
int counter=0;
unsigned long ECG_peak_time;
static unsigned long PPG_peak_time;
boolean check_ppg = false;
int PTT_val;
float bpm = 0;
float lastBP_val = 0;

// Variable timers
unsigned long currentTime = 0;
unsigned long lastTime = 0;
unsigned long lastOutputTime;

int incomingByte = 0;

void setup() {
  // initialize the serial communication:

    if (pulse.isPresent()) {
        Serial.print("PPG Sensor Found");
        Serial.println(portForSI114);
    }
        else{
        Serial.print("PPG Sensor not found.");
        Serial.println(portForSI114);
    }
    Serial.begin(9600);
    
    Serial.println("\n Welcome To to the Cuffless BP Monitor!");
    Serial.println("Place device on inside of wrist. Place opposite fingers on top electrode and LEDs");
    
    digitalWrite(3, HIGH);

      // set the digital pin as output:
  pinMode(LED_PIN, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(ECG_PIN, INPUT);

  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(RED, OUTPUT);


  // set LED to off
  digitalWrite(LED_PIN, LOW);

  //Setting LEDS init
  digitalWrite(GREEN, LOW);
  digitalWrite(BLUE, LOW);
  digitalWrite(RED, HIGH);

  initPulseSensor();


}
  
void loop() {
  
      currentTime = millis();

      // Tell user what is happening based on elapsed time
      if((currentTime - lastTime)>=5000){
        Serial.println(" ...Measurement in progress... ");
        lastTime = currentTime;
        
        if((currentTime - lastOutputTime)>=10000){
          Serial.println(" ...Reposition the device... "); 
        }
      } 


      // Reset microcontroller if LEDs bugged
      // Send '\n" or just enter through serial monitor input (or smartphone app)
      if (Serial.available()>0){
        incomingByte = Serial.read();
        if(incomingByte == 10){
          softReset();
        }
        Serial.println(incomingByte, DEC);
      }

      // Read values for both ECG and PPG
      // ECG value will need to be filtered
      readPulseSensor();        
      sensorValue = analogRead(A3);                

      // **ECG LOWPASS EMA**
      lowpass = (EMA_a_l*sensorValue) + ((1-EMA_a_l)*lowpass);    //run the EMA
           
      // **ECG HIGHPASS EMA**
      EMA_s = (EMA_a*lowpass) + ((1-EMA_a)*EMA_s);
      highpass = lowpass - EMA_s;
      //Serial.println(highpass);


      
      //QRS detection using thresholding
      next_ecg_pt = highpass;

      // Check to see if the next ECG point is greater than a predetermined threshold
      // If it is, mark the time & set a flag
      // If it is not, do not set flag
      if(next_ecg_pt > threshold){
        ECG_max_time = millis();   
        get_ready = true;
      }
      else if(next_ecg_pt <= threshold){
        get_ready = false;
      }

      // If the flag is set, check to see if the ECG is decreasing.
      // This signifies that the R-peak has just occured
      // If true: set another flag, flash LED yellow
      // If false: reset flag, flash red LED
      if(get_ready == true){
          if(next_ecg_pt < prev_ecg_pt){
            get_ready = false;
            foundTimeMillis = ECG_max_time;
            QRS_detected = true;

            //Set Yellow LED ON
              digitalWrite(GREEN, HIGH);
              digitalWrite(BLUE, LOW);
              digitalWrite(RED, HIGH);
          }
          else{
            get_ready = false;
            
            //Set Red LED ON
            digitalWrite(GREEN, LOW);
            digitalWrite(BLUE, LOW);
            digitalWrite(RED, HIGH);

          }
        }

      prev_ecg_pt = next_ecg_pt;

      // Check to see if the QRS detection has occured on succesive heartbeats
      // This is important for calculating HR.
      // If true: set new flags, calculate HR, store QRS time, flash LED blue
      // IF false: reset flags
      if (QRS_detected == true && (foundTimeMillis - old_foundTimeMillis)< 2000 && (foundTimeMillis - old_foundTimeMillis)> 700){
        
        QRS_detected = false;
        ECG_bpm = 60000/(foundTimeMillis - old_foundTimeMillis);
        
        ECG_peak_time = old_foundTimeMillis;
        check_ppg = true;

        //Set Blue LED ON
        digitalWrite(GREEN, LOW);
        digitalWrite(BLUE, HIGH);
        digitalWrite(RED, LOW);

        Serial.println("ECG signal acquired.");
      }
      else{
        QRS_detected = false;
        check_ppg = false;
      }
      old_foundTimeMillis = foundTimeMillis;

      
      // If the flag is set, we need to check the PTT value and see if it is in an appropriate ra
      if(check_ppg == true){
          check_ppg = false;
          PTT_val = PPG_peak_time - ECG_peak_time;

          // Set green LED ON
          digitalWrite(GREEN, LOW);
          digitalWrite(BLUE, HIGH);
          digitalWrite(RED, LOW);
          
         if(PTT_val < 600 && PTT_val>25){
              //Serial.print(PTT_val);
              //Serial.print();
              if(PTT_val>500){
                PTT_val = PTT_val/6;
              }
              else if(PTT_val>400){
                PTT_val = PTT_val/5;
              }
              else if(PTT_val>300){
                PTT_val = PTT_val/4;
              }
              else if(PTT_val>200){
                  PTT_val = PTT_val/3;
              }
              else if(PTT_val<50){
                PTT_val=PTT_val*4;
              }
              else if(PTT_val<20){
                PTT_val=PTT_val*5;
              }
              
              
              
              //Serial.print(PTT_val);


              bpm = (ECG_bpm + BPM)/2;
              
              lastBP_val = BP_val;
              BP_val = (-0.6881*PTT_val) + 210.94; 
              BP_val = BP_val*0.85;
              BP_total = BP_avg*counter + BP_val;
              counter++;
              BP_avg = BP_total/counter;
              
              Serial.print("BP measurement found (");
              Serial.print(counter);
              Serial.print("/5): ");
              Serial.println(BP_avg);
              //Serial.print("Your HR is"); 
              //Serial.print(" ");
             // Serial.print(ECG_bpm);
              //Serial.print(" beats per minute. ");

              // Start counting in between output times
              // If the time is greater than some threshold,
              // instruct the user to reposition finger
              lastOutputTime = currentTime;

             if(counter ==5){
                //Set Purple LED ON
                digitalWrite(GREEN, HIGH);
                digitalWrite(BLUE, LOW);
                digitalWrite(RED, LOW);

                //leave green light on for 1 second
                delay(1000);
                

                Serial.println("BP measurement found (5/5).");

                Serial.println("Your Systolic BP is estimated to be ");
                Serial.print(BP_avg);
                Serial.print(" mmHg. ");
                Serial.println("Continue to measure again. ");

                
                counter = 0;
                BP_avg = 0;
                BP_total - 0;
              }        
          }
          else{
              check_ppg = false;
              QRS_detected = false;
              get_ready = false;     
          }
      }
      
}
      

// simple smoothing function for  heartbeat detection and processing
float smooth(float data, float filterVal, float smoothedVal){

    if (filterVal > 1){      // check to make sure param's are within range
        filterVal = .99;
    }
    else if (filterVal <= 0.0){
        filterVal = 0.01;
    }

    smoothedVal = (data * (1.0 - filterVal)) + (smoothedVal  *  filterVal);
    return smoothedVal;
    
}

void softReset(){
  asm volatile (" jmp 0");
}

void initPulseSensor(){
    
    pulse.setReg(PulsePlug::HW_KEY, 0x17);  
    // pulse.setReg(PulsePlug::COMMAND, PulsePlug::RESET_Cmd);

   
    pulse.setReg(PulsePlug::INT_CFG, 0x03);       // turn on interrupts
    pulse.setReg(PulsePlug::IRQ_ENABLE, 0x10);    // turn on interrupt on PS3
    pulse.setReg(PulsePlug::IRQ_MODE2, 0x01);     // interrupt on ps3 measurement
    pulse.setReg(PulsePlug::MEAS_RATE, 0x84);     // see datasheet
    pulse.setReg(PulsePlug::ALS_RATE, 0x08);      // see datasheet
    pulse.setReg(PulsePlug::PS_RATE, 0x08);       // see datasheet

    // Current setting for LEDs pulsed while taking readings
    // PS_LED21  Setting for LEDs 1 & 2. LED 2 is high nibble
    // each LED has 16 possible (0-F in hex) possible settings
    // read the 
    pulse.setReg(PulsePlug::PS_LED21, 0xFF);      // LED current for 2 (IR1 - high nibble) & LEDs 1 (red - low nibble) 
    pulse.setReg(PulsePlug::PS_LED3, 0x0F);       // LED current for LED 3 (IR2)

   

    pulse.writeParam(PulsePlug::PARAM_CH_LIST, 0x77);         // all measurements on

    // increasing PARAM_PS_ADC_GAIN will increase the LED on time and ADC window
    // you will see increase in brightness of visible LED's, ADC output, & noise 
    // datasheet warns not to go beyond 4 because chip or LEDs may be damaged
    pulse.writeParam(PulsePlug::PARAM_PS_ADC_GAIN, 0x04);


    // You can select which LEDs are energized for each reading.
    // The settings below turn on only the LED that "normally" would be read
    // ie LED1 is pulsed and read first, then LED2 is pulsed and read etc.
    pulse.writeParam(PulsePlug::PARAM_PSLED12_SELECT, 0x21);  // 21 = LED 2 & LED 1 (red) resp.                                                              
    pulse.writeParam(PulsePlug::PARAM_PSLED3_SELECT, 0x04);   // 4 = LED 3 only

    // Sensors for reading the three LEDs
    // 0x03: Large IR Photodiode
    // 0x02: Visible Photodiode - cannot be read with LEDs on - just for ambient measurement
    // 0x00: Small IR Photodiode
    pulse.writeParam(PulsePlug::PARAM_PS1_ADCMUX, 0x02);      // PS1 photodiode select 
    pulse.writeParam(PulsePlug::PARAM_PS2_ADCMUX, 0x02);      // PS2 photodiode select 
    pulse.writeParam(PulsePlug::PARAM_PS3_ADCMUX, 0x02);      // PS3 photodiode select  

    pulse.writeParam(PulsePlug::PARAM_PS_ADC_COUNTER, B01110000);    // B01110000 is default                                   
    pulse.setReg(PulsePlug::COMMAND, PulsePlug::PSALS_AUTO_Cmd);     // starts an autonomous read loop
    // Serial.println(pulse.getReg(PulsePlug::CHIP_STAT), HEX);  
}

void readPulseSensor(){

    static int foundNewFinger;
    static  int valley=0, peak=0, smoothPeak, smoothValley, binOut, lastBinOut, BPM;
    static unsigned long lastTotal, lastMillis,  valleyTime = millis(), lastValleyTime = millis(), peakTime = millis(), lastPeakTime=millis(), lastBeat, beat;
    static float baseline, HFoutput, HFoutput2, shiftedOutput, LFoutput, hysterisis;
    

    
    unsigned long total=0, start;
    int i=0;
    int signalSize;
    red = 0;
    IR1 = 0;
    IR2 = 0;
    total = 0;
    start = millis();
         
    
     #ifdef POWERLINE_SAMPLING
     
     while (millis() - start < 16){   // 60 hz - or use 33 for two cycles
     // 50 hz in Europe use 20, or 40
       Serial.print("sample");
     #else     
     while (i < SAMPLES_TO_AVERAGE){      
     #endif
     
     
     #ifdef AMBIENT_LIGHT_SAMPLING   
     pulse.fetchData();
     
     #else 
     pulse.fetchLedData();
     #endif
     
     red += pulse.ps1;
     IR1 += pulse.ps2;
     IR2 += pulse.ps3;
     i++;
     }
     
    red = red / i;  // get averages
    IR1 = IR1 / i;
    IR2 = IR2 / i;
    total = red + IR1 + IR2;
    
   

//#ifdef AMBIENT_LIGHT_SAMPLING
//
//    Serial.print(pulse.resp, HEX);     // resp
//    Serial.print("\t");
//    Serial.print(pulse.als_vis);       //  ambient visible
//    Serial.print("\t");
//    Serial.print(pulse.als_ir);        //  ambient IR
//    Serial.print("\t");
//
//#endif

//
//#ifdef PRINT_LED_VALS
//
//    Serial.print(red);
//    Serial.print("\t");
//    Serial.print(IR1);
//    Serial.print("\t");
//    Serial.print(IR2);
//    Serial.print("\t");
//    Serial.println((long)total);   
//
//#endif

 #ifdef SEND_TOTAL_TO_PROCESSING
    //Serial.println(total);
 #endif

 #ifdef GET_PULSE_READING

    // except this one for Processing heartbeat monitor
    // comment out all the bottom print lines

    if (lastTotal < 20000L && total > 20000L) foundNewFinger = 1;  // found new finger!

    lastTotal = total;
     
    // if found a new finger prime filters first 20 times through the loop
    if (++foundNewFinger > 25) foundNewFinger = 25;   // prevent rollover 

    if ( foundNewFinger < 20){
        baseline = total - 200;   // take a guess at the baseline to prime smooth filter
   //Serial.println("......");     
    }
    
    else if(total > 20000L) {    // main running function
    
    
        // baseline is the moving average of the signal - the middle of the waveform
        // the idea here is to keep track of a high frequency signal, HFoutput and a 
        // low frequency signal, LFoutput
        // The HF signal is shifted downward slightly (heartbeats are negative peaks)
        // The high freq signal has some hysterisis added. When the HF signal is less than the 
        // shifted LF signal, we have found a heartbeat.
        baseline = smooth(total, 0.99, baseline);   // 
        HFoutput = smooth((total - baseline), 0.2, HFoutput);    // recycling output - filter to slow down response
        HFoutput2 = HFoutput + hysterisis;     
        LFoutput = smooth((total - baseline), 0.95, LFoutput);
        // heartbeat signal is inverted - we are looking for negative peaks
        shiftedOutput = LFoutput - (signalSize * .05);

        // We need to be able to keep track of peaks and valleys to scale the output for 
        // user convenience. Hysterisis is also scaled.
        if (HFoutput  > peak) peak = HFoutput; 
        if (peak > 1500) peak = 1500; 
        //Serial.println(shiftedOutput);
        //Serial.println("\t");
        if (millis() - lastPeakTime > 1800){  // reset peak detector slower than lowest human HB
            smoothPeak =  smooth((float)peak, 0.6, (float)smoothPeak);  // smooth peaks
            peak = 0;
            lastPeakTime = millis();
            PPG_peak_time = lastPeakTime; 
            //Serial.println("peak if");
        }

        if (HFoutput  < valley)   valley = HFoutput;
        if (valley < -1500) valley = -1500;

        if (millis() - lastValleyTime > 1800){  // reset valleys detector slower than lowest human HB
            smoothValley =  smooth((float)valley, 0.6, (float)smoothValley);  // smooth valleys
            valley = 0;
            lastValleyTime = millis();      
                        //Serial.println("valley if");     
        }

        signalSize = smoothPeak - smoothValley;  // this the size of the smoothed HF heartbeat signal
        
        // Serial.print(" T  ");
         //Serial.println(signalSize); 

        if(HFoutput2 < shiftedOutput){
            lastBinOut = binOut;
            binOut = 1;
         //   Serial.println("\t1");
            hysterisis = - constrain((signalSize / 10), 35, 120) ;   // you might want to divide by smaller number
            // if you start getting "double bumps"
        } 
        else {
         //   Serial.println("\t0");
            lastBinOut = binOut;
            binOut = 0;
            hysterisis = constrain((signalSize / 10), 35, 120);    // ditto above
            
        } 

  if (lastBinOut == 1 && binOut == 0){
      //Serial.println(binOut);
  }

        if (lastBinOut == 0 && binOut == 1){
            lastBeat = beat;
            beat = millis();

            BPM = 60000 / (beat - lastBeat);
            //Serial.print(binOut);
            //Serial.print("\t BPM ");
            //Serial.print(BPM);  
            //Serial.print("\t PPG peak found ");
            //Serial.println(lastPeakTime);
 
        }

    }
 #endif

}





