/* maximum frequency seems to be at about 8 kHz */

#include <stdio.h>
#include <stdlib.h>

#define M 			5
#define N 			30
#define winSize			250 
#define HP_CONSTANT		((float) 1 / (float) M)
#define MAX_BPM  		100

// resolution of RNG
#define RAND_RES 100000000

// constants won't change. Used here to set a pin number 
const int LED_PIN =   13;      // the number of the LED pin (digital)
const int MOTOR_PIN = 12;      // the number of the MOTOR pin (digital)
const int ECG_PIN =   0;       // the number of the ECG pin (analog)
const int PWM_PIN =   1;       // the number of the PWM pin (analog)

// pins for leads off detection
const int LEADS_OFF_PLUS_PIN  = 10;      // the number of the LO+ pin (digital)
const int LEADS_OFF_MINUS_PIN = 11;      // the number of the LO- pin (digital) 

// timing variables
unsigned long previousMicros  = 0;        // will store last time LED was updated
unsigned long foundTimeMicros = 0;        // time at which last QRS was found
unsigned long old_foundTimeMicros = 0;        // time at which QRS before last was found
unsigned long currentMicros   = 0;        // current time

#define LED_ON_TIME_MICROS   200000
#define MOTOR_ON_TIME_MICROS 200000

// interval at which to take samples and iterate algorithm (microseconds)
const long PERIOD = 1000000 / winSize;

// circular buffer for BPM averaging
float bpm = 0;

#define BPM_BUFFER_SIZE 5
unsigned long bpm_buff[BPM_BUFFER_SIZE] = {0};
int bpm_buff_WR_idx = 0;
int bpm_buff_RD_idx = 0;

int tmp = 0;


void setup() {
  // set the digital pin as output:
  pinMode(LED_PIN, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(ECG_PIN, INPUT);

  // leads for electrodes off detection
  pinMode(LEADS_OFF_PLUS_PIN, INPUT); // Setup for leads off detection LO +
  pinMode(LEADS_OFF_MINUS_PIN, INPUT); // Setup for leads off detection LO -

  // set LED to off
  digitalWrite(LED_PIN, LOW);
  
  Serial.begin(115200);
}

void loop() {        
  currentMicros = micros();

  // iterate if it's time for a new data point (according to PERIOD)
  if (currentMicros - previousMicros >= PERIOD) {
    // save the last time you blinked the LED
    previousMicros = currentMicros;

    // see if it's time to turn LED off    
    if((currentMicros - foundTimeMicros) > LED_ON_TIME_MICROS){
      digitalWrite(LED_PIN, LOW);  
    }
    
    // see if it's time to turn MOTOR off
    if((currentMicros - foundTimeMicros) > MOTOR_ON_TIME_MICROS){
      digitalWrite(MOTOR_PIN, LOW); 
    }
  
    // only read in data and perform detection if leads are on
    boolean QRS_detected = false;
    
    // only read data if ECG chip has detected that leads are attached to patient
    boolean leads_are_on = (digitalRead(LEADS_OFF_PLUS_PIN) == 0) && (digitalRead(LEADS_OFF_MINUS_PIN) == 0);
    
    if(leads_are_on){           
      // read next ECG data point
      int next_ecg_pt = analogRead(ECG_PIN);
      //Serial.println(next_ecg_pt);
      //Serial.print("\t");
      // give next data point to algorithm
      QRS_detected = detect(next_ecg_pt);
    
            
      if(QRS_detected == true){
        foundTimeMicros = micros();
        Serial.println(foundTimeMicros);
        Serial.print("\t");
        // set the LED with the ledState of the variable:
        digitalWrite(LED_PIN, HIGH);

        bpm_buff[bpm_buff_WR_idx] = (60.0 / (((float) (foundTimeMicros - old_foundTimeMicros)) / 1000000.0));
        bpm_buff_WR_idx++;
      	bpm_buff_WR_idx %= BPM_BUFFER_SIZE;

        bpm += bpm_buff[bpm_buff_RD_idx];
		
	      tmp = bpm_buff_RD_idx - BPM_BUFFER_SIZE + 1;
	      if(tmp < 0) tmp += BPM_BUFFER_SIZE;
		
        bpm -= bpm_buff[tmp];
        
        bpm_buff_RD_idx++;
      	bpm_buff_RD_idx %= BPM_BUFFER_SIZE;

        old_foundTimeMicros = foundTimeMicros;

        Serial.println(bpm / ((float) BPM_BUFFER_SIZE - 1));

      }
    }    
  }
}



/* Portion pertaining to Pan-Tompkins QRS detection */


// circular buffer for input ecg signal
// we need to keep a history of M + 1 samples for HP filter
float ecg_buff[M + 1] = {0};
int ecg_buff_WR_idx = 0;
int ecg_buff_RD_idx = 0;

// circular buffer for input ecg signal
// we need to keep a history of N+1 samples for LP filter
float hp_buff[N + 1] = {0};
int hp_buff_WR_idx = 0;
int hp_buff_RD_idx = 0;

// LP filter outputs a single point for every input point
// This goes straight to adaptive filtering for eval
float next_eval_pt = 0;

// running sums for HP and LP filters, values shifted in FILO
float hp_sum = 0;
float lp_sum = 0;

// working variables for adaptive thresholding
float treshold = 0;
boolean triggered = false;
int trig_time = 0;
float win_max = 0;
int win_idx = 0;

// numebr of starting iterations, used determine when moving windows are filled
int number_iter = 0;

boolean detect(float new_ecg_pt) {
        // copy new point into circular buffer, increment index
	ecg_buff[ecg_buff_WR_idx++] = new_ecg_pt;  
	ecg_buff_WR_idx %= (M+1);
 
 
	/* High pass filtering */
	if(number_iter < M){
		// first fill buffer with enough points for HP filter
		hp_sum += ecg_buff[ecg_buff_RD_idx];
		hp_buff[hp_buff_WR_idx] = 0;
	}
	else{
		hp_sum += ecg_buff[ecg_buff_RD_idx];
		
		tmp = ecg_buff_RD_idx - M;
		if(tmp < 0) tmp += M + 1;
		
		hp_sum -= ecg_buff[tmp];
		
		float y1 = 0;
		float y2 = 0;
		
		tmp = (ecg_buff_RD_idx - ((M+1)/2));
		if(tmp < 0) tmp += M + 1;
		
		y2 = ecg_buff[tmp];
		
		y1 = HP_CONSTANT * hp_sum;
		
		hp_buff[hp_buff_WR_idx] = y2 - y1;
	}
	
	// done reading ECG buffer, increment position
	ecg_buff_RD_idx++;
	ecg_buff_RD_idx %= (M+1);
	
	// done writing to HP buffer, increment position
	hp_buff_WR_idx++;
	hp_buff_WR_idx %= (N+1);
	

	/* Low pass filtering */
	
	// shift in new sample from high pass filter
	lp_sum += hp_buff[hp_buff_RD_idx] * hp_buff[hp_buff_RD_idx];
	
	if(number_iter < N){
		// first fill buffer with enough points for LP filter
		next_eval_pt = 0;
		
	}
	else{
		// shift out oldest data point
		tmp = hp_buff_RD_idx - N;
		if(tmp < 0) tmp += (N+1);
		
		lp_sum -= hp_buff[tmp] * hp_buff[tmp];
		
		next_eval_pt = lp_sum;
	}
	
	// done reading HP buffer, increment position
	hp_buff_RD_idx++;
	hp_buff_RD_idx %= (N+1);
	

	/* Adapative thresholding beat detection */
	// set initial threshold				
	if(number_iter < winSize) {
		if(next_eval_pt > treshold) {
			treshold = next_eval_pt;
		}

                // only increment number_iter iff it is less than winSize
                // if it is bigger, then the counter serves no further purpose
                number_iter++;
	}
	
	// check if detection hold off period has passed
	if(triggered == true){
		trig_time++;
		
		if(trig_time >= 100){
			triggered = false;
			trig_time = 0;
		}
	}
	
	// find if we have a new max
	if(next_eval_pt > win_max) win_max = next_eval_pt;
	
	// find if we are above adaptive threshold
	if(next_eval_pt > treshold && !triggered) {
		triggered = true;

		return true;
	}
        // else we'll finish the function before returning FALSE,
        // to potentially change threshold
        	
	// adjust adaptive threshold using max of signal found 
	// in previous window            
	if(win_idx++ >= winSize){
		// weighting factor for determining the contribution of
		// the current peak value to the threshold adjustment
		float gamma = 0.175;
		
		// forgetting factor - 
		// rate at which we forget old observations
                // choose a random value between 0.01 and 0.1 for this, 
		float alpha = 0.01 + ( ((float) random(0, RAND_RES) / (float) (RAND_RES)) * ((0.1 - 0.01)));
		
                // compute new threshold
		treshold = alpha * gamma * win_max + (1 - alpha) * treshold;
		
		// reset current window index
		win_idx = 0;
		win_max = -10000000;
	}
			
        // return false if we didn't detect a new QRS
	return false;
  	
}
