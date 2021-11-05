# GL-workshops  
*Link to videos: [GoogleDrive](https://drive.google.com/drive/folders/1N6_LGeXfZefZ2izT5ZKz3xPsAGWl0FHN?usp=sharing)*  
  
## **Training01**  
`Four consecutive blinking LEDs with cyclically changing delay.`  
	* used HAL_GPIO_WritePin and HAL_Delay functions.   
## **Training02**  
`Three modes for LED blinking with adjustable frequency and power down mode.`  
  
	* first mode: consecutive light up one at the time LED with cyclically changing delay and adjustable delay changing step;  
	* second mode: consecutive switch off one at the time LED with cyclically changing delay and adjustable delay changing step;  
	* third mode: randomly light up several LEDs depending on RNG with adjustable delay;  
	* UP/DOWN buttons used to control delay (or delay changing step in case of mode 1 and 2);  
	* LEFT/RIGHT buttons used to switching modes;  
	* CENTER button used to suspend/restore program executing and switching CPU to/from sleep mode.  
## **Training03**  
`PWM generator with adjustable frequency and duty cycle using hardware timer TIM4 all channes (ch1, ch2, ch3, ch4)`  
  
	* frequency range is 5-100 kHz with 5kHz step, start value is 50 kHz. For adjusting used buttons UP/DOWN;  
	* duty cycle range is 0-100% with 5% step, start value is 50%. For adjusting used buttons LEFT/RIGHT;  
	* cyclically switching PWM channels (ch1, ch2, ch3, ch4, no output) via CENTER button;  
	* PWM channels configured to be connected to PD12, PD13, PD14, PD15 port outputs;  
## **Training04**  
`Two temperature (external and internal temperature sensors) and potentiometer memesurments via ADC1; value visualisation via PWM channels of TIM4 and warning indication via TIM10`  
	* temperature mesurmet range os 0 - 100 °C for both sensors. Hysteresis is 1 °C;  
	* potentiometer range is 0 - 3000 mV (Vref). Hysteresis is 50 mV;  
	* alarm blinking of red LED implemented via TIM10 elapsing event;  
	* all tree measured values (two temperatures and voltage) is translated to individual PWM channels of TIM4 (ch1, ch2, ch4). it's duty cycle is proportional to mesured value.





 
