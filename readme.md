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
  
	* temperature mesurmet range is 0 - 100 °C for both sensors. Hysteresis is 1 °C;  
	* potentiometer range is 0 - 3000 mV (Vref). Hysteresis is 50 mV;  
	* alarm blinking of red LED implemented via TIM10 elapsing event;  
	* all tree measured values (two temperatures and voltage) is translated to individual PWM channels of TIM4 (ch1, ch2, ch4). it's duty cycle is proportional to mesured value.
	  
## **Training05**  
`LED on/off control via buttons and UART. Cyclical temperature info send via UART`  
  
	* UART3 configured with following settings: baud rate 115200, 8bit, no parity, 1 stop bit;  
	* receiving data via UART implemented using DMA;
	* external temperature sensor info is sending every 5 sec via UART. Temperature mesurmet range is 0 - 100 °C;  
	* buttons on GL Starter Kit toggling corresponding LEDs on Discovery board (SWT3 - Orange, SWT4 - Red, SWT1 - Blue, SWT5 - Green);  
	* buttons on keyboard of PC toggling corresponding LEDs on Discovery board via UART serial termanl ('1' - Orange, '2' - Red, '3' - Blue, '4' - Green);  
		  
## **Training06**  
  `pca9685 LED PWM control via UART`  
  
	* UART3 configured with following settings: baud rate 115200, 8bit, no parity, 1 stop bit;  
	* receiving data via UART implemented using DMA;  
	* I2C interface to communicate with pca9685 configured with following settings: clock requency 50kHz;  
	* typing command on keyboard of PC via UART serial termanl will control chosen pca9685 output pins (0-15) with custom brightness (0-100%) for each pin;  
	* command format: led <led number> <brightness>, where <led number> is 1 to 16 (pca9685 output pin +1), <brightness> - 0-100;  
	* Examples: "led 5 100" - will power up pin number 4 with 100% PWM duty cycle;  
				"led all 50"- will power up all out pins with 50% PWM duty cycle;  
	* to switch off output - enter brightness with value "0" for corresponding pin/pins;  
	* to confirm input - press Enter, to correct input - press Backspace;  
		  
## **Training07**  
  `Read, write and erase data to/from AT25DF161-SH-T flash memory chip via SPI with debug via UART3`  
	* UART3 configured with following settings: baud rate 115200, 8bit, no parity, 1 stop bit;    
	* SPI interface to communicate with flash memory configured with following settings: clock baud rate 500 kBits/s;  
	* to read data from flash memory: press <DOWN> button  
	* to erase whole flash memory: press <CENTER/OK> button  
	* to write Time Capsule to flash memory: press <CENTER/OK> button  
	* still in developing. TODO: commands via UART terminal, write data from termonal to flash memory.







 
