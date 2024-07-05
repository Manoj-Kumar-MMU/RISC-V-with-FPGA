
#include <neorv32.h>
#include <neorv32_rte.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 115200
/**@}*/

neorv32_uart_t *UART0 = NEORV32_UART0;
neorv32_uart_t *UART1 = NEORV32_UART1;

typedef union {
  struct {
	//unsigned char x;
    unsigned char joyStatus;        //byte0
    unsigned char joyLeftX;         //byte1
    unsigned char joyLeftY;         //byte2
    unsigned char joyRightX;        //byte3
    unsigned char joyRightY;        //byte4
    struct {
      unsigned char dpad : 4;       //byte5
      unsigned char btnSquare : 1;
      unsigned char btnCross : 1;
      unsigned char btnCircle : 1;
      unsigned char btnTriangle : 1;

      unsigned char btnL1 : 1;      //byte6
      unsigned char btnR1 : 1;
      unsigned char btnL2 : 1;
      unsigned char btnR2 : 1;
      unsigned char btnShare : 1;
      unsigned char btnOptions : 1;
      unsigned char btnL3 : 1;
      unsigned char btnR3 : 1;

      unsigned char chk : 3;        //byte7. First 4 bits not changing, can be used as verification code
      unsigned char btnMode : 1;    //Mode button
      unsigned char btnTouchPad : 1;
      unsigned char : 1;            //unsed
      unsigned char btnPs : 1;
      unsigned char : 1;            //unused
    } btn;
  } state;
  unsigned char rawdata[8];
} Controller;
Controller gamepad;

//-------------Function Prototype-------------------------
double PID_M1(int error_M1);
double PID_M2(int error_M2);
double PID_M3(int error_M3);
double PID_M4(int error_M4);
void debug_gamepad(void);
void process_gamepad(void);
double limitValue(double value);
int map(int x, int in_min, int in_max, int out_min, int out_max);
//-------------Function Prototype-------------------------

//---------Timeout Mechanism------------
unsigned int i = 0;
unsigned long long comm_timeout = 0;
unsigned long long timeout_duration = 0;
//---------Timeout Mechanism------------

//---------Motor Driver-----------------
int MAX_SPEED = 200;
double theta = 0, mag = 0;
int lx = 0, ly = 0, rx = 0;
double M1 = 0, M2 = 0, M3 = 0, M4 = 0;

#define ROTATION_SPEED_FACTOR 2.55
//---------Motor Driver-----------------

//---------PID-------------
double controlSignal = 0;
double Kp = 0.05, Ki = 0.00, Kd = 25.00;    //KP=0.172/KI=0.00/KD=10.00 OR 0.05/0/12
double prevTime = 0, currentTime = 0, dt = 0;
double prevError = 0, errorIntegral = 0, derivative = 0;

int M1_Error = 0, M2_Error = 0, M3_Error = 0, M4_Error = 0;
double M1_Speed = 0, M2_Speed = 0, M3_Speed = 0, M4_Speed = 0;
int M1_Coords = 0, M2_Coords = 0, M3_Coords = 0, M4_Coords = 0;
double prevError_M1 = 0, prevError_M2 = 0, prevError_M3 = 0, prevError_M4 = 0;

int numberOfDestination = 5;
int destinations[5][4] = 
{
  {0, 0, 0, 0},
  {-20000, -20000, 20000, 20000},
  {-40000, 0, 40000, 0},
  {-20000, 20000, 20000, -20000},
  {0, 0, 0, 0}
};
int currentDestinationIndex = 0;
//---------PID-------------

/* MOVE IN SQUARE PATTERN
  {0, 0, 0, 0},
  {-20000, -20000, 20000, 20000},
  {-40000, 0, 40000, 0},
  {-20000, 20000, 20000, -20000},
  {0, 0, 0, 0}
*/

//---------Controller----------
bool Speed = 0; 
int currentMode = 0;
bool btnL1Check = false;
bool btnR1Check = false;
bool btnPsCheck = false;
bool btnOptionsCheck = false;
//---------Controller----------

//-----DEBUG-----
bool debugAuto = false;
bool debugManual = false;
//-----DEBUG-----

int main() 
{
  neorv32_rte_setup();								// capture all exceptions and give debug info via UART
  neorv32_uart_setup(NEORV32_UART0, BAUD_RATE, 0);	// setup UART0 at default baud rate, no interrupts	
  neorv32_uart_setup(NEORV32_UART1, BAUD_RATE, 0);	// setup UART1 at default baud rate, no interrupts  
  neorv32_uart_rtscts_enable(NEORV32_UART1);
  
  if (neorv32_uart_available(NEORV32_UART0) == 0) 	// check if UART0 is implemented at all
  {
    neorv32_uart_puts(NEORV32_UART0, "Error! UART0 not synthesized!\n");
    return 1;
  }
 
  if (neorv32_uart_available(NEORV32_UART1) == 0) 	// check if UART1 is implemented at all
  {
    neorv32_uart_puts(NEORV32_UART0, "Error! UART1 not synthesized!\n");
    return 1;
  }

  if (neorv32_pwm_available() == 0) 			// check if PWM unit is implemented at all
  {
    if (neorv32_uart0_available()) 
    {
      neorv32_uart0_printf("ERROR: PWM module not implemented!\n");
    }
    return 1;
  }

  int num_pwm_channels = neorv32_pmw_get_num_channels();
  int j;
  for (j=0; j<num_pwm_channels; j++) {
    neorv32_pwm_set(j, 0);
  }
  
  neorv32_pwm_setup(CLK_PRSC_8);	// Configure and enable PWM => PWM Frequency = fmain(Hz) / 2^8 * clk_prescaler(2, 4, 8, 64, 1024, 2048, 4096) 
 
  // check if CFS is implemented at all
  if (neorv32_cfs_available() == 0) 
  {
    neorv32_uart0_printf("Error! No CFS synthesized!\n");
    return 1;
  }
 
  memset(&gamepad, 0, sizeof(Controller));	//Clears the struct data at start of program
  
  timeout_duration = 10000; //5209
 
  neorv32_uart_enable(NEORV32_UART0);
  neorv32_uart_disable(NEORV32_UART1);
  
  while (1) 
  {
  	neorv32_uart_enable(NEORV32_UART1);
  	
	if(neorv32_cpu_get_cycle() > comm_timeout)
	{
	   i = 0;
	   //neorv32_uart_puts(NEORV32_UART0, "Timeout occured. Resetting\n");
	}

    if (neorv32_uart_char_received(NEORV32_UART1))		
	{
		comm_timeout = neorv32_cpu_get_cycle() + timeout_duration;

      	gamepad.rawdata[i] = neorv32_uart_getc(NEORV32_UART1);	//Replace: neorv32_uart_getc() <- blocking function
      	i++;
	  
		if (i == 8) 
		{
			i = 0;
			neorv32_uart_disable(NEORV32_UART1);
			//neorv32_uart_puts(NEORV32_UART0, "Received full data packet\n");
			//debug_gamepad();
			process_gamepad();
		}
    }
  }
  return 0;
}

void process_gamepad(void)
{
	if (gamepad.state.joyStatus == 1)
	{	
		//----PID Timer------
		currentTime = neorv32_cpu_get_cycle();
		dt = (currentTime - prevTime);
		prevTime = currentTime;
		//----PID Timer------
		
		if(gamepad.state.btn.btnOptions && !btnOptionsCheck)  //Change between Max(255) or slow(100) speed
		{
			if(Speed == 0)
			{
				MAX_SPEED = 200;
				Speed = 1;
			}
			else
			{
				MAX_SPEED = 80;
				Speed = 0;
			}			
		}
		btnOptionsCheck = gamepad.state.btn.btnOptions;

		if(gamepad.state.btn.btnPs && !btnPsCheck)	//Change between Manual mode or Pre-defined mode
		{
			if(currentMode == 0)
			{
				currentMode = 1;
			}
			else
			{
				currentMode = 0;
			}
		}
		btnPsCheck = gamepad.state.btn.btnPs;
		
		if(currentMode == 0)
		{
			lx = map(gamepad.state.joyLeftX, 0, 255, -255, 255);
			ly = map(gamepad.state.joyLeftY, 0, 255, 255, -255); 
			rx = map(gamepad.state.joyRightX, 0, 255, -255, 255);
						
			theta = atan2f(ly, lx);			//Calculates the angle for each quadrant based on the joystick position
			mag = sqrt((lx*lx)+(ly*ly));	//Calculates the magnitude to increase speed factor
				
			M1 = mag * cos(theta - 3.14159/4) - rx/ROTATION_SPEED_FACTOR;
			M2 = mag * -sin(theta - 3.14159/4) + rx/ROTATION_SPEED_FACTOR;
			M3 = mag * -cos(theta - 3.14159/4) - rx/ROTATION_SPEED_FACTOR;
			M4 = mag * sin(theta - 3.14159/4) + rx/ROTATION_SPEED_FACTOR;
			
			//Speed multiplier
			M1 = M1 * 1.25;
			M2 = M2 * 1.25;
			M3 = M3 * 1.25;
			M4 = M4 * 1.25;
			
			M1 = round(M1);
			M2 = round(M2);
			M3 = round(M3);
			M4 = round(M4);
			
			M1 = limitValue(M1);
			M2 = limitValue(M2);
			M3 = limitValue(M3);
			M4 = limitValue(M4);
			
			if(M1 > 0) neorv32_gpio_pin_clr(1);	// LOW
			else neorv32_gpio_pin_set(1);		// HIGH
			
			if(M2 > 0) neorv32_gpio_pin_set(2);	// HIGH
			else neorv32_gpio_pin_clr(2);		// LOW
			
			if(M3 > 0) neorv32_gpio_pin_clr(3);	// LOW
			else neorv32_gpio_pin_set(3);		// HIGH
			
			if(M4 > 0) neorv32_gpio_pin_set(4);	// HIGH
			else neorv32_gpio_pin_clr(4);		// LOW
			
			neorv32_pwm_set(0, abs((int)(M1)));
			neorv32_pwm_set(1, abs((int)(M2)));	
			neorv32_pwm_set(2, abs((int)(M3)));
			neorv32_pwm_set(3, abs((int)(M4)));
	
			//-----DEBUG-----
			if(debugManual)
			{
				neorv32_uart_printf(NEORV32_UART0, "lx: %d\n", lx);
				neorv32_uart_printf(NEORV32_UART0, "ly: %d\n", ly);
				neorv32_uart_printf(NEORV32_UART0, "rx: %d\n\n", rx);
				
				neorv32_uart_printf(NEORV32_UART0, "M1: %d\t", (int)M1);
				neorv32_uart_printf(NEORV32_UART0, "M2: %d\t", (int)M2);
				neorv32_uart_printf(NEORV32_UART0, "M3: %d\t", (int)M3);
				neorv32_uart_printf(NEORV32_UART0, "M4: %d\t\n", (int)M4);
			}
		}
		else
		{

			if(gamepad.state.btn.btnR1 && !btnR1Check)
			{
				currentDestinationIndex++;
				
				if(currentDestinationIndex >= numberOfDestination)	//cycles back if reach final destination
				{
				  currentDestinationIndex = 0;
				}				
			}
			btnR1Check = gamepad.state.btn.btnR1;
			
			if(gamepad.state.btn.btnL1 && !btnL1Check)
			{
				currentDestinationIndex--;
				
				if(currentDestinationIndex < 0)		//cycles back if reach final destination
				{
				  currentDestinationIndex = numberOfDestination - 1;
				}			
			}
			btnL1Check = gamepad.state.btn.btnL1;
			
			//Gets the Setpoint data for 4 motors
		    M1_Coords = destinations[currentDestinationIndex][0];
			M2_Coords = destinations[currentDestinationIndex][1];
		    M3_Coords = destinations[currentDestinationIndex][2];
			M4_Coords = destinations[currentDestinationIndex][3];
			
			//NEORV32_CFS->REG[x], x = 0, 1, 2 & 3, reads the register that contains the encoder count for each motor
			// Error = Setpoint(Reference) - Actual
			M1_Error = M1_Coords - NEORV32_CFS->REG[0];
			M2_Error = M2_Coords - NEORV32_CFS->REG[1];
			M3_Error = M3_Coords - NEORV32_CFS->REG[2];
			M4_Error = M4_Coords - NEORV32_CFS->REG[3];
	
			//Returns Output signal from PID algorithm
			M1_Speed = PID_M1(M1_Error);
			M2_Speed = PID_M2(M2_Error);
			M3_Speed = PID_M3(M3_Error);
			M4_Speed = PID_M4(M4_Error);
			
			if(M1_Speed >= 0) neorv32_gpio_pin_set(1);	// HIGH
			else neorv32_gpio_pin_clr(1);				// LOW
			
			if(M2_Speed >= 0) neorv32_gpio_pin_set(2);	// HIGH
			else neorv32_gpio_pin_clr(2);				// LOW
			
			if(M3_Speed >= 0) neorv32_gpio_pin_set(3);	// HIGH
			else neorv32_gpio_pin_clr(3);				// LOW
			
			if(M4_Speed >= 0) neorv32_gpio_pin_set(4);	// HIGH
			else neorv32_gpio_pin_clr(4);				// LOW
			
			neorv32_pwm_set(0, abs((int)(M1_Speed)));	//Generate PWM for Motor 1
			neorv32_pwm_set(1, abs((int)(M2_Speed)));	//Generate PWM for Motor 2
			neorv32_pwm_set(2, abs((int)(M3_Speed)));	//Generate PWM for Motor 3
			neorv32_pwm_set(3, abs((int)(M4_Speed)));	//Generate PWM for Motor 4
			
			//-----DEBUG-----
			if(debugAuto)
			{
				neorv32_uart_printf(NEORV32_UART0, "Encoder Counter M1: %d\n", NEORV32_CFS->REG[0]);
				neorv32_uart_printf(NEORV32_UART0, "M1 Coords: %d\n", M1_Coords);
				neorv32_uart_printf(NEORV32_UART0, "M1 Error: %d\n", M1_Error);
				neorv32_uart_printf(NEORV32_UART0, "M1 Signal: %d\n", (int)M1_Speed);

				neorv32_uart_printf(NEORV32_UART0, "Encoder Counter M2: %d\n", NEORV32_CFS->REG[1]);
				neorv32_uart_printf(NEORV32_UART0, "M2 Coords: %d\n", M2_Coords);
				neorv32_uart_printf(NEORV32_UART0, "M2 Error: %d\n", M2_Error);
				neorv32_uart_printf(NEORV32_UART0, "M2 Signal: %d\n", (int)M2_Speed);

				neorv32_uart_printf(NEORV32_UART0, "Encoder Counter M3: %d\n", NEORV32_CFS->REG[2]);
				neorv32_uart_printf(NEORV32_UART0, "M3 Coords: %d\n", M3_Coords);
				neorv32_uart_printf(NEORV32_UART0, "M3 Error: %d\n", M3_Error);
				neorv32_uart_printf(NEORV32_UART0, "M3 Signal: %d\n", (int)M3_Speed);
		
				neorv32_uart_printf(NEORV32_UART0, "Encoder Counter M4: %d\n", NEORV32_CFS->REG[3]);
				neorv32_uart_printf(NEORV32_UART0, "M4 Coords: %d\n", M4_Coords);
				neorv32_uart_printf(NEORV32_UART0, "M4 Error: %d\n", M4_Error);
				neorv32_uart_printf(NEORV32_UART0, "M4 Signal: %d\n\n", (int)M4_Speed);					
			}
		}
	}
}

double PID_M1(int error_M1)
{
	derivative = (error_M1 - prevError_M1) / dt;
	errorIntegral = errorIntegral + error_M1 * dt;
	controlSignal = (Kp * error_M1) + (Ki * errorIntegral) + (Kd * derivative);

	controlSignal = round(controlSignal);
	controlSignal = limitValue(controlSignal);

	prevError_M1 = error_M1;
	
	return controlSignal;
}

double PID_M2(int error_M2)
{
	derivative = (error_M2 - prevError_M2) / dt;
	errorIntegral = errorIntegral + error_M2 * dt;
	controlSignal = (Kp * error_M2) + (Ki * errorIntegral) + (Kd * derivative);

	controlSignal = round(controlSignal);
	controlSignal = limitValue(controlSignal);

	prevError_M2 = error_M2;
	
	return controlSignal;
}

double PID_M3(int error_M3)
{
	derivative = (error_M3 - prevError_M3) / dt;
	errorIntegral = errorIntegral + error_M3 * dt;
	controlSignal = (Kp * error_M3) + (Ki * errorIntegral) + (Kd * derivative);

	controlSignal = round(controlSignal);
	controlSignal = limitValue(controlSignal);

	prevError_M3 = error_M3;
	
	return controlSignal;
}

double PID_M4(int error_M4)
{
	derivative = (error_M4 - prevError_M4) / dt;
	errorIntegral = errorIntegral + error_M4 * dt;
	controlSignal = (Kp * error_M4) + (Ki * errorIntegral) + (Kd * derivative);

	controlSignal = round(controlSignal);
	controlSignal = limitValue(controlSignal);

	prevError_M4 = error_M4;
	
	return controlSignal;
}

int map(int x, int in_min, int in_max, int out_min, int out_max) 
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double limitValue(double value)
{
   if(value >= MAX_SPEED) return value = MAX_SPEED;
   else if(value <= -MAX_SPEED) return value = -MAX_SPEED;
   else return value;
}

void debug_gamepad(void) 
{
    if (gamepad.state.joyStatus == 1) 
    {
        neorv32_uart_printf(NEORV32_UART0, "LX=%u\n", gamepad.state.joyLeftX);
        neorv32_uart_printf(NEORV32_UART0, "LY=%u\n", gamepad.state.joyLeftY);
        neorv32_uart_printf(NEORV32_UART0, "RX=%u\n", gamepad.state.joyRightX);
        neorv32_uart_printf(NEORV32_UART0, "RY=%u\n", gamepad.state.joyRightY);
        neorv32_uart_printf(NEORV32_UART0, "dpad=%u\n", gamepad.state.btn.dpad);

        if (gamepad.state.btn.btnSquare)	neorv32_uart_puts(NEORV32_UART0, "Square\n");

        if (gamepad.state.btn.btnTriangle)	neorv32_uart_puts(NEORV32_UART0, "Triangle\n");

        if (gamepad.state.btn.btnCross)	neorv32_uart_puts(NEORV32_UART0, "Cross\n");

        if (gamepad.state.btn.btnCircle)	neorv32_uart_puts(NEORV32_UART0, "Circle\n");

		if (gamepad.state.btn.btnL1)	neorv32_uart_puts(NEORV32_UART0, "L1\n");

		if (gamepad.state.btn.btnR1)	neorv32_uart_puts(NEORV32_UART0, "R1\n");

		if (gamepad.state.btn.btnL2)	neorv32_uart_puts(NEORV32_UART0, "L2\n");

		if (gamepad.state.btn.btnR2)	neorv32_uart_puts(NEORV32_UART0, "R2\n");

		if (gamepad.state.btn.btnShare)	neorv32_uart_puts(NEORV32_UART0, "Share\n");

		if (gamepad.state.btn.btnOptions)	neorv32_uart_puts(NEORV32_UART0, "Options\n");

		if (gamepad.state.btn.btnL3)	neorv32_uart_puts(NEORV32_UART0, "L3\n");

		if (gamepad.state.btn.btnR3)	neorv32_uart_puts(NEORV32_UART0, "R3\n");

		if (gamepad.state.btn.btnPs)	neorv32_uart_puts(NEORV32_UART0, "PS button\n");

		if (gamepad.state.btn.btnTouchPad)	neorv32_uart_puts(NEORV32_UART0, "TouchPad\n");
    }
}
