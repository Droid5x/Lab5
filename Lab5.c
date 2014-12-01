/* Aaron Clippinger, Lance Gerday, Bradley Jewett, Mark Blanco
   Section 2 Side A Seat 5
   October 27, 2014
   Lab 5 */


#include <stdio.h>
#include <stdlib.h>
#include <c8051_SDCC.h>
#include <i2c.h>

#define MAX_RANGE 60
#define MOTOR_PW_MIN 2030
#define MOTOR_PW_MAX 3500
#define MOTOR_PW_NEUT 2760
#define R_ADDR 0xE0
#define C_ADDR 0xC0
#define A_ADDR 0x30
#define SPEED  8


//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void Port_Init(void);
void PCA_Init(void);
void Drive_Motor(void);
void Steering_Servo(void);
void Interrupt_Init(void);
void SMB_Init(void);
void ADC_Init(void); // Initialize A to D Conversion
unsigned char read_AD_input(void);
void Check_Menu(void);
void Load_Menu(void);
void Data_Point(void);
void Read_Accelerometer(void);


//-----------------------------------------------------------------------------
// Global Variables
//----------------------------------------------------------------------------
unsigned char interrupts;
unsigned char take_heading;
unsigned int servo_PW_CENTER = 2905; // Center PW value
unsigned int servo_PW_MIN = 2385; // Minimum left PW value
unsigned int servo_PW_MAX = 3315; // Maximum right PW value
unsigned int servo_PW = 2905; // Start PW at center

float voltage; // Global voltage variable for checking battery voltage

unsigned int MOTOR_PW = 0; 	// Motor Pulsewidth to control motor speed
unsigned int c = 0; 		// Counter for printing data at regular intervals
unsigned char getTilt = 1; 	// Boolean flag to tell if safe to read accelerometer
unsigned char Data[2]; 		// Array for sending and receiving from ranger
signed int x_tilt = 0;
signed int y_tilt = 0;

float steering_gain = 2;	// Steering gain setting
float drive_gain_x = 4;    	// Drive gain for x axis tilt
float drive_gain_y = 4;    	// Drive gain for y axis tilt

__sbit __at 0xB6 SS_drive; 	// Assign P3.6 to SS (Slide Switch)
__sbit __at 0xB7 SS_steer; 	// Slide switch input pin at P3.7

//-----------------------------------------------------------------------------
// Main Function
//-----------------------------------------------------------------------------

void main(void) {
    // initialize board
    Sys_Init();
    putchar(' '); //the quotes in this line may not format correctly
    Port_Init();
    PCA_Init();
    SMB_Init();
    ADC_Init();
    Accel_Init();       // From i2c.h
    Interrupt_Init();
    printf("Starting\n\r");

    //print beginning message
    printf("Embedded Control Drive Motor Control\r\n");

    // Initialize motor in neutral and wait for 1 second
    Drive_Motor(0);
    c = 0;
    while (c < 50); //wait 1 second in neutral
    c = 0;
    printf("end wait \r\n");
    //Load_Menu();

	

    //Main Functionality
    while (1) {
        if (SS_steer) {// If the slide switch is active, set PW to center
            servo_PW = servo_PW_CENTER;
            PCA0CP0 = 0xFFFF - servo_PW; // Update comparator with new PW value
        } else if (getTilt) { // Otherwise take a new heading
            Read_Accelerometer();	// Get new readings and store globally
            Data_Point();
			Steering_Servo(); // Change PW based on current heading
            PCA0CP0 = 0xFFFF - servo_PW; // Update comparator with new PW value
        }
        // control statements
        servo_PW = servo_PW_CENTER - steering_gain * x_tilt; //(ks is the steering feedback gain)
        MOTOR_PW = MOTOR_PW_NEUT + drive_gain_y * y_tilt; //(kdy is the y-axis drive feedback gain)
        //Add correction for side-to-side tilt, forcing a forward movement to turn the car.
        MOTOR_PW += drive_gain_x * abs(x_tilt); //(kdx is the x-axis drive feedback gain)

        // Hold the motor in neutral if the slide switch is active
        if (SS_drive) Drive_Motor(0);
        else Drive_Motor(SPEED);
        if (c >= 25) {
            Data_Point();
            Load_Menu();
            c = 0;
        }
        while (SS_drive && SS_steer) {
            Drive_Motor(0);
            servo_PW = servo_PW_CENTER;
            PCA0CP0 = 0xFFFF - servo_PW;
            Check_Menu();
            c = 0;
            while (c < 5) {
            }
            c = 0;
        }
    }
}


//-----------------------------------------------------------------------------
// Helper Functions
//-----------------------------------------------------------------------------
//

//Function for reading and setting accerometer values
void Read_Accelerometer() {
	//Initialize averages to 0 (first to be summed)
	signed int x_Average = 0;
	signed int y_Average = 0;
    int x = 0;
	//Read in accelerometer status
	unsigned char a_Data[4];
	//Take 4 Readings and Average these
	for (x; x < 4; x++) {
		//While Reading is not ready to be taken
		while ((a_Data[0] & 0x03) != 0x03) {
            i2c_read_data(A_ADDR, 0x27, a_Data, 1);
            //Reset c
            c = 0;
			//wait 20ms
			while (c < 1);
		}
		//Read Acceleratometer Data
		i2c_read_data(A_ADDR, 0x28|0x80, a_Data, 4);
		//Update Average
		x_Average += ((a_Data[1] << 8) >> 4);
		y_Average += ((a_Data[3] << 8) >> 4);
	}
	//Average Results
	x_Average /= 4;
	y_Average /= 4;
	//Set global variables
	x_tilt = x_Average;
	y_tilt = y_Average;
    getTilt = 0;
}

void Data_Point() {
    //Print Serial Output for data collection
    printf_fast_f("Steering: %f Drive X: %f Drive Y: %f\n\r"
            , steering_gain, drive_gain_x, drive_gain_y);
    printf("BEGIN DATA POINT\n\r");
    printf("x_tilt: %d  y_tilt: %d  \n\r"
          ,x_tilt, y_tilt );
    printf("END DATA POINT\n\n\r");

    // Print the battery voltage (from AD conversion);
    voltage = read_AD_input();
    voltage /= 256;
    voltage *= 15.6;
    printf_fast_f("Battery voltage is: %.2f\n\r", voltage);
}

void Check_Menu() {/*
    signed char menu_input = read_keypad(); //Determine pressed button on keypad
    unsigned int keypad_input;

    if ((menu_input - '0') == 1) { //If steering gain is selected
        printf("Please enter a 5 digit gain constant 
                (of the form : xx.xxx) \n\r");
                lcd_clear();
                lcd_print("Enter a 5 digit gain\nconstant (xx.xxx)");
        while (read_keypad() != -1);
                keypad_input = kpd_input(1);
                steering_gain = keypad_input * 0.001;
                printf_fast_f("New steering gain is %f\n\r", steering_gain);
                Load_Menu();
        } else if ((menu_input - '0') == 2) { //If drive (motor) gain is selected
        printf("Please enter a 5 digit gain constant 
                (of the form : xx.xxx) \n\r");
                lcd_clear();
                lcd_print("Enter a 5 digit gain\nconstant (xx.xxx)");
        while (read_keypad() != -1);
                keypad_input = kpd_input(1);
                //drive_gain = keypad_input * 0.001;
//                printf_fast_f("New drive gain is %f\n\r", drive_gain);
                Load_Menu();
        } else if ((menu_input - '0') == 3) { //If desired heading is selected
        printf("Please choose an option: \n\r");
                //Print menu on terminal output
                printf("1: 0 degrees\n\r2: 90 degrees\n\r3: 180 degrees
                \n\r4 : 270 degrees\n\r5 : Enter a value\n\r");
                lcd_clear();
                //Print menu on lcd
                lcd_print("\n1.0 deg   2.90 deg\n3.180 deg 4.270 deg\n5.Enter a value");

        while (read_keypad() != -1);
               menu_input = read_keypad();
            while (menu_input == -1) menu_input = read_keypad();
                if ((menu_input - '0') == 1) { //For 0 degrees
                    // DO STUFF
                } else if ((menu_input - '0') == 2) { //For 90 degrees
                    // DO STUFF
                } else if ((menu_input - '0') == 3) { //For 180 degrees
                    // DO STUFF
                } else if ((menu_input - '0') == 4) { //For 270 degrees
                    // DO STUFF
                } else if ((menu_input - '0') == 5) { //For enter own value
                    printf("Please enter a 5 digit compass heading (of the form : 0xxxx) \n\r");
                            lcd_clear();
                            lcd_print("\nEnter a 5 digit\nheading (0xxxx)\n\r");

                    while (read_keypad() == -1);
                            keypad_input = kpd_input(1);
                            desired_heading = keypad_input % 3600;
                    }
        printf("New heading is %d\n\r", desired_heading);
        Load_Menu();
    }*/
}

void Load_Menu(void) {
    /*unsigned int PW_Percent;
    lcd_clear();
    lcd_print("1. Steering Gain\n");
    lcd_print("2. Drive Gain\n");
    lcd_print("3. Desired Heading\n");

    PW_Percent = (abs(servo_PW - servo_PW_CENTER)*200.0)
    / ((servo_PW_MAX - servo_PW_MIN));
    lcd_print("R:%3dH:%4dS:%2dB:%2d\n", range_val, compass_val,
    PW_Percent, (int) voltage);*/
}



//-----------------------------------------------------------------------------
// Drive_Motor
//-----------------------------------------------------------------------------
//
// Vary the pulsewidth based on the user input to change the speed 
// of the drive motor.
//

void Drive_Motor(void) {     
    MOTOR_PW = MOTOR_PW_NEUT + drive_gain * y_tilt//(drive_gain is the y - axis drive feedback gain)

            //Add correction for side-to-side tilt, forcing a forward movement to turn the car.
            MOTOR_PW += steering_gain * abs(x_tilt) //(steering_gain is the x - axis drive feedback gain)
            
            if(MOTOR_PW > MOTOR_PW_MAX)MOTOR_PW = MOTOR_PW_MAX;
            if(MOTOR_PW < MOTOR_PW_MIN)MOTOR_PW = MOTOR_PW_MIN;
            
            PCA0CP2 = 0xFFFF - MOTOR_PW; // Set High and low byte for the motor
}

//-----------------------------------------------------------------------------
// Port_Init
//-----------------------------------------------------------------------------
//
// Set up ports for input and output
//

void Port_Init() {

    XBR0 = 0x27; // configure crossbar with UART, SPI, SMBus, and CEX channels 
            // set output pin P1.2 and P1.0 for push-pull mode (CEX2 and CEX0)
            P1MDOUT |= 0x05;

            // Port 1 ADC
            P1MDIN &= ~0x80; //set P1.7 to Analog input
            P1MDOUT &= ~0x80; //set P1.7 to open drain mode
            P1 |= 0x80; //set P1.7 to high impedance

            P3MDOUT &= ~0xC0; // Set P3.6 and 3.7 to inputs
            P3 |= 0xC0; //set P3.6 and 3.7 to high impedance

}

//-----------------------------------------------------------------------------
// ADC_Init
//-----------------------------------------------------------------------------
//
// initilize analog to digitial conversion
//

void ADC_Init(void) {

    REF0CN = 0x03; // Use internal reference voltage (2.4V)
            ADC1CN = 0x80; // Enable A/D conversion
            ADC1CF &= 0xFC; // Reset last two bits to 0
            ADC1CF |= 0x01; // Gain set to 1.0
}

//-----------------------------------------------------------------------------
// read_AD_input
//-----------------------------------------------------------------------------
//
// read analog input
//

unsigned char read_AD_input(void) {
    AMX1SL = 7; // Set pin 7 as the analog input
            ADC1CN &= ~0x20; // Clear 'conversion complete' flag
            ADC1CN |= 0x10; // Initiate A/D conversion

    while ((ADC1CN & 0x20) == 0x00); // Wait for conversion to complete

        return ADC1; // Return digital conversion value
    }

//-----------------------------------------------------------------------------
// PCA_Init
//-----------------------------------------------------------------------------
//
// Set up Programmable Counter Array
//

void PCA_Init(void) {

    PCA0MD = 0x81; // enable CF interrupt, use SYSCLK/12

            PCA0CN = 0x40; // enable PCA0 counter
            // select 16bit PWM, enable positive edge capture, 
            // enable pulse width modulation(ranger)
            PCA0CPM2 = 0xC2;
            // select 16bit PWM, enable positive edge capture,
            // enable pulse width modulation(compass)
            PCA0CPM0 = 0xC2;
}

//-----------------------------------------------------------------------------
// Interrupt_Init
//-----------------------------------------------------------------------------
//
// Set up interrupts
//

void Interrupt_Init(void) {

    EIE1 |= 0x08; //Enable PCA0 Interrupt (bit 3) 
            EA = 1; //Enable global interrupts

}

//-----------------------------------------------------------------------------
// SMB_Init
//-----------------------------------------------------------------------------
//
// Configure Magnetic Compass Interface
//

void SMB_Init(void) {

    SMB0CR = 0x93; //Configure SCL frequency to 100kHz
            ENSMB = 1; // Enable SMBus
}



//-----------------------------------------------------------------------------
// PCA_ISR
//-----------------------------------------------------------------------------
//
// Interrupt Service Routine for Programmable Counter Array Overflow Interrupt
//

void PCA_ISR(void) __interrupt 9 {
    if (CF) { // If an interrupt has occured
        c++; // General purpose counter variable
        getTilt = 1;
        CF = 0; // Clear Interrupt Flag
                PCA0 = 28672; // Jump timer ahead for given period
    }
    PCA0CN &= 0xC0; // Handle other PCA interrupts
}

//-----------------------------------------------------------------------------
// Steering_Servo
//-----------------------------------------------------------------------------
//

void Steering_Servo(void) {

	//Update servo pulsewidth value
	servo_PW = servo_PW_CENTER - steering_gain*y_tilt;
    //Correct PW motor maximum/minimum
    if (servo_PW > servo_PW_MAX) { // Check if pulsewidth maximum exceeded
        servo_PW = servo_PW_MAX; // Set PW to a maximum value
    } else if (servo_PW < servo_PW_MIN) { // Check if less than pulsewidth min
        servo_PW = servo_PW_MIN; // Set SERVO_PW to a minimum value
    }
}
