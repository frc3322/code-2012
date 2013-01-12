#include "WPILib.h"
#include "DashboardDataSender.h"
#include "math.h"
#include "timer.h"
#define ABUTTON 1
#define BBUTTON 2
#define XBUTTON 3
#define YBUTTON 4
#define START 7
#define LBUMPER 5
#define RBUMPER 6
#define LSTICKP 9
#define RSTICKP 10
#define DPADVERT 7
#define DPADSIDE 6

#define BACK 8
#define ANSWER 0x2A    
class BuiltinDefaultCode : public IterativeRobot
{
	// Declare a variable to use to access the driver station object
	DriverStation *m_ds;						// driver station object
	UINT32 m_priorPacketNumber;					// keep track of the most recent packet number from the DS
	UINT8 m_dsPacketsReceivedInCurrentSecond;	// keep track of the ds packets received in the current second
	
	// Variable for the Smart Dashboard
	//SmartDashboard *dash;
	
	//variable for encoder
	double encoder_zero;
	
	// Variable for front/back switch
	int front;					// 1 is shooter front -1 is gather front
	
	//Declare robot drive
	RobotDrive *m_robotdrive;
	
	DriverStationLCD *d_st;
	
	// Declare variables for the two joysticks being used
	Joystick *m_controller;			// joystick 1 (arcade stick or right tank stick)
	Joystick *m_tech;			// joystick 2 (tank left stick)
	
	// Variable for the compressor
	Compressor *m_compressor;
	
	//variable for timer
	Timer *m_timer;
	
	// Variable for the solenoid
	Solenoid *m_solenoid1;//bridge tipper
	Solenoid *m_solenoid2;//bridge tipper
	Solenoid *m_solenoid3;//omniwheels
	Solenoid *m_solenoid4;//omniwheels
	Solenoid *m_solenoid5;//solenoid pi
	Solenoid *m_solenoid6;//solenoid pi
	Solenoid *m_solenoid7;//super shifter
	Solenoid *m_solenoid8;//super shifter
	
	//variables for bridge toggle
	int start_last;
	int bridge_status;
	
	//variables for solenoid pi toggle
	int back_last;
	int solenoid_pi_status;
	//variable for x button switch
	int xbutton_last;
	
	//variable for manual shooter increase
	bool tech5_last;
	
	//variable for manual shooter decrease
	bool tech10_last;
	
	//variable for auton mode
	int auton_mode;
	int auton_speed;
	int auton_waitTime;		//wait time should be in milliseconds
	double auton_distance;		//how far to drive backwards
	
	// robot motors
	CANJaguar *jaguar1;   ///1-4 wheels
	CANJaguar *jaguar2;
	CANJaguar *jaguar3;
	CANJaguar *jaguar4;
	CANJaguar *jaguar5;   ///shooter
	CANJaguar *jaguar6;   ///shooter   (spike for feed)
	CANJaguar *jaguar7;   /// ball gatherer
	
	// limit switches
	//DigitalInput *limit1;   //stops ball from launching 
	
	// spike
	Relay *spike1; //feed motor
	//Relay *; //Ball Gatherer
	Relay *spike3; //ball agitator

	///values to set motors
	double setValue1,setValue2; 
	
	///shooter speed (pwm)
	int SSpeed;
	
	// Variable for the encoders
	Encoder *m_encoder;
	// Variable for the Axis Camera
	// AxisCamera *m_camera;

	Image *image1;
	
	// Dashboard Data Sender
//	DashboardDataSender *m_dds;
	
	static const int NUM_JOYSTICK_BUTTONS = 16;
	bool m_controllerButtonState[(NUM_JOYSTICK_BUTTONS+1)];
	bool m_techButtonState[(NUM_JOYSTICK_BUTTONS+1)];	

	enum {							// drive mode selection
		UNINITIALIZED_DRIVE = 0,
		ARCADE_DRIVE = 1,
		TANK_DRIVE = 2
	} m_driveMode;
	
	// Local variables to count the number of periodic loops performed
	UINT32 m_autoPeriodicLoops;
	UINT32 m_disabledPeriodicLoops;
	UINT32 m_telePeriodicLoops;
	
public:
	BuiltinDefaultCode(void)	{
		printf("BuiltinDefaultCode Constructor Started\n");

		// Acquire the Driver Station object
		m_ds = DriverStation::GetInstance();
		m_priorPacketNumber = 0;
		m_dsPacketsReceivedInCurrentSecond = 0;
		
		// Initialize SmartDashboard
		//dash = SmartDashboard::GetInstance();
		
		// Initialize Driver Station
		d_st = DriverStationLCD::GetInstance();

		// Define joysticks being used at USB port #1 and USB port #2 on the Drivers Station
		m_controller = new Joystick(1);
		m_tech = new Joystick(2);
		
		//define encoder_zero
		encoder_zero = 0;
		
		//define bridge_status
		bridge_status = 1;
		
		//define solenoid pi status
		solenoid_pi_status = 1;  // 1 = up -1 = down 				
		// Define compressor
		m_compressor = new Compressor(1, 3);
		
		//Define variable front
		front = 1;

		// Define solenoid
		m_solenoid1 = new Solenoid(1);//bridge tipper
		m_solenoid2 = new Solenoid(2);//bridge tipper
		m_solenoid3 = new Solenoid(3);//omniwheels
		m_solenoid4 = new Solenoid(4);//omniwheels
		m_solenoid5 = new Solenoid(5);//solenoid pi
		m_solenoid6 = new Solenoid(6);//solenoid pi
		m_solenoid7 = new Solenoid(7);//super shifter
		m_solenoid8 = new Solenoid(8);//super shifter
		
		//xbutton_last initial value
		xbutton_last = 0;
		
		//tech5_last initial value
		tech5_last = 0;
		
		//tech10_last initial value
		tech10_last = 0;
		
		//SSpeed initial value
		SSpeed = 2300;
		
		// Define motor
		jaguar1 = new CANJaguar(5, CANJaguar::kPercentVbus); //1-4 wheels     
		jaguar2 = new CANJaguar(2, CANJaguar::kPercentVbus);
		jaguar3 = new CANJaguar(6, CANJaguar::kPercentVbus);
		jaguar4 = new CANJaguar(3, CANJaguar::kPercentVbus);
		jaguar5 = new CANJaguar(7, CANJaguar::kVoltage); //shooter
		jaguar6 = new CANJaguar(4, CANJaguar::kSpeed); //shooter    ///has encoder
		jaguar7 = new CANJaguar(9, CANJaguar::kPercentVbus);
		
		//set jags to coast or brake
		jaguar6->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);//shooter
		jaguar5->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);//shooter
		jaguar1->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);//wheel
		jaguar2->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);//wheel
		jaguar3->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);//wheel
		jaguar4->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);//wheel
		//define robotdrive
		m_robotdrive = new RobotDrive(jaguar1,jaguar2,jaguar3,jaguar4);
		//m_robotdrive->SetInvertedMotor(RobotDrive::kFrontLeftMotor,true);
		//m_robotdrive->SetInvertedMotor(RobotDrive::kFrontRightMotor,true);
		// Define spikes
		spike1 = new Relay(1, 1, Relay::kBothDirections); ///feed motor
		// = new Relay(1, 2, Relay::kBothDirections); ///Ball Gatherer
		spike3 = new Relay(1, 4, Relay::kBothDirections); ///ball agitator
		
		//define Timer
		m_timer = new Timer();
		
		// Define encoder
		m_encoder = new Encoder(5,6,true);
		m_encoder->Start();
		
		//Defualt value for auton_speed
		auton_speed = 2300;  
		auton_mode = 0;
		auton_waitTime = 0;
		auton_distance = 16.5;
		
		// Define camera
		/*m_camera = &AxisCamera::GetInstance("10.33.22.11");
		m_camera->WriteResolution(AxisCamera::kResolution_320x240);
		m_camera->WriteCompression(20);
		m_camera->WriteBrightness(0);*/

		// Define dashboard data sender
//		m_dds = new DashboardDataSender;
		
		// Iterate over all the buttons on each joystick, setting state to false for each
		UINT8 buttonNum = 1;						// start counting buttons at button 1
		for (buttonNum = 1; buttonNum <= NUM_JOYSTICK_BUTTONS; buttonNum++) {
			m_controllerButtonState[buttonNum] = false;
			m_techButtonState[buttonNum] = false;
		}
		// Set drive mode to uninitialized
		m_driveMode = UNINITIALIZED_DRIVE;

		// Initialize counters to record the number of loops completed in autonomous and teleop modes
		m_autoPeriodicLoops = 0;
		m_disabledPeriodicLoops = 0;
		m_telePeriodicLoops = 0;
	}
	
	/********************************** Init Routines *************************************/

	void RobotInit(void) {
		// Actions which would be performed once (and only once) upon initialization of the
		// robot would be put here.
		
		printf("RobotInit() completed.\n");
	}
	
	void DisabledInit(void) {
		m_disabledPeriodicLoops = 0;			// Reset the loop counter for disabled mode
		ClearSolenoidLEDsKITT();
		//dash->init();
		// Move the cursor down a few, since we'll move it back up in periodic.
		printf("\x1b[2B");
		m_compressor->Start();
		
		spike3->Set(Relay::kOff);   //sets ball agitator to off
		
		d_st->Clear();     //clears driver station
	}

	void AutonomousInit(void) {
		m_autoPeriodicLoops = 0;				// Reset the loop counter for autonomous mode
		ClearSolenoidLEDsKITT();
		jaguar1->EnableControl();
		jaguar2->EnableControl();
		jaguar3->EnableControl();
		jaguar4->EnableControl();
		jaguar5->EnableControl();
		jaguar6->EnableControl();
		
		///encoders for drive and shooter
		
		//must find jaguar numbers for encoders for drive.
		
		jaguar2->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
		jaguar2->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
		jaguar2->ConfigEncoderCodesPerRev(360);
		
		jaguar4->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
		jaguar4->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
		jaguar4->ConfigEncoderCodesPerRev(360);
		
		jaguar6->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
		jaguar6->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
		jaguar6->ConfigEncoderCodesPerRev(360);
		jaguar6->SetPID(0.1, 0.003, 0);
		//encoders for drive
		jaguar1->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
		jaguar1->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
		jaguar1->ConfigEncoderCodesPerRev(82);
		
		m_compressor->Start();
		
		d_st->Clear();
		encoder_zero = jaguar1->GetPosition();
	}

	void TeleopInit(void) {
		m_telePeriodicLoops = 0;				// Reset the loop counter for teleop mode
		m_dsPacketsReceivedInCurrentSecond = 0;	// Reset the number of dsPackets in current second
		m_driveMode = UNINITIALIZED_DRIVE;		// Set drive mode to uninitialized
		ClearSolenoidLEDsKITT();
		jaguar1->EnableControl(0.0);
		jaguar1->EnableControl(0.0);
		jaguar2->EnableControl();
		jaguar3->EnableControl();
		jaguar4->EnableControl();
		jaguar5->EnableControl();
		jaguar6->EnableControl();
		///encoders for shooter
		jaguar6->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
		jaguar6->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
		jaguar6->ConfigEncoderCodesPerRev(360);
		jaguar6->SetPID(0.1, 0.003, 0);
		//encoders for drive
		jaguar1->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
		jaguar1->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
		jaguar1->ConfigEncoderCodesPerRev(82);

		
		m_compressor->Start();
		 
		m_robotdrive->SetExpiration(0.1);
		m_robotdrive->SetSafetyEnabled(true);
		
		d_st->Clear();
	}

	/********************************** Periodic Routines *************************************/
	
	void DisabledPeriodic(void)  {
		// increment the number of disabled periodic loops completed
		m_disabledPeriodicLoops++;

		static bool button1 = false;      		//decrement 
		static bool button2 = false;			//move speed by 100
		static bool button3 = false;			//move speed by 10
		static bool button4 = false;			//mode
		static bool button5 = false;			//wait
		static bool button6 = false;
		static bool button7 = false;
		
		
		int incrementOrDecrement; 				//1 if increment -1 if decrement
		
		if(m_tech->GetRawButton(1))
			incrementOrDecrement = -1;
		else
			incrementOrDecrement = 1;
		
		if(m_tech->GetRawButton(2) && !button2)
			auton_speed += 100*incrementOrDecrement;
		else if(m_tech->GetRawButton(3) && !button3)
			auton_speed += 10*incrementOrDecrement;
		
		if(m_tech->GetRawButton(4) && !button4)
			auton_mode += incrementOrDecrement;
		if(m_tech->GetRawButton(5) && !button5)
			auton_waitTime += incrementOrDecrement;
		
		if(m_tech->GetRawButton(6) && !button6)
			auton_distance += incrementOrDecrement;
		if(m_tech->GetRawButton(7) && !button7)
			auton_distance += 0.1*incrementOrDecrement;
		
		button1 = m_tech->GetRawButton(1);
		button2 = m_tech->GetRawButton(2);
		button3 = m_tech->GetRawButton(3);
		button4 = m_tech->GetRawButton(4);
		button5 = m_tech->GetRawButton(5);
		button6 = m_tech->GetRawButton(6);    
		button7 = m_tech->GetRawButton(7);
		
		
		
		d_st->Printf(DriverStationLCD::kUser_Line1,1,"Shooter speed = %d   ",auton_speed);
		d_st->Printf(DriverStationLCD::kUser_Line2,1,"Auton wait time = %d   ",auton_waitTime);
		d_st->Printf(DriverStationLCD::kUser_Line3,1,"Autonomous mode = %d   ",auton_mode);
		//d_st->Printf(DriverStationLCD::kUser_Line4,1,"Autonomous distance = %d   ",auton_distance);
		d_st->UpdateLCD();
	}
	void AutonomousPeriodic(void) {
		d_st->Printf(DriverStationLCD::kUser_Line4,1,"Encoder = %f   ",jaguar1->GetPosition() - encoder_zero);
		switch (auton_mode)
		{
		case 1:
			AutonomousMode1();
			break;
		case 2:
			AutonomousMode2();
			break;
		case 3:
			AutonomousMode3();
			break;
		default:
			AutonomousMode1();
		}
		
		m_autoPeriodicLoops++;
	
		//printf("encoder value: %d\n", m_encoder->GetRaw());
		/* the below code (if uncommented) would drive the robot forward at half speed
		 * for two seconds.  This code is provided as an example of how to drive the 
		 * robot in autonomous mode, but is not enabled in the default code in order
		 * to prevent an unsuspecting team from having their robot drive autonomously!
		 */
		/* below code commented out for safety*/
	}
	void TeleopPeriodic(void) {
		printf("derpitworks herp\n");
		// increment the number of teleop periodic loops completed
		m_telePeriodicLoops++;

		/*
		 * No longer needed since periodic loops are now synchronized with incoming packets.
		if (m_ds->GetPacketNumber() != m_priorPacketNumber) {
		*/
			m_dsPacketsReceivedInCurrentSecond++;					// increment DS packets received
						
						
			setValue1 = m_controller->GetY()-m_controller->GetTwist();
			setValue2 = -(m_controller->GetY()+m_controller->GetTwist());
			
			//this button will manually run the compressor
			//if (m_controller->GetRawButton(YBUTTON))
				//m_compressor->Start();  ///should use  instead
			
		
			// if the X button is pressed the direction of the front of the robot will change.
			/*if (m_controller->GetRawButton(XBUTTON)&& !xbutton_last)
			{
				front *= -1;
			}*/
			xbutton_last = m_controller->GetRawButton(XBUTTON);
			
			//ball agitator
			if(m_controller->GetRawButton(BBUTTON))
			{
				spike3->Set(Relay::kOn);
				spike3->Set(Relay::kReverse);
			}
			else if(m_controller->GetRawButton(XBUTTON))
			{
				spike3->Set(Relay::kOn);
				spike3->Set(Relay::kForward);
			}
			else
			{
				spike3->Set(Relay::kOff);
			}
			//start compressor
			m_compressor->Start();
			
			//DriveRobot(jaguar1, jaguar2, jaguar3, jaguar4, setValue1, setValue2);
			m_robotdrive->ArcadeDrive(m_controller,2,m_controller,4);
			
			//code for manual variable shooter speed		
			static bool button1 = false;      		//decrement 
			static bool button2 = false;			//move speed by 100
			static bool button3 = false;			//move speed by 10
			static bool button4 = false;			//mode
			static bool button5 = false;			//wait
			int incrementOrDecrement; 				//1 if increment -1 if decrement
			
			if(m_tech->GetRawButton(1))
				incrementOrDecrement = -1;
			else
				incrementOrDecrement = 1;
			
			if(m_tech->GetRawButton(2) && !button2)
				SSpeed += 100*incrementOrDecrement;
			else if(m_tech->GetRawButton(3) && !button3)
				SSpeed += 10*incrementOrDecrement;
			
			if(m_tech->GetRawButton(4) && !button4)
				auton_mode += incrementOrDecrement;
			if(m_tech->GetRawButton(5) && !button5)
				auton_waitTime += incrementOrDecrement;
			if(m_tech->GetRawButton(6))
			{
				SSpeed = 2300;
			}
			if(m_tech->GetRawButton(7))
			{
				SSpeed = 4000;
			}
			if(m_tech->GetRawButton(8))
			{
				encoder_zero = jaguar1->GetPosition();
			}
			
			
			
			button1 = m_tech->GetRawButton(1);
			button2 = m_tech->GetRawButton(2);
			button3 = m_tech->GetRawButton(3);
			button4 = m_tech->GetRawButton(4);
			button5 = m_tech->GetRawButton(5);
			
			///bridge tipper
			if (m_controller->GetRawButton(START) && !start_last)
			{
				bridge_status *= -1;
			}
			if(bridge_status == -1)
			{
				m_solenoid1->Set(true);
				m_solenoid2->Set(false);
			}
			else if(bridge_status == 1)
			{
				m_solenoid1->Set(false);
				m_solenoid2->Set(true);
			}
			start_last = m_controller->GetRawButton(START);
			
			///omniwheels
			if (m_controller->GetRawButton(LSTICKP))
			{
				m_solenoid3->Set(true);
				m_solenoid4->Set(false);
			}
			else
			{
				m_solenoid3->Set(false);
				m_solenoid4->Set(true);
			}
			///solenoid pi
			if(m_controller->GetRawButton(BACK)&& !back_last)
			{
				solenoid_pi_status *= -1;
			}
 			if(solenoid_pi_status == -1)
			{
				m_solenoid5->Set(false);
				m_solenoid6->Set(true); 	
			}
			else if(solenoid_pi_status == 1) 
			{
				m_solenoid5->Set(true);
				m_solenoid6->Set(false);
			}
 			back_last = m_controller->GetRawButton(BACK);
			
			///super shifters
			if (m_controller->GetRawButton(LSTICKP))
			{
				m_solenoid7->Set(true);
				m_solenoid8->Set(false);
			}
			else
			{
				m_solenoid7->Set(false);
				m_solenoid8->Set(true);
			}
			
			///ball gatherer
			if(m_controller->GetRawButton(ABUTTON))
			{
				jaguar7->Set(1);
			}
			else if(m_controller->GetRawButton(YBUTTON))
			{
				jaguar7->Set(-1);
			}
			else
			{
				jaguar7->Set(0);
			}
			
			///shooter motor
			if(m_controller->GetRawAxis(3)<0)  ///motor on
			{
				jaguar6->Set(SSpeed);//-m_controller->GetRawAxis(3)*12); ///get rid of the -1 on competition robot 
				jaguar5->Set(jaguar6->GetOutputVoltage());//-m_controller->GetRawAxis(3)*12);
				if (jaguar6->GetSpeed() > 0.8*SSpeed)          ///change to 0.9 on competition robot
				{
					spike1->Set(Relay::kOn);
					spike1->Set(Relay::kForward);
				}
				else
				{
					
					spike1->Set(Relay::kOff);	
				}
			}
			else              //// motor off
			{
				jaguar5->Set(0);
				jaguar6->Set(0);
				if(m_controller->GetRawButton(LBUMPER))
				{
					spike1->Set(Relay::kOn);
					spike1->Set(Relay::kReverse);
				}
				else if(m_controller->GetRawButton(RBUMPER))
				{
					spike1->Set(Relay::kOn);
					spike1->Set(Relay::kForward);
				}
				else
				spike1->Set(Relay::kOff);
			}
			
		
			////////////feed motor
			/*if(m_controller->GetRawButton(LBUMPER)) ///feed motor forward
			{
				spike1->Set(Relay::kOn);    
				spike1->Set(Relay::kForward);
			}
			else if(m_controller->GetRawAxis(3) < 0)///feed motor reverse
			{
				spike1->Set(Relay::kOn);
				spike1->Set(Relay::kReverse);		
			}
			else		///feed motor off
			{
				spike1->Set(Relay::kOff);
			}*/
			
			//dash->PutInt("Something", m_telePeriodicLoops);
			//dash->PutInt("Camera working?", m_camera->GetImage(image1));
			d_st->Printf(DriverStationLCD::kUser_Line1,1,"Juxtaposition = %3.2f   ", (double) m_telePeriodicLoops);       
			d_st->Printf(DriverStationLCD::kUser_Line2,1,"Encoder = %f   ",jaguar6->GetSpeed());
			d_st->Printf(DriverStationLCD::kUser_Line3,1,"Shooter speed = %d   ",SSpeed);
			d_st->Printf(DriverStationLCD::kUser_Line4,1,"Auton wait time = %d   ",auton_waitTime);
			d_st->Printf(DriverStationLCD::kUser_Line5,1,"Autonomous mode = %d   ",auton_mode);
			d_st->Printf(DriverStationLCD::kUser_Line6,1,"Encoder = %f   ",jaguar1->GetPosition() - encoder_zero);

			//d_st->Printf(DriverStationLCD::kUser_Line3,1,"Shooter Speed = %i", SSpeed);
			//d_st->Printf(DriverStationLCD::kUser_Line2,1,"BButton = %d   ", m_controller->GetRawButton(2));
			//d_st->Printf(DriverStationLCD::kUser_Line2,1,"button5 = %f   ", m_controller->GetRawAxis(5));
			d_st->UpdateLCD();
			Wait(0.05);
			//m_dds->sendIOPortData();
				/*
		}  // if (m_ds->GetPacketNumber()...
		*/

	} // TeleopPeriodic(void)
/********************************** Continuous Routines *************************************/

	/* 
	 * These routines are not used in this demonstration robot
	 *
	 * 
	void DisabledContinuous(void) {
	}

	void AutonomousContinuous(void)	{
	}

	void TeleopContinuous(void) {
	}
	*/
	
/********************************** Miscellaneous Routines *************************************/
	
	/**
	 * Clear KITT-style LED display on the solenoids
	 * 
	 * Clear the solenoid LEDs used for a KITT-style LED display.
	 */	
	void ClearSolenoidLEDsKITT() {
		
	}

	/**
+	 * Demonstrate handling of joystick buttons
	 * 
	 * This method expects to be called during each periodic loop, providing the following
	 * capabilities:
	 * - Print out a message when a button is initially pressed
	 * - Solenoid LEDs light up according to joystick buttons:
	 *   - When no buttons pressed, clear the solenoid LEDs
	 *   - When only one button is pressed, show the button number (in binary) via the solenoid LEDs
	 *   - When more than one button is pressed, show "15" (in binary) via the solenoid LEDs
	 */
	void DemonstrateJoystickButtons(Joystick *currStick,
									bool *buttonPreviouslyPressed,
									const char *stickString,
									Solenoid *solenoids[]) {
		
		UINT8 buttonNum = 1;				// start counting buttons at button 1
		bool outputGenerated = false;		// flag for whether or not output is generated for a button
		INT8 numOfButtonPressed = 0;		// 0 if no buttons pressed, -1 if00 multiple buttons pressed
		
		/* Iterate over all the buttons on the joystick, checking to see if each is pressed
		 * If a button is pressed, check to see if it is newly pressed; if so, print out a
		 * message on the console
		 */ 
		for (buttonNum = 1; buttonNum <= NUM_JOYSTICK_BUTTONS; buttonNum++) {
			if (currStick->GetRawButton(buttonNum)) {
				// the current button is pressed, now act accordingly...
				if (!buttonPreviouslyPressed[buttonNum]) {
					// button newly pressed; print out a message
					if (!outputGenerated) {
						// print out a heading if no other button pressed this cycle
						outputGenerated = true;
						printf("%s button pressed:", stickString);
					}
					printf(" %d", buttonNum);
				}
				// remember that this button is pressed for the next iteration
				buttonPreviouslyPressed[buttonNum] = true;
				
				// set numOfButtonPressed appropriately
				if (numOfButtonPressed == 0) {
					// no button pressed yet this time through, set the number correctly
					numOfButtonPressed = buttonNum;
				} else {
					// another button (or buttons) must have already been pressed, set appropriately
					numOfButtonPressed = -1;
				}
			} else {
				buttonPreviouslyPressed[buttonNum] = false;
			}
		}
		
		// after iterating through all the buttons, add a newline to output if needed
		if (outputGenerated) {
			printf("\n");
		}
		
		if (numOfButtonPressed == -1) {
			// multiple buttons were pressed, display as if button 15 was pressed
			DisplayBinaryNumberOnSolenoidLEDs(15, solenoids);
		} else {
			// display the number of the button pressed on the solenoids;
			// note that if no button was pressed (0), the solenoid display will be cleared (set to 0)
			DisplayBinaryNumberOnSolenoidLEDs(numOfButtonPressed, solenoids);
		}
	}

	/**
	 * Display a given four-bit value in binary on the given solenoid LEDs
	 */
	void DisplayBinaryNumberOnSolenoidLEDs(UINT8 displayNumber, Solenoid *solenoids[]) {

		if (displayNumber > 15) {
			// if the number to display is larger than can be displayed in 4 LEDs, display 0 instead
			displayNumber = 0;
		}
		
		solenoids[3]->Set( (displayNumber & 1) != 0);
		solenoids[2]->Set( (displayNumber & 2) != 0);
		solenoids[1]->Set( (displayNumber & 4) != 0);
		solenoids[0]->Set( (displayNumber & 8) != 0);
	}
	
	////code to  the robot (we added this)
	/*void DriveRobot(CANJaguar *jaguar1, CANJaguar *jaguar2, CANJaguar *jaguar3, CANJaguar *jaguar4, double setValue1, double setValue2)
	{
		if(setValue1 > 1 )setValue1 = 1;
		if(setValue1 < -1 )setValue1 = -1;
		if(setValue2 > 1 )setValue2 = 1;
		if(setValue2 < -1 )setValue2 = -1;
		
		setValue1 *= fabs(setValue1);
		setValue2 *= fabs(setValue2);
		
		jaguar1->Set(setValue1*12);
		jaguar2->Set(setValue1*12);
		jaguar3->Set(setValue2*12);
		jaguar4->Set(setValue2*12);
	}
	*/
	
	//distance is in feet,   jags  1 and 3   are masters
	/*void DriveRobot(CANJaguar *jaguar1, CANJaguar *jaguar2, CANJaguar *jaguar3, CANJaguar *jaguar4, double distance)
	{
		double distanceDriven = 0.0;
		while(distanceDriven < distance)
		{
			// Set motor,"PID"
			jaguar1->Set(6);
			jaguar2->Set(jaguar1->GetOutputVoltage());
			jaguar3->Set(6);
			jaguar4->Set(jaguar3->GetOutputVoltage());
			
			
		}
	}*/
	/////bridge tipper acts as a toggle
	/*void ToggleBridgeTipper()
	{
		static bool bridgeDown = false;
		if(bridgeDown) //if bridge is down then bridge raised
		{
			m_solenoid1->Set(false);
			m_solenoid2->Set(true);
			bridgeDown = false;
		}
		else	//if bridge is raised then bridge is lowered
		{
			m_solenoid1->Set(true);
			m_solenoid2->Set(false);
			bridgeDown = true;
		}
	}*/
	////1 = forward,0 = off, -1 = reverse
	void SetFeedMotor(int value)

	
	{
		switch(value)
		{
		case 1:			
			break;
		case -1:
			break;
		}
	}

	void AutonomousMode1()								//this should be very similar to the autonomous mode used at kettering
	{	
		m_solenoid3->Set(true); // omniwheels
		m_solenoid4->Set(false);//omniwheels
		m_solenoid5->Set(true);//solenoidpi
		m_solenoid6->Set(false);//solenoidpi
		m_solenoid7->Set(true);//supershifters
		m_solenoid8->Set(false);//supershifters
		m_solenoid1->Set(false);//bridge tipper
		m_solenoid2->Set(true);//bridge tipper
		
		if(m_timer->Get()== 0 )
		{
			m_timer->Start();
		}
		if(m_timer->Get() > auton_waitTime)
		{
			jaguar6->Set(auton_speed);
			jaguar5->Set(jaguar6->GetOutputVoltage());	//run shooter motor
			
			spike1->Set(Relay::kOn);					//run feed motor
			spike1->Set(Relay::kForward);
			
			jaguar7->Set(1);					//run ball gatherer
		}
		else
		{
			jaguar5->Set(0);							//turn off all the motors
			jaguar6->Set(0);
			spike1->Set(Relay::kOff);
			jaguar7->Set(0);
		}	
	}

	void AutonomousMode2()								//shoot, drive backwards, shoot
	{	
		m_solenoid3->Set(true); // omniwheels
		m_solenoid4->Set(false);//omniwheels
		m_solenoid5->Set(false);//solenoid3
		m_solenoid6->Set(true);//solenoid3
		m_solenoid7->Set(true);//supershifters
		m_solenoid8->Set(false);//supershifters
		m_solenoid1->Set(false);//bridge tipper
		m_solenoid2->Set(true);//bridge tipper
		
		if(m_timer->Get()== 0 )									//starts timer
		{
			m_timer->Start();
		}
		else if(m_timer->Get() < auton_waitTime)				//shoots from key
		{
			jaguar6->Set(auton_speed);
			//jaguar6->Set(2300+75*(jaguar1->GetPosition() - encoder_zero)*(5.5/16.5));
			jaguar5->Set(jaguar6->GetOutputVoltage());	//run shooter motor
			
			spike1->Set(Relay::kOn);					//run feed motor
			spike1->Set(Relay::kForward);
			
			jaguar7->Set(1);					//run ball gatherer
		}
		else if(jaguar1->GetPosition() - encoder_zero > -16.3)  //drive backward
		{
			jaguar5->Set(0);							// sets shooter to off
			jaguar6->Set(0);
			spike1->Set(Relay::kOff);					//sets feeder to off
			jaguar7->Set(1);					//starts ball gatherer
			m_robotdrive->ArcadeDrive(1,0,false);		//drives backwards
		}
		else													//shoots from bridge
		{
			jaguar6->Set(3000);							//starts shooter
			jaguar5->Set(jaguar6->GetOutputVoltage());
			spike1->Set(Relay::kOn);					//starts feed
			spike1->Set(Relay::kForward);
		}
	}
	void AutonomousMode3()								//shoot, drive backwards, shoot
	{	
		m_solenoid3->Set(true); // omniwheels
		m_solenoid4->Set(false);//omniwheels
		m_solenoid5->Set(false);//solenoid3
		m_solenoid6->Set(true);//solenoid3
		m_solenoid7->Set(true);//supershifters
		m_solenoid8->Set(false);//supershifters
		m_solenoid1->Set(false);//bridge tipper
		m_solenoid2->Set(true);//bridge tipper
		
		if(m_timer->Get()== 0 )									//starts timer
		{
			m_timer->Start();
		}
		else if(m_timer->Get() < auton_waitTime)				//shoots from key
		{
			jaguar6->Set(2600);
			//jaguar6->Set(2300+75*(jaguar1->GetPosition() - encoder_zero)*(5.5/16.5));
			jaguar5->Set(jaguar6->GetOutputVoltage());	//run shooter motor
			
			spike1->Set(Relay::kOn);					//run feed motor
			spike1->Set(Relay::kForward);
			
			jaguar7->Set(1);					//run ball gatherer
		}
		else if(jaguar1->GetPosition() - encoder_zero > -16.3)  //drive backward
		{
			jaguar5->Set(0);							// sets shooter to off
			jaguar6->Set(0);
			spike1->Set(Relay::kOff);					//sets feeder to off
			jaguar7->Set(1);					//starts ball gatherer
			m_robotdrive->ArcadeDrive(1,0,false);		//drives backwards
		}
		else													//shoots from bridge
		{
			jaguar6->Set(auton_speed);							//starts shooter
			jaguar5->Set(jaguar6->GetOutputVoltage());
			spike1->Set(Relay::kOn);					//starts feed
			spike1->Set(Relay::kForward);
		}
	}
};
//jaguar6->Set(2300+75*(jaguar1->GetPosition() - encoder_zero)*(5.5/16.5));

START_ROBOT_CLASS(BuiltinDefaultCode);
/*
 if(m_timer->Get()== 0 )
			{
				m_timer->Start();
			}
			else if(m_timer->Get() < 4000)
			{
				jaguar5->Set(2300);
				jaguar6->Set(2300);
				if(m_timer->Get() > 500)
				{
					->Set(Relay::kOn);
					->Set(Relay::kForward);
				}
				else
				{
					->Set(Relay::kOff);
				}
			}
			jaguar5->Set(0);
			jaguar6->Set(0);
			->Set(Relay::kOff);
			autonomousCase = 1;
			m_timer->Stop();
			m_timer->Reset();
			break;
		case 1:
			//drop bridge tipper
			m_solenoid1->Set(true);
			m_solenoid2->Set(false);
			autonomousCase = 2;
			break;
		case 2:
			//drive backward
			if (m_timer->Get() == 0 )
			{
				m_timer->Start();
			}
			else if(m_timer->Get() > 2000) ///number of milliseconds to drive backward exceeded
			{
				m_timer->Stop();
				m_timer->Reset();
				
				jaguar1->Set(0.0); //stop drive
				jaguar2->Set(0.0);
				jaguar3->Set(0.0);
				jaguar4->Set(0.0);
				
				autonomousCase = 3;
			}
			else
			{
				jaguar1->Set(-1.0);
				jaguar2->Set(-1.0);
				jaguar3->Set(-1.0);
				jaguar4->Set(-1.0);
			}
			break;
		case 3:
			//gather balls
			if(m_timer->Get() == 0)
			{
				m_timer->Start();
			}
			else if(m_timer->Get() > 4000)
			{
				m_timer->Stop();
				m_timer->Reset();
				
				->Set(Relay::kOff);
				
				autonomousCase = 4;
			}
			else //run ball gatherer
			{
				->Set(Relay::kOn);
				->Set(Relay::kReverse);
			}
			break;
 */
 
