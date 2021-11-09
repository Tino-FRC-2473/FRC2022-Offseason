package frc.robot.systems;

 // WPILib Imports
 import edu.wpi.first.wpilibj.SPI;

 // Third party Hardware Imports
 import com.revrobotics.CANSparkMax;
 import com.revrobotics.CANSparkMaxLowLevel;
 import com.revrobotics.ControlType;
 import com.kauailabs.navx.frc.AHRS;
 import edu.wpi.first.wpilibj.SPI;

 // Robot Imports
 import frc.robot.TeleopInput;
 @@ -32,7 +32,6 @@

 	// Hardware devices should be owned by one and only one system. They must
 	// be private to their owner system and may not be used elsewhere.
 	//private CANSparkMax exampleMotor;

 	private CANSparkMax frontRightMotor;
 	private CANSparkMax backRightMotor;
	private CANSparkMax frontLeftMotor;
	private CANSparkMax backLeftMotor;
	AHRS gyro;
	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public DriveFSMSystem() {
		// Perform hardware init
		frontRightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_RIGHT, CANSparkMax.MotorType.kBrushless);
		frontLeftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_LEFT, CANSparkMax.MotorType.kBrushless);
		backRightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_BACK_RIGHT, CANSparkMax.MotorType.kBrushless);
		backLeftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_BACK_LEFT, CANSparkMax.MotorType.kBrushless);
		
		frontRightMotor.getEncoder().setPosition(0);
		frontLeftMotor.getEncoder().setPosition(0);
		backRightMotor.getEncoder().setPosition(0);
		backLeftMotor.getEncoder().setPosition(0);
		// Reset state machine
		reset();
	}
	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public FSMState getCurrentState() {
		return currentState;
	}
	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void reset() {
		currentState = FSMState.START_STATE;
		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}
	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		switch (currentState) {
			case START_STATE:
				handleStartState(input);
				break;
			case TELEOP_STATE:
				handleTeleOpState(input);
                break;
                
			case FORWARD_STATE_10_IN:
				handleForwardOrBackwardState(input, 10);
				break;
			case TURN_STATE:
				handleTurnState(input, 90); // test 90 degrees
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		currentState = nextState(input);
	}
 @@ -135,69 +134,69 @@
					return FSMState.START_STATE;
				}
			case TELEOP_STATE:
				return FSMState.TELEOP_STATE;
            
			case FORWARD_STATE_10_IN:
				return FSMState.TURN_STATE;
				//should be return FSMState.TURN_RIGHT; once simon pushes his code
			case TURN_STATE:
				return FSMState.START_STATE;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}
	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in START_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleStartState(TeleopInput input) {
		setPowerForAllMotors(0);
	}
	//Assume encoder starts at 0.
	/**
	* Handle behavior in FORWARD_STATE, or BACKWARD_STATE
	* @param input Global TeleopInput if robot in teleop mode or null if
	*        the robot is in autonomous mode.
	* @param inches The number of inches to move forward or backward
	*/
	//Assume encoder positions are at 0 initially
	public void handleForwardOrBackwardState(TeleopInput input, double inches) {
		double currentPos_inches = 
			frontLeftMotor.getEncoder().getPosition() * Math.PI * WHEEL_DIAMETER_INCHES;
		double error = inches - currentPos_inches;
		double speed = kP_move_straight * error;
		if(speed >= 1) {
			setPowerForAllMotors(1);
		} else if(speed <= -1) {
			setPowerForAllMotors(-1);
		} else {
			setPowerForAllMotors(speed);
		}
	}
	
	/**
	* Sets power for all motors
	* @param power The power to set all the motors to
	*/
	public void setPowerForAllMotors(double power) {
		frontLeftMotor.set(power);
		frontRightMotor.set(power);
		backLeftMotor.set(power);
 		backRightMotor.set(power);
 	}

	private void handleTurnState(TeleopInput input, double degrees) { // turn x degrees, +x is right, -x is left
		double error = degrees - getHeading();
		double power = error / 360;

		frontLeftMotor.set(power);
		frontRightMotor.set(-power);
		backLeftMotor.set(power);
		backRightMotor.set(-power);
	}

	public double getHeading() {
		return -Math.IEEEremainder(gyro.getAngle(), 360);
	}

	private void limitPower(double number){
		if(number > 1) {
			number = 1;
		}
		if(number < -1) {
			number = -1;
		}
	}
}