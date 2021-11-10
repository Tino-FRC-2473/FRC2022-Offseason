package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.kauailabs.navx.frc.AHRS;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class DriveFSMSystem {
	/* ======================== Constants ======================== */
	public static final double WHEEL_DIAMETER_INCHES = 7.65;
	public static final double KP_MOVE_STRAIGHT = 0.1;
	// FSM state definitions
	public enum FSMState {
		START_STATE,
		FORWARD_STATE_10_IN,
		TURN_STATE,
		TELEOP_STATE
	}

	private static final float MOTOR_RUN_POWER = 0.1f;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

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

		frontRightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_RIGHT, 
											CANSparkMax.MotorType.kBrushless);
		frontLeftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_LEFT, 
											CANSparkMax.MotorType.kBrushless);
		backRightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_BACK_RIGHT, 
											CANSparkMax.MotorType.kBrushless);
		backLeftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_BACK_LEFT, 
											CANSparkMax.MotorType.kBrushless);

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
		currentState = FSMState.TELEOP_STATE;

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

	/* ======================== Private methods ======================== */
	/**
	 * Decide the next state to transition to. This is a function of the inputs
	 * and the current state of this FSM. This method should not have any side
	 * effects on outputs. In other words, this method should only read or get
	 * values to decide what state to go to.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private FSMState nextState(TeleopInput input) {
		switch (currentState) {
			case START_STATE:
				if (input != null) {
					return FSMState.TELEOP_STATE;
				} else {
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
	private void handleForwardOrBackwardState(TeleopInput input, double inches) {
		double currentPos_inches = 
			frontLeftMotor.getEncoder().getPosition() * Math.PI * WHEEL_DIAMETER_INCHES;
		double error = inches - currentPos_inches;
		double speed = KP_MOVE_STRAIGHT * error;

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
	private void setPowerForAllMotors(double power) {
		frontLeftMotor.set(power);
		frontRightMotor.set(power);
		backLeftMotor.set(power);
		backRightMotor.set(power);
	}

	private void handleTurnState(TeleopInput input, double degrees) { // turn x degrees, +x is right, -x is left
		double error = degrees - getHeading();
		double power = error / 360;

		frontLeftMotor.set(power);
		frontRightMotor.set(power);
		backLeftMotor.set(power);
		backRightMotor.set(power);
	}

	private double getHeading() {
		return -Math.IEEEremainder(gyro.getAngle(), 360);
	}

	private void handleTeleOpState(TeleopInput input) {
		if(input == null) return;
		double leftPower = -input.getDrivingJoystickY() * (1 + input.getSteerAngleDegrees() / 90);
		double rightPower = input.getDrivingJoystickY() * (1 - input.getSteerAngleDegrees() / 90);

		limitPower(leftPower);
		limitPower(rightPower);

		frontRightMotor.set(rightPower);
        frontLeftMotor.set(leftPower);
        backRightMotor.set(rightPower);
        backLeftMotor.set(leftPower);
	}

	private void limitPower(double number){
		if (number > 1) {
			number = 1;
		}
		if (number < -1) {
			number = -1;
		}
	}
}