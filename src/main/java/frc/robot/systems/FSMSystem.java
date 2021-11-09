package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class FSMSystem {
	/* ======================== Constants ======================== */
	public static final double WHEEL_DIAMETER_INCHES = 7.65;
	public static final double KP_MOVE_STRAIGHT = 0.1;
	public static final double ERR_THRESHOLD_STRAIGHT_IN = 0.1;
	
	// FSM state definitions
	public enum FSMState {
		START_STATE,
		FORWARD_STATE_10_IN,
		TURN_RIGHT
	}

	private static final float MOTOR_RUN_POWER = 0.1f;

	/* ======================== Private variables ======================== */
	private FSMState currentState;
	private boolean finishedMovingStraight;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	//private CANSparkMax exampleMotor;

	private CANSparkMax frontRightMotor;
	private CANSparkMax backRightMotor;
	private CANSparkMax frontLeftMotor;
	private CANSparkMax backLeftMotor;


	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public FSMSystem() {
		// Perform hardware init

		frontRightID = HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_RIGHT;
		frontLeftID = HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_LEFT;
		backRightID = HardwareMap.CAN_ID_SPARK_DRIVE_BACK_RIGHT;
		backLeftID = HardwareMap.CAN_ID_SPARK_DRIVE_BACK_LEFT;

		frontRightMotor = new CANSparkMax(frontRightID, CANSparkMax.MotorType.kBrushless);
		frontLeftMotor = new CANSparkMax(frontLeftID, CANSparkMax.MotorType.kBrushless);
		backRightMotor = new CANSparkMax(backRightID, CANSparkMax.MotorType.kBrushless);
		backLeftMotor = new CANSparkMax(backLeftID, CANSparkMax.MotorType.kBrushless);

		frontRightMotor.getEncoder().setPosition(0);
		frontLeftMotor.getEncoder().setPosition(0);
		backRightMotor.getEncoder().setPosition(0);
		backLeftMotor.getEncoder().setPosition(0);

		finishedMovingStraight = false;

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

			case FORWARD_STATE_10_IN:
				handleForwardOrBackwardState(input, 10, 10);
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
					return FSMState.FORWARD_STATE_10_IN;
				} else {
					return FSMState.START_STATE;
				}

			case FORWARD_STATE_10_IN:
				if(finishedMovingStraight) {
					finishedMovingStraight = false;
					return FSMState.TURN_RIGHT;
				} else {
					return FORWARD_STATE_10_IN;
				}
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
		setPowerForAllMotors(0); //start with all motors set to 0
	}

	//Assume encoder starts at 0
	/**
	* Handle behavior in FORWARD_STATE, or BACKWARD_STATE
	* @param input Global TeleopInput if robot in teleop mode or null if
	*        the robot is in autonomous mode.
	* @param inches The number of inches to move forward or backward
	*/
	private void handleForwardOrBackwardState(TeleopInput input, double inches) {
		double currentPosInches = frontLeftMotor.getEncoder().getPosition() * Math.PI * WHEEL_DIAMETER_INCHES;
		double error = inches - currentPosInches;
		if(error < ERR_THRESHOLD_STRAIGHT_IN) {
			finishedMovingStraight = true;
		}
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
	* Sets power for all motors.
	* @param power The power to set all the motors.
	*/
	public void setPowerForAllMotors(double power) {
		frontLeftMotor.set(power);
		frontRightMotor.set(power);
		backLeftMotor.set(power);
		backRightMotor.set(power);
	}
}
