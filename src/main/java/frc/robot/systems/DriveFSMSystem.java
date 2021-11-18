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
	public static final double ERR_THRESHOLD_STRAIGHT_IN = 0.1;
	private static final double TELEOP_ANGLE_POWER_RATIO = 90.0;
	private static final double TURN_ERROR_POWER_RATIO = 360;
	private static final double MIN_TURN_POWER = 0.1;
	private static final double TURN_ERROR_THRESHOLD_DEGREE = 1.0;
	private static final double COUNTS_PER_MOTOR_REVOLUTION = 42;
	private static final double GEAR_RATIO = 26.0 * 4.67 / 12.0;
	private static final double DRIVE_TICKS_PER_INCH
		= COUNTS_PER_MOTOR_REVOLUTION * GEAR_RATIO / (Math.PI * WHEEL_DIAMETER_INCHES);

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
	private boolean finishedMovingStraight;
	private boolean finishedTurning;
	private double forwardStateInitialEncoderPos = -1;
	private double rawGyroAngle = 0;
	private double robotXPosLine = 0;
	private double robotYPosLine = 0;
	private double prevEncoderPosLine = 0;
	private double prevEncoderPosArc = 0;
	private double robotXPosArc = 0;
	private double robotYPosArc = 0;
	private double prevGyroAngle = 0;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

	private CANSparkMax frontRightMotor;
	private CANSparkMax backRightMotor;
	private CANSparkMax frontLeftMotor;
	private CANSparkMax backLeftMotor;

	private AHRS gyro;

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

		finishedMovingStraight = false;
		finishedTurning = false;

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
		rawGyroAngle = gyro.getAngle();
		updateLineOdometry();
		updateArcOdometry();
		switch (currentState) {
			case START_STATE:
				handleStartState(input);
				break;

			case TELEOP_STATE:
				handleTeleOpState(input);
				break;

			case FORWARD_STATE_10_IN:
				handleForwardOrBackwardState(input, 10,
					forwardStateInitialEncoderPos != -1 ? forwardStateInitialEncoderPos
					: frontLeftMotor.getEncoder().getPosition());
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
				if (finishedMovingStraight) {
					finishedMovingStraight = false;
					forwardStateInitialEncoderPos = -1;
					return FSMState.TURN_STATE;
				} else {
					return FSMState.FORWARD_STATE_10_IN;
				}

			case TURN_STATE:
				if (finishedTurning) {
					finishedTurning = false;
					return FSMState.TELEOP_STATE;
				} else {
					return FSMState.TURN_STATE;
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
	//Assume encoder starts at 0.
	/**
	* Handle behavior in FORWARD_STATE, or BACKWARD_STATE.
	* @param input Global TeleopInput if robot in teleop mode or null if
	*        the robot is in autonomous mode.
	* @param inches The number of inches to move forward or backward
	* @param initialEncoderPos The encoder position of the front left motor
	* when the state/handler method was first initiated
	*/
	private void handleForwardOrBackwardState(TeleopInput input,
		double inches, double initialEncoderPos) {
		forwardStateInitialEncoderPos = initialEncoderPos;
		double positionRev = frontLeftMotor.getEncoder().getPosition() - initialEncoderPos;
		double currentPosInches = positionRev * Math.PI * WHEEL_DIAMETER_INCHES;
		double error = inches - currentPosInches;
		if (error < ERR_THRESHOLD_STRAIGHT_IN) {
			finishedMovingStraight = true;
		}
		double speed = KP_MOVE_STRAIGHT * error;

		if (speed >= 1) {
			setPowerForAllMotors(1);
		} else if (speed <= -1) {
			setPowerForAllMotors(-1);
		} else {
			setPowerForAllMotors(speed);
		}
	}

	/**
	* Sets power for all motors.
	* @param power The power to set all the motors to
	*/
	private void setPowerForAllMotors(double power) {
		frontLeftMotor.set(-power);
		frontRightMotor.set(power);
		backLeftMotor.set(-power);
		backRightMotor.set(power);
	}

	// turn x degrees, +x is right, -x is left
	private void handleTurnState(TeleopInput input, double degrees) {
		double error = degrees - getHeading();
		if (error <= TURN_ERROR_THRESHOLD_DEGREE) {
			finishedTurning = true;
			return;
		}
		double power = error / TURN_ERROR_POWER_RATIO;
		if (Math.abs(power) < MIN_TURN_POWER) {
			power = MIN_TURN_POWER * power < 0 ? -1 : 1;
		}

		frontLeftMotor.set(power);
		frontRightMotor.set(-power);
		backLeftMotor.set(power);
		backRightMotor.set(-power);
	}

	/**
	* Gets the heading from the gyro.
	* @return the gyro heading
	*/
	private double getHeading() {
		return -Math.IEEEremainder(gyro.getAngle(), 360);
	}

	private void handleTeleOpState(TeleopInput input) {
		if (input == null) {
			return;
		}

		double joystickZ = input.getDrivingJoystickZ();
		double steerAngle = -input.getSteerAngle();

		double leftPower = -joystickZ * (1 + steerAngle);
		double rightPower = joystickZ * (1 - steerAngle);

		leftPower = limitPower(leftPower);
		rightPower = limitPower(rightPower);

		System.out.println("Driving Stick: " + joystickZ);
		System.out.println("Steering Wheel: " + steerAngle);
		System.out.println("Left power: " + leftPower);
		System.out.println("Right Power: " + rightPower);

		frontRightMotor.set(rightPower);
		frontLeftMotor.set(leftPower);
		backRightMotor.set(rightPower);
		backLeftMotor.set(leftPower);
	}

	private double limitPower(double number) {
		if (number > 1) {
			return 1;
		}
		if (number < -1) {
			return -1;
		}
		return number;
	}

	private void updateLineOdometry() {
		double adjustedAngle = 90 - rawGyroAngle;
		double currentEncoderPos = ((frontLeftMotor.getEncoder().getPosition()
			+ frontRightMotor.getEncoder().getPosition()) / 2.0);
		double dEncoder = (currentEncoderPos - prevEncoderPosLine) / DRIVE_TICKS_PER_INCH;
		double dX = dEncoder * Math.sin(Math.toRadians(adjustedAngle));
		double dY = dEncoder * Math.cos(Math.toRadians(adjustedAngle));
		robotXPosLine += dX;
		robotYPosLine += dY;

		prevEncoderPosLine = currentEncoderPos;
	}

	private void updateArcOdometry() {
		double adjustedAngle = 90 - rawGyroAngle;
		double theta = Math.abs(adjustedAngle - prevGyroAngle);
		double currentEncoderPos = ((frontLeftMotor.getEncoder().getPosition()
			+ frontRightMotor.getEncoder().getPosition()) / 2.0);
		double arcLength = (currentEncoderPos - prevEncoderPosArc) / DRIVE_TICKS_PER_INCH;
		double radius = 180 * arcLength / (Math.PI * theta);
		double alpha = prevGyroAngle - 90;
		double circleX = robotXPosArc + radius * Math.cos(Math.toRadians(alpha));
		double circleY = robotYPosArc + radius * Math.sin(Math.toRadians(alpha));
		double beta = alpha + 180 - theta;
		robotXPosArc = circleX + radius * Math.cos(Math.toRadians(beta));
		robotYPosArc = circleY + radius * Math.sin(Math.toRadians(beta));

		prevGyroAngle = adjustedAngle;
		prevEncoderPosArc = currentEncoderPos;
	}
}
