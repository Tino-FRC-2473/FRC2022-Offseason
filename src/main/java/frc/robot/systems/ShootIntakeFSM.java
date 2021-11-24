package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import net.thefletcher.revrobotics.CANSparkMax;
import net.thefletcher.revrobotics.enums.MotorType;
// Above imports are for simulation purposes, replace with bottom when building to run
// import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Solenoid;
// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class ShootIntakeFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		IDLE,
		INTAKING,
		SHOOTING,
		INTAKING_AND_SHOOTING
	}

	private static final float MOTOR_SHOOTING_POWER = 0.2f;
	private static final float MOTOR_INTAKE_POWER = 0.1f;
	private static final float MOTOR_TRANSPORT_POWER = 0.07f;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	private boolean rampState; //true for extended; false for retracted

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax shooterMotor;
	private CANSparkMax intakeMotor;
	private CANSparkMax transportMotor;

	private Solenoid armActuator;

	/* ======================== Constructor ======================== */
	/**
		* Create FSMSystem and initialize to starting state. Also perform any
		* one-time initialization or configuration of hardware required. Note
		* the constructor is called only once when the robot boots.
		*/
	public ShootIntakeFSM() {
		// Perform hardware init

		////////INIT CODE FOR NORMAL RUN SYSTEM, WITHOUT SPARK MAX SIM
		// shooterMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_SHOOTER,
		//                                 CANSparkMax.MotorType.kBrushless);
		// intakeMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_INTAKE,
		//                                 CANSparkMax.MotorType.kBrushless);
		// transportMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_TRANSPORT_PULLEY,
		//                                 CANSparkMax.MotorType.kBrushless);


		shooterMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_SHOOTER,
										MotorType.kBrushless);
		intakeMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_INTAKE,
										MotorType.kBrushless);
		transportMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_TRANSPORT_PULLEY,
										MotorType.kBrushless);
		armActuator = new Solenoid(HardwareMap.PCM_CHANNEL_INTAKE_CYLINDER_FORWARD);

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
		* Return current ramp state.
		* @return Current ramp state
		*/
	public boolean getRampState(){
		return rampState;
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
		currentState = FSMState.IDLE;

		rampState = false;
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
			case IDLE:
				handleIdleState(input);
				break;

			case INTAKING:
				handleIntakingState(input);
				break;

			case SHOOTING:
				handleShootingState(input);
				break;

			case INTAKING_AND_SHOOTING:
				handleIntakingAndShootingState(input);

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}

		currentState = nextState(input);

		handleRemainingSystem(input);
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
			case IDLE:
				if (input != null) {
					if (input.isRampToggleButtonPressed()) {
						rampState = !rampState;
						return FSMState.IDLE;
					} else if (input.isIntakeButtonPressed() && input.isShooterButtonPressed()) {
						return FSMState.INTAKING_AND_SHOOTING;
					} else if (input.isShooterButtonPressed()) {
						return FSMState.SHOOTING;
					} else if (input.isIntakeButtonPressed()) {
						return FSMState.INTAKING;
					}
				}
				return FSMState.IDLE;

			case SHOOTING:
				if (input != null) {
					if (input.isIntakeButtonPressed() && input.isShooterButtonPressed()) {
						return FSMState.INTAKING_AND_SHOOTING;
					} else if (input.isShooterButtonPressed()) {
						return FSMState.SHOOTING;
					} else if (input.isIntakeButtonPressed()) {
						return FSMState.INTAKING;
					}
				}
				return FSMState.IDLE;

			case INTAKING:
				if (input != null) {
					if (input.isIntakeButtonPressed() && input.isShooterButtonPressed()) {
						return FSMState.INTAKING_AND_SHOOTING;
					} else if (input.isShooterButtonPressed()) {
						return FSMState.SHOOTING;
					} else if (input.isIntakeButtonPressed()) {
						return FSMState.INTAKING;
					}
				}
				return FSMState.IDLE;

			case INTAKING_AND_SHOOTING:
				if (input.isIntakeButtonPressed() && input.isShooterButtonPressed()) {
					return FSMState.INTAKING_AND_SHOOTING;
				} else if (input.isShooterButtonPressed()) {
					return FSMState.SHOOTING;
				} else if (input.isIntakeButtonPressed()) {
					return FSMState.INTAKING;
				}
				return FSMState.IDLE;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
		* Handle behavior in IDLE.
		* @param input Global TeleopInput if robot in teleop mode or null if
		*        the robot is in autonomous mode.
		*/
	private void handleIdleState(TeleopInput input) {
		shooterMotor.set(0);
		intakeMotor.set(0);
		transportMotor.set(0);
	}
	/**
		* Handle behavior in INTAKING.
		* @param input Global TeleopInput if robot in teleop mode or null if
		*        the robot is in autonomous mode.
		*/
	private void handleIntakingState(TeleopInput input) {
		shooterMotor.set(0);
		intakeMotor.set(MOTOR_INTAKE_POWER);
		transportMotor.set(MOTOR_TRANSPORT_POWER);
	}
	/**
		* Handle behavior in SHOOTING.
		* @param input Global TeleopInput if robot in teleop mode or null if
		*        the robot is in autonomous mode.
		*/
	private void handleShootingState(TeleopInput input) {
		shooterMotor.set(MOTOR_SHOOTING_POWER);
		intakeMotor.set(0);
		transportMotor.set(0);
	}
	/**
		* Handle behavior in SHOOTING.
		* @param input Global TeleopInput if robot in teleop mode or null if
		*        the robot is in autonomous mode.
		*/
	private void handleIntakingAndShootingState(TeleopInput input) {
		shooterMotor.set(MOTOR_SHOOTING_POWER);
		intakeMotor.set(MOTOR_INTAKE_POWER);
		transportMotor.set(MOTOR_TRANSPORT_POWER);
	}



	/**
		* Handle behavior of system sumparts.
		* @param input Global TeleopInput if robot in teleop mode or null if
		*        the robot is in autonomous mode.
		*/
	private void handleRemainingSystem(TeleopInput input){
		if (rampState) {
			armActuator.set(true);
		} else {
			armActuator.set(false);
		}
	}
}
