package frc.robot.systems;

import javax.lang.model.util.ElementScanner6;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Solenoid;
// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class FSMSystem {
    /* ======================== Constants ======================== */
    // FSM state definitions
    public enum FSMState {
        RETRACTED,
        EXTENDED,
        INTAKING_RETRACTED,
        INTAKING_EXTENDED,
        SHOOT_RETRACTED,
        SHOOT_EXTENDED
    }
    
    private static final float MOTOR_SHOOTING_POWER = 0.2f;
    private static final float MOTOR_INTAKE_POWER = 0.1f;
    private static final float MOTOR_TRANSPORT_POWER = 0.07f;
    
    /* ======================== Private variables ======================== */
    private FSMState currentState;

    private boolean canToggleRamp;
    
    // Hardware devices should be owned by one and only one system. They must
    // be private to their owner system and may not be used elsewhere.
    private CANSparkMax shooterMotor, intakeMotor, transportMotor;
    
    private Solenoid armActuator;
    
    /* ======================== Constructor ======================== */
    /**
        * Create FSMSystem and initialize to starting state. Also perform any
        * one-time initialization or configuration of hardware required. Note
        * the constructor is called only once when the robot boots.
        */
    public FSMSystem() {
        // Perform hardware init
        shooterMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_SHOOTER,
                                        CANSparkMax.MotorType.kBrushless);
        intakeMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_INTAKE,
                                        CANSparkMax.MotorType.kBrushless);
        transportMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_TRANSPORT_PULLEY,
                                        CANSparkMax.MotorType.kBrushless);
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
        * Reset this system to its start state. This may be called from mode init
        * when the robot is enabled.
        *
        * Note this is distinct from the one-time initialization in the constructor
        * as it may be called multiple times in a boot cycle,
        * Ex. if the robot is enabled, disabled, then reenabled.
        */
    public void reset() {
        currentState = FSMState.RETRACTED;

        canToggleRamp = true;
    
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
            case EXTENDED:
                handleExtendedState(input);
                break;
                
            case INTAKING_EXTENDED:
                handleIntakingExtendedState(input);
                break;
                
            case INTAKING_RETRACTED:
                handleIntakingRetractedState(input);
                break;
    
            case SHOOT_EXTENDED:
                handleShootExtendedState(input);
                break;
    
            case SHOOT_RETRACTED:
                handleShootRetractedState(input);
                break;
    
            case RETRACTED:
                handleRetractedState(input);
                break;
    
            default:
                throw new IllegalStateException("Invalid state: " + currentState.toString());
        }
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
            case RETRACTED:
                if (input != null) {
                    if(input.isShooterButtonPressed())
                        return FSMState.SHOOT_RETRACTED;
                    else if(input.isIntakeButtonPressed())
                        return FSMState.INTAKING_RETRACTED;
                    else if(input.isRampToggleButtonPressed() && canToggleRamp){
                        canToggleRamp = false;
                        return FSMState.EXTENDED;
                    } else if(!input.isRampToggleButtonPressed())
                        canToggleRamp = true;
                    else
                        return FSMState.RETRACTED;
                } else {
                    return FSMState.RETRACTED;
                }
    
            case EXTENDED:
                if(input != null){
                    if(input.isShooterButtonPressed())
                        return FSMState.SHOOT_EXTENDED;
                    else if(input.isIntakeButtonPressed())
                        return FSMState.INTAKING_EXTENDED;
                    else if(input.isRampToggleButtonPressed() && canToggleRamp){
                        canToggleRamp = false;
                        return FSMState.RETRACTED;
                    } else if(!input.isRampToggleButtonPressed())
                        canToggleRamp = true;
                    else
                        return FSMState.EXTENDED;
                } else
                    return FSMState.EXTENDED;
    
            case SHOOT_EXTENDED:
                if(input != null){
                    if(input.isShooterButtonPressed())
                        return FSMState.SHOOT_EXTENDED;
                    else
                        return FSMState.EXTENDED;
                }else
                    return FSMState.EXTENDED;
    
            case SHOOT_RETRACTED:
                if(input != null){
                    if(input.isShooterButtonPressed())
                        return FSMState.SHOOT_RETRACTED;
                    else
                        return FSMState.RETRACTED;
                }else
                    return FSMState.RETRACTED;

            case INTAKING_EXTENDED:
                if(input != null){
                    if(input.isIntakeButtonPressed())
                        return FSMState.INTAKING_EXTENDED;
                    else
                        return FSMState.EXTENDED;
                }else
                    return FSMState.EXTENDED;
            
            case INTAKING_RETRACTED:
                if(input != null){
                    if(input.isIntakeButtonPressed())
                        return FSMState.INTAKING_RETRACTED;
                    else
                        return FSMState.RETRACTED;
                }else
                    return FSMState.RETRACTED;
    
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
    private void handleExtendedState(TeleopInput input) {
        armActuator.set(true);
        shooterMotor.set(0);
        intakeMotor.set(0);
        transportMotor.set(0);
    }
    /**
    * Handle behavior in OTHER_STATE.
    * @param input Global TeleopInput if robot in teleop mode or null if
    *        the robot is in autonomous mode.
    */
    private void handleRetractedState(TeleopInput input) {
        armActuator.set(false);
        shooterMotor.set(0);
        intakeMotor.set(0);
        transportMotor.set(0);
    }
    /**
        * Handle behavior in OTHER_STATE.
        * @param input Global TeleopInput if robot in teleop mode or null if
        *        the robot is in autonomous mode.
        */
    private void handleIntakingExtendedState(TeleopInput input) {
        armActuator.set(true);
        shooterMotor.set(0);
        intakeMotor.set(MOTOR_INTAKE_POWER);
        transportMotor.set(MOTOR_TRANSPORT_POWER);
    }
    /**
        * Handle behavior in OTHER_STATE.
        * @param input Global TeleopInput if robot in teleop mode or null if
        *        the robot is in autonomous mode.
        */
    private void handleIntakingRetractedState(TeleopInput input) {
        armActuator.set(false);
        shooterMotor.set(0);
        intakeMotor.set(MOTOR_INTAKE_POWER);
        transportMotor.set(MOTOR_TRANSPORT_POWER);
    }
    /**
        * Handle behavior in OTHER_STATE.
        * @param input Global TeleopInput if robot in teleop mode or null if
        *        the robot is in autonomous mode.
        */
    private void handleShootExtendedState(TeleopInput input) {
        armActuator.set(true);
        shooterMotor.set(MOTOR_SHOOTING_POWER);
        intakeMotor.set(0);
        transportMotor.set(0);
    }
    /**
        * Handle behavior in OTHER_STATE.
        * @param input Global TeleopInput if robot in teleop mode or null if
        *        the robot is in autonomous mode.
        */
    private void handleShootRetractedState(TeleopInput input) {
        armActuator.set(false);
        shooterMotor.set(MOTOR_SHOOTING_POWER);
        intakeMotor.set(0);
        transportMotor.set(0);
    }
}