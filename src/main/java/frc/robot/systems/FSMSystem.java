package frc.robot.systems;
 
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
       IDLE,
       RUN_INTAKE,
       RETRACTED
   }
 
   private static final float MOTOR_RUN_POWER = 0.1f;
 
   /* ======================== Private variables ======================== */
   private FSMState currentState;
 
   // Hardware devices should be owned by one and only one system. They must
   // be private to their owner system and may not be used elsewhere.
   private CANSparkMax exampleMotor;
 
   private Solenoid armActuator;
 
   /* ======================== Constructor ======================== */
   /**
    * Create FSMSystem and initialize to starting state. Also perform any
    * one-time initialization or configuration of hardware required. Note
    * the constructor is called only once when the robot boots.
    */
   public FSMSystem() {
       // Perform hardware init
       exampleMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_SHOOTER,
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
 
           case RUN_INTAKE:
               handleRunIntakeState(input);
               break;
 
           case RETRACTED:
               handleRetractedState(input);
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
           case RETRACTED:
               if (input != null) {
                   if(input.isIntakeButtonPressed()) //TODO
                       return FSMState.IDLE;
                   else
                       return FSMState.RETRACTED;
               } else {
                   return FSMState.RETRACTED;
               }
 
           case IDLE:
               if(input != null && input.isIntakeButtonPressed())
                   return FSMState.RUN_INTAKE;
               else
                   return FSMState.IDLE;
 
           case RUN_INTAKE:
               if(input != null && !input.isIntakeButtonPressed())
                   return FSMState.IDLE;
               else
                   return FSMState.RUN_INTAKE;
 
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
   private void handleIdleState(TeleopInput input) {
       armActuator.set(true);
       exampleMotor.set(0);
   }
   /**
    * Handle behavior in OTHER_STATE.
    * @param input Global TeleopInput if robot in teleop mode or null if
    *        the robot is in autonomous mode.
    */
   private void handleRunIntakeState(TeleopInput input) {
       exampleMotor.set(MOTOR_RUN_POWER);
   }
   /**
    * Handle behavior in OTHER_STATE.
    * @param input Global TeleopInput if robot in teleop mode or null if
    *        the robot is in autonomous mode.
    */
   private void handleRetractedState(TeleopInput input) {
       armActuator.set(false);
       exampleMotor.set(0);
   }
}
