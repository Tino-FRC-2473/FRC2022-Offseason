package frc.robot.systems;

import java.util.Scanner;

// WPILib Imports


// Robot Imports
import frc.robot.TeleopInput;

public class SerialDecoder {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		IDLE,
		LISTENING
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	private Data[] currentData;

	private String incomingData;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public SerialDecoder() {
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
		currentState = FSMState.IDLE;

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

			case LISTENING:
				handleListeningState(input);
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
			case IDLE:
				return currentState;

			case LISTENING:
				return currentState;

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
	}
	/**
	 * Handle behavior in LISTENING.
	 * This method will see if the system has received a new full message from the jetson, and
	 * if so, it parses and translates the most recent, full, completed message to a Data[],
	 * and sets the active data to this new data array.
	 * Example value for the variable incomingData:
	 * "
	 * [Open]
	 * [Circle 234 23 2]
	 * [Circle 234 230 2]
	 * [Close]
	 * [Open]
	 * [Circle 234 23 2]
	 * [Close]
	 * [Open]
	 * [Ci
	 * "
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleListeningState(TeleopInput input) {
		while (true) {
			int startIndex = incomingData.indexOf("[Open]");
			int endIndex = incomingData.indexOf("[Close]");

			if (startIndex == -1 || endIndex == -1) {
				return;
			}

			String rawPackets = incomingData.substring(startIndex + 7, endIndex - 1);
			incomingData = incomingData.substring(endIndex + 8);

			try {
				int numPackets = 1;
				for (int i = 0; i < rawPackets.length(); i++) {
					if (rawPackets.charAt(i) == '\n') {
						numPackets++;
					}
				}
				Data[] parsedData = new Data[numPackets];
				for (int i = 0; i < numPackets; i++) {
					parsedData[i] = Data.stringToData(
						rawPackets.substring(0, rawPackets.indexOf("\n"))
						);
				}

				currentData = parsedData;
			} catch (Exception e) {
				System.out.println(e);
			}
		}
	}

	/**
	 * Recieves string input from serial communication.
	 * @param str Raw data sent by serial bus
	 */
	public void onData(String str) {
		incomingData += str;
	}

	/**
	 * Returns most recent parsed data sent by Jetson.
	 * @return Most recent parsed CV data sent by Jetson
	 */
	public Data[] getData() {
		return currentData.clone();
	}

	/**
	 * Returns most recent parsed data sent by Jetson, containing Circles.
	 * @return Most recent parsed CV data sent by Jetson containg Circles
	 */
	public Circle[] getCircles() {
		int c = 0;
		for (Data d : currentData) {
			if (d instanceof Circle) {
				c++;
			}
		}

		Circle[] circles = new Circle[c];

		c = 0;
		for (Data d : currentData) {
			if (d instanceof Circle) {
				circles[c] = (Circle) d;
				c++;
			}
		}
		return circles;
	}

}

class Data {
	public static Data stringToData(String str) throws IllegalArgumentException {
		str = str.substring(1, str.length() - 1);

		Scanner sc = new Scanner(str);

		String type = sc.next();

		switch (type) {
			case "Circle":
				String xStr = sc.next();
				String yStr = sc.next();
				String rStr = sc.next();

				sc.close();

				int x;
				int y;
				int r;

				try {
					x = Integer.parseInt(xStr);
					y = Integer.parseInt(yStr);
					r = Integer.parseInt(rStr);
				} catch (NumberFormatException e) {
					throw new IllegalArgumentException("Recieved invalid numerical data. Received:\n" +
					"x = " + xStr + "\n" +
					"y = " + yStr + "\n" +
					"r = " + rStr);
				}

				return new Circle(x, y, r);

			default:
				sc.close();
				throw new IllegalArgumentException("Data Type is Unrecognized, recieved type \"" +
				type + "\"");
		}

	}
}

class Circle extends Data {

	private final int x;

	private final int y;

	private final int r;

	public Circle(int x, int y, int r) {
		this.x = x;
		this.y = y;
		this.r = r;
	}

	/**
	 * The X position of this detected circle.
	 * @return x position of this detected circle
	 */
	public int getX(){
		return x;
	}

	/**
	 * The Y position of this detected circle.
	 * @return y position of this detected circle
	 */
	public int getY(){
		return y;
	}

	/**
	 * The radius of this detected circle.
	 * @return radius of this detected circle
	 */
	public int getR(){
		return r;
	}
}
