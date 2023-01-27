package frc.robot.input;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * IGamepad implementation for XBox-like gamepads.
 * <p>
 * Currently known to work with:
 * <ul>
 * <li>Xbox 360 wired controller</li>
 * <li>Logitech F310</li>
 * </ul>
 *
 * @author Jacob Bublitz
 * @since 1.0
 */
public final class XboxController extends Controller {
	private final Joystick joystick;

	private final Trigger aButton;
	private final Trigger bButton;
	private final Trigger xButton;
	private final Trigger yButton;
	private final Trigger leftBumperButton;
	private final Trigger rightBumperButton;
	private final Trigger backButton;
	private final Trigger startButton;
	private final Trigger leftStickButton;
	private final Trigger rightStickButton;

	private final Axis leftTriggerAxis;
	private final Axis leftXAxis;
	private final Axis leftYAxis;
	private final Axis rightTriggerAxis;
	private final Axis rightXAxis;
	private final Axis rightYAxis;

	private final DPadButton[] dpadButtons;

	/**
	 * @param port The port the controller is on
	 */
	public XboxController(int port) {
		joystick = new Joystick(port);

		aButton = new JoystickButton(joystick, 1);
		bButton = new JoystickButton(joystick, 2);
		xButton = new JoystickButton(joystick, 3);
		yButton = new JoystickButton(joystick, 4);
		leftBumperButton = new JoystickButton(joystick, 5);
		rightBumperButton = new JoystickButton(joystick, 6);
		backButton = new JoystickButton(joystick, 7);
		startButton = new JoystickButton(joystick, 8);
		leftStickButton = new JoystickButton(joystick, 9);
		rightStickButton = new JoystickButton(joystick, 10);

		leftTriggerAxis = new JoystickAxis(joystick, 2);
		leftXAxis = new JoystickAxis(joystick, 0);
		leftYAxis = new JoystickAxis(joystick, 1);
		leftYAxis.setInverted(true);
		rightTriggerAxis = new JoystickAxis(joystick, 3);
		rightXAxis = new JoystickAxis(joystick, 4);
		rightYAxis = new JoystickAxis(joystick, 5);
		rightYAxis.setInverted(true);

		dpadButtons = new DPadButton[DPadButton.Direction.values().length];

		for (DPadButton.Direction dir : DPadButton.Direction.values()) {
			dpadButtons[dir.ordinal()] = new DPadButton(joystick, dir);
		}
	}

	@Override
	public Axis getLeftTriggerAxis() {
		return leftTriggerAxis;
	}

	@Override
	public Axis getLeftXAxis() {
		return leftXAxis;
	}

	@Override
	public Axis getLeftYAxis() {
		return leftYAxis;
	}

	@Override
	public Axis getRightTriggerAxis() {
		return rightTriggerAxis;
	}

	@Override
	public Axis getRightXAxis() {
		return rightXAxis;
	}

	@Override
	public Axis getRightYAxis() {
		return rightYAxis;
	}

	@Override
	public Trigger getAButton() {
		return aButton;
	}

	@Override
	public Trigger getBButton() {
		return bButton;
	}

	@Override
	public Trigger getXButton() {
		return xButton;
	}

	@Override
	public Trigger getYButton() {
		return yButton;
	}

	@Override
	public Trigger getLeftBumperButton() {
		return leftBumperButton;
	}

	@Override
	public Trigger getRightBumperButton() {
		return rightBumperButton;
	}

	@Override
	public Trigger getBackButton() {
		return backButton;
	}

	@Override
	public Trigger getStartButton() {
		return startButton;
	}

	@Override
	public Trigger getLeftJoystickButton() {
		return leftStickButton;
	}

	@Override
	public Trigger getRightJoystickButton() {
		return rightStickButton;
	}

	@Override
	public Trigger getDPadButton(DPadButton.Direction direction) {
		return dpadButtons[direction.ordinal()];
	}

	public Joystick getRawJoystick() {
		return joystick;
	}
}
