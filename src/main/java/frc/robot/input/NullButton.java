package frc.robot.input;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class NullButton extends Trigger {
	private boolean value;

	public NullButton() {
		this(false);
	}

	public NullButton(boolean initialValue) {
		value = initialValue;
	}

	public boolean get() {
		return value;
	}

	public void set(boolean value) {
		this.value = value;
	}
}
