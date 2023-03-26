package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ClawPneumatics{

	DoubleSolenoid doubleSolenoid;
	
	Compressor pcmCompressor = new Compressor(Constants.Compressor_ID, PneumaticsModuleType.REVPH);
	private ArmSubsystem arm;
	public boolean isClawClosed = true;

    public ClawPneumatics(int forwardChannel, int reverseChannel, ArmSubsystem arm) {
        doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, forwardChannel, reverseChannel);
		doubleSolenoid.set(DoubleSolenoid.Value.kOff);
		this.arm = arm;
		// disableCompressor();
		enableCompressor();
    }

	public boolean isNotFull() {
		return pcmCompressor.getPressureSwitchValue(); // if not full, returns true
	}
	
	public void setHigh() { // we believe that setHigh is closing the claw
		
		doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
		isClawClosed = false;
		if(arm.smallArmMotorPosition < Constants.SmallArm_default){
			arm.smallArmMotorPosition += Constants.ARM_SPEED*4.5;
			arm.maintainPosition();
		}
	}
	
	public void setLow() { // opening the claw (maybe????)
		doubleSolenoid.set(DoubleSolenoid.Value.kForward);
		isClawClosed = true;
	}
	
	public void setOff() {
		doubleSolenoid.set(DoubleSolenoid.Value.kOff);
	}

	public void toggleCompressor() {
		if (pcmCompressor.isEnabled()) {
			disableCompressor();
		}
		else {
			enableCompressor();
		}
	}

	public void enableCompressor() {
		pcmCompressor.enableDigital();
	}

	public void disableCompressor() {
		pcmCompressor.disable();
	}
}