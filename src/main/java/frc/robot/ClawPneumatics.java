package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClawPneumatics{

	DoubleSolenoid doubleSolenoid;
	
	Compressor pcmCompressor = new Compressor(Constants.Compressor_ID, PneumaticsModuleType.CTREPCM);

    public ClawPneumatics(int forwardChannel, int reverseChannel) {
        doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, forwardChannel, reverseChannel);
		doubleSolenoid.set(DoubleSolenoid.Value.kOff);
    }

	public double findPressure() {
		return pcmCompressor.getPressure();
	}
	
	public void setHigh() {
		doubleSolenoid.set(DoubleSolenoid.Value.kForward);
	}
	
	public void setLow() {
		doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
	}
	
	public void setOff() {
		doubleSolenoid.set(DoubleSolenoid.Value.kOff);
	}

	public void enableCompressor() {
		pcmCompressor.enableDigital();
	}

	public void disableCompressor() {
		pcmCompressor.disable();
	}
}