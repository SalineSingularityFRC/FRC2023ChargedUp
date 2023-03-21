package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;

public class LightSensor {
    
    private AnalogInput analog;
    private AnalogTrigger trigger;

    public LightSensor(){
        analog = new AnalogInput(3);
        trigger = new AnalogTrigger(analog);
        analog.setAverageBits(2);
        trigger.setLimitsVoltage(0.5, 1);
    }
    public boolean isSensed(){
        return trigger.getTriggerState();
    }
    public double volts() {
        return analog.getVoltage();
    }
}
