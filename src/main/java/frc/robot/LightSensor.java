package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;

public class LightSensor {
    
    private AnalogInput analog;
    private AnalogTrigger trigger;

    public LightSensor(){
        trigger = new AnalogTrigger(3);
        analog = new AnalogInput(3);
        analog.setAverageBits(2);
        trigger.setLimitsVoltage(0.2, 1);
    }
    public boolean isSensed(){
        return !trigger.getTriggerState();
    }
}
