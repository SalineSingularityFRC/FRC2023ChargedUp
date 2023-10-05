package frc.robot;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANdleSystem extends SubsystemBase {
  private CANdle m_candle = new CANdle(1, Constants.Canivore.CANIVORE);

  public CANdleSystem() {
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = false;
    configAll.disableWhenLOS = false;
    configAll.brightnessScalar = 1;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    m_candle.configAllSettings(configAll, 100);
    m_candle.setLEDs(0, 0, 0);

    m_candle.modulateVBatOutput(0);
  }

  public void turnOn() {
    m_candle.modulateVBatOutput(1);
  }

  public void turnOff() {
    m_candle.modulateVBatOutput(0);
  }
}
