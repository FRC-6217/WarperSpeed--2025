// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Hallsensor extends SubsystemBase {
  /** Creates a new Hallsensor. */

  public Hallsensor() {}

   DigitalInput firstBeamBreak = new DigitalInput(1);
    Debouncer debouncer = new Debouncer(0.15);
    boolean debouncedBeamBreak = false;

    public boolean get() {
        return !firstBeamBreak.get();
    }

    @Override
    public void periodic() { 
    debouncedBeamBreak = debouncer.calculate(get());
    SmartDashboard.putBoolean("Hall Sensor: ",debouncedBeamBreak);
    }

    public boolean getDebouncedBeamBreak() {
        return debouncedBeamBreak;
    }


}
