// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Climber extends SubsystemBase {

  TalonFX climberMotor = new TalonFX(RobotConstants.climberMotorID);

  /** Creates a new Climber. */
  public Climber() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setSpeed(double speed){
    climberMotor.set(speed);
  }
  public void stop(){
    climberMotor.set(0);
  }
}
