// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.management.MemoryType;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Intake extends SubsystemBase {
  SparkMax intakeMotor = new SparkMax(RobotConstants.intakeMotorID, MotorType.kBrushless);
  /** Creates a new Intake. */
  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setSpeed(double speed){
    intakeMotor.set(speed);
  }
  public void stop(){
    intakeMotor.set(0);
  }
}
