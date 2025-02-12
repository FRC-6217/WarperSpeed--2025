// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Placer extends SubsystemBase {
  SparkMax placer = new SparkMax(RobotConstants.placerMoterID, MotorType.kBrushless);
  /** Creates a new Placer. */
  public Placer() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setSpeed(double speed){
    placer.set(speed);
  }
  public void stop(){
    placer.set(0);
  }
}
