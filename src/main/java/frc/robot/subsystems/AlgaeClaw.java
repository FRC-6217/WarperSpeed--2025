// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class AlgaeClaw extends SubsystemBase {
  /** Creates a new AlgeClaw. */

  SparkMax leftAlgaeClawWheel = new SparkMax(RobotConstants.algeaClawLeftMotorID, MotorType.kBrushless);
  SparkMax rightAlgaeClawWheel = new SparkMax(RobotConstants.algeaClawRightMotorID, MotorType.kBrushless);

  public AlgaeClaw() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void setSpeed(double speed){
    leftAlgaeClawWheel.set(speed);
    rightAlgaeClawWheel.set(-speed);
  }
  public void stop(){
    leftAlgaeClawWheel.set(0);
    rightAlgaeClawWheel.set(0);
  }
}
