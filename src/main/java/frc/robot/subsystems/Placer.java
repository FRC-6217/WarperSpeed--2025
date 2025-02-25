// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Placer extends SubsystemBase {
  SparkMax placer = new SparkMax(RobotConstants.placerMoterID, MotorType.kBrushless);
  SparkMaxConfig placerConfig = new SparkMaxConfig();
  String uniqueID = "Placer Speed:";

  /** Creates a new Placer. */
  public Placer() {
    SmartDashboard.putNumber(uniqueID, 0);

    placerConfig.limitSwitch.forwardLimitSwitchEnabled(false);

      
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void forward(){
    placer.set(SmartDashboard.getNumber(uniqueID, 0));
  }
  public void backward(){
    placer.set(-SmartDashboard.getNumber(uniqueID, 0));
  }
  public void stop(){
    placer.set(0);
  }
}
