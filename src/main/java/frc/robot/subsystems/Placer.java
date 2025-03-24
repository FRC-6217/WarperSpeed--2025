// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;

public class Placer extends SubsystemBase {
  SparkMax placer = new SparkMax(RobotConstants.placerMoterID, MotorType.kBrushless);
  SparkMaxConfig placerConfig = new SparkMaxConfig();
  String uniqueID = "Placer Speed:";


  /** Creates a new Placer. */
  public Placer() {
    SmartDashboard.putNumber(uniqueID, 0);

    placerConfig.limitSwitch.forwardLimitSwitchEnabled(false);
    placerConfig.limitSwitch.reverseLimitSwitchEnabled(false);

    placer.configure(placerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);     
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("placer beambreak", getPlacerBeamBreak().isPressed());
    SmartDashboard.putBoolean("Elevator Top Limit", getElevatorLimitSwitch().isPressed());
    // This method will be called once per scheduler run
  }
  public void forward(){
    placer.set(-Constants.RobotConstants.defaultPlacerSpeed);
  }
  public void algaePlace(){
    placer.set(-1);
  }
  public void backward(){
    placer.set(Constants.RobotConstants.defaultPlacerSpeed);
  }
  public void stop(){
    placer.set(0);
  }

  public SparkLimitSwitch getElevatorLimitSwitch() {

    return placer.getReverseLimitSwitch();

  }

  public SparkLimitSwitch getPlacerBeamBreak(){
    return placer.getForwardLimitSwitch();
  }
}
