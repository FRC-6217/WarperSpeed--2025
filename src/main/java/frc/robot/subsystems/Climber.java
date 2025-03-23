// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Climber extends SubsystemBase {

  SparkMax climberMotor = new SparkMax(RobotConstants.climberMotorID, MotorType.kBrushless);
  SparkMaxConfig climberConfig = new SparkMaxConfig();

  /** Creates a new Climber. */
  public Climber() {
    climberConfig.idleMode(IdleMode.kBrake);
    climberConfig.inverted(true);
    climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SmartDashboard.putNumber("climber speed",.2);

  }

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

  public void forward() {
    climberMotor.set(1);
  }
  public void reverse(){
    climberMotor.set(-1);
  }
}
