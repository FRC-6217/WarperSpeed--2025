// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.management.MemoryType;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;

public class Intake extends SubsystemBase {
  SparkMax intakeMotor = new SparkMax(RobotConstants.intakeMotorID, MotorType.kBrushless);
  SparkMaxConfig intakeConfig = new SparkMaxConfig();
  DigitalInput intakeBeamBrake = new DigitalInput(Constants.RobotConstants.intakeBeamBreak);
  String uniqueID = "Intake Speed:";
  /** Creates a new Intake. */
  public Intake() {

    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run


  }
  public void forward(){
    intakeMotor.set(Constants.RobotConstants.defaultIntakeSpeed);
  }
  public void backward(){
    intakeMotor.set(-Constants.RobotConstants.defaultIntakeSpeed);
  }
  public void stop(){
    intakeMotor.set(0);
  }

  public DigitalInput getIntakeBeamBrake(){
    return intakeBeamBrake;
  }
}
