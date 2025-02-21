// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Elevator extends SubsystemBase {

  SparkMax followerElevatorMotor = new SparkMax(RobotConstants.elevatorLeftMotorID, MotorType.kBrushless);
  SparkMax leaderElevatorMotor = new SparkMax(RobotConstants.elevatorRightMotorID, MotorType.kBrushless);
  SparkMaxConfig followerElevatorConfig = new SparkMaxConfig();
  SparkMaxConfig leaderElevatorConfig = new SparkMaxConfig();

  /** Creates a new Elevator. */
  public Elevator() {
     leaderElevatorConfig.idleMode(IdleMode.kBrake);
    leaderElevatorMotor.configure(leaderElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    followerElevatorConfig.idleMode(IdleMode.kBrake);
    followerElevatorConfig.follow(leaderElevatorMotor.getDeviceId(), true);
    followerElevatorMotor.configure(followerElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed){
    leaderElevatorMotor.set(speed);
  }
  public void stop(){
    leaderElevatorMotor.set(0);
  }
}
