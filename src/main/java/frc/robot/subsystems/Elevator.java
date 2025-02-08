// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Elevator extends SubsystemBase {

  TalonFX leader = new TalonFX(RobotConstants.elevatorLeftMotorID);
  TalonFX follower = new TalonFX(RobotConstants.elevatorRightMotorID);

  /** Creates a new Elevator. */
  public Elevator() {
    Follower myFollower = new Follower(leader.getDeviceID(), true);
    follower.setControl(myFollower);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed){
    leader.set(speed);
  }
  public void stop(){
    leader.set(0);
  }
}
