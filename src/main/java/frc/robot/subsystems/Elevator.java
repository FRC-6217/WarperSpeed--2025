// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.core.filter.FilteringGeneratorDelegate;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;

public class Elevator extends SubsystemBase {

  SparkMax followerElevatorMotor = new SparkMax(RobotConstants.elevatorLeftMotorID, MotorType.kBrushless);
  SparkMax leaderElevatorMotor = new SparkMax(RobotConstants.elevatorRightMotorID, MotorType.kBrushless);
  SparkMaxConfig followerElevatorConfig = new SparkMaxConfig();
  SparkMaxConfig leaderElevatorConfig = new SparkMaxConfig();
  DigitalInput L1sensor = new DigitalInput(Constants.RobotConstants.L1HallID);
  DigitalInput L2sensor = new DigitalInput(Constants.RobotConstants.L2HallID);
  DigitalInput L3sensor = new DigitalInput(Constants.RobotConstants.L3HallID);
  boolean L0sensor;
  Boolean L4sensor;


  /** Creates a new Elevator. */
  public Elevator() {
     leaderElevatorConfig.idleMode(IdleMode.kBrake);
    leaderElevatorMotor.configure(leaderElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    followerElevatorConfig.idleMode(IdleMode.kBrake);
    followerElevatorConfig.follow(leaderElevatorMotor.getDeviceId(), true);
    followerElevatorMotor.configure(followerElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    L0sensor = followerElevatorMotor.getReverseLimitSwitch().isPressed();
    L4sensor = followerElevatorMotor.getForwardLimitSwitch().isPressed();

   
    new Trigger(() -> L0sensor).onTrue(Commands.runOnce(this::setL0, this));
    new Trigger(() -> L1sensor.get()).onTrue(Commands.runOnce(this::setL1, this));
    new Trigger(() -> L2sensor.get()).onTrue(Commands.runOnce(this::setL2, this));
    new Trigger(() -> L3sensor.get()).onTrue(Commands.runOnce(this::setL3, this));
    new Trigger(() -> L4sensor).onTrue(Commands.runOnce(this::setL4, this));

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

  public void setL0() {
    // TODO Auto-generated method stub
    leaderElevatorMotor.getAlternateEncoder().setPosition(Constants.RobotConstants.L0Position);
  }

public void setL1() {
    // TODO Auto-generated method stub
    leaderElevatorMotor.getAlternateEncoder().setPosition(Constants.RobotConstants.L1Position);
}

public void setL2() {
    // TODO Auto-generated method stub
    leaderElevatorMotor.getAlternateEncoder().setPosition(Constants.RobotConstants.L2Position);
}

public void setL3() {
    // TODO Auto-generated method stub
    leaderElevatorMotor.getAlternateEncoder().setPosition(Constants.RobotConstants.L3Position);
}

public void setL4() {
    // TODO Auto-generated method stub
    leaderElevatorMotor.getAlternateEncoder().setPosition(Constants.RobotConstants.L4Position);
}

public boolean getSensor(double value){
  if(value == 0){
    return L0sensor;
  }else if(value == 1){
    return L1sensor.get();
  }else if(value == 2){
    return L2sensor.get();
  }else if(value == 3){
    return L3sensor.get();
  }else if(value == 4){
    return L4sensor;
  }else{
    return true;
  }
}



public double getPosition() {
  // TODO Auto-generated method stub
 return leaderElevatorMotor.getAlternateEncoder().getPosition();
}
}
