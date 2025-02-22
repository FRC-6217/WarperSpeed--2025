// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.config.RobotConfig;
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

  public static enum EleLevel
  {
    L0,
    L1,
    L2,
    L3,
    L4,
    TOTAL,
  }

  double[] levelToDistance = {RobotConstants.L0Position, RobotConstants.L1Position, RobotConstants.L2Position, RobotConstants.L3Position, RobotConstants.L4Position }; 

  Trigger[] levelToTrigger = new Trigger[EleLevel.TOTAL.ordinal()];
  


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
    leaderElevatorConfig.inverted(true);
    leaderElevatorConfig.encoder.positionConversionFactor(RobotConstants.elevatorScalingFactor);
    leaderElevatorMotor.configure(leaderElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    followerElevatorConfig.idleMode(IdleMode.kBrake);
    followerElevatorConfig.follow(leaderElevatorMotor.getDeviceId(), true);
    followerElevatorMotor.configure(followerElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    L0sensor = leaderElevatorMotor.getReverseLimitSwitch().isPressed();
    L4sensor = leaderElevatorMotor.getForwardLimitSwitch().isPressed();

    levelToTrigger[EleLevel.L0.ordinal()] = new Trigger(() -> L0sensor).onTrue(Commands.runOnce(this::setL0, this));
    levelToTrigger[EleLevel.L1.ordinal()] = new Trigger(() -> L1sensor.get()).onTrue(Commands.runOnce(this::setL1, this));
    levelToTrigger[EleLevel.L2.ordinal()] = new Trigger(() -> L2sensor.get()).onTrue(Commands.runOnce(this::setL2, this));
    levelToTrigger[EleLevel.L3.ordinal()] = new Trigger(() -> L3sensor.get()).onTrue(Commands.runOnce(this::setL3, this));
    levelToTrigger[EleLevel.L4.ordinal()] = new Trigger(() -> L4sensor).onTrue(Commands.runOnce(this::setL4, this));
    
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

  public void setPosition(EleLevel level)
  {
    leaderElevatorMotor.getAlternateEncoder().setPosition(getDistanceFromLevel(level));
  }

  public double getDistanceFromLevel(EleLevel level) {
    return levelToDistance[level.ordinal()];
  }

  public void moveUp(){
    leaderElevatorMotor.set(0.2);
  }
  public void moveDown(){
    leaderElevatorMotor.set(-0.2);
  }


  public void setL0(){
    this.setPosition(EleLevel.L0);
  }
  public void setL1(){
    this.setPosition(EleLevel.L1);
  }
  public void setL2(){
    this.setPosition(EleLevel.L2);
  }
  public void setL3(){
    this.setPosition(EleLevel.L3);
  }
  public void setL4(){
    this.setPosition(EleLevel.L4);
  }




public boolean getSignalOfLevel(EleLevel level){
 return levelToTrigger[level.ordinal()].getAsBoolean();
}



public double getPosition() {
  // TODO Auto-generated method stub
 return leaderElevatorMotor.getEncoder().getPosition();
}





}


