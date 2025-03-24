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
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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

  double[] levelToDistance = {RobotConstants.L0Position, RobotConstants.L1Position, RobotConstants.L2Position, RobotConstants.L3Position, RobotConstants.L4Position}; 

  Boolean[] levelToBooleans = new Boolean[EleLevel.TOTAL.ordinal()];
  

  public SparkMax leaderElevatorMotor = new SparkMax(RobotConstants.elevatorRightMotorID, MotorType.kBrushless);
  public SparkMax followerElevatorMotor = new SparkMax(RobotConstants.elevatorLeftMotorID, MotorType.kBrushless);
 
  SparkMaxConfig followerElevatorConfig = new SparkMaxConfig();
  SparkMaxConfig leaderElevatorConfig = new SparkMaxConfig();

  boolean L0sensor;
  DigitalInput L1sensor = new DigitalInput(Constants.RobotConstants.L1HallID);
  DigitalInput L2sensor = new DigitalInput(Constants.RobotConstants.L2HallID);
  DigitalInput L3sensor = new DigitalInput(Constants.RobotConstants.L3HallID);
  DigitalInput L4sensor = new DigitalInput(Constants.RobotConstants.L4HallID);
  SparkLimitSwitch topLimitSwitch;


  /** Creates a new Elevator. */
  public Elevator(SparkLimitSwitch topLimitSwitch) {
    this.topLimitSwitch = topLimitSwitch;

    leaderElevatorConfig.idleMode(IdleMode.kBrake);
    leaderElevatorConfig.inverted(true);
    leaderElevatorConfig.encoder.positionConversionFactor(RobotConstants.elevatorScalingFactor);
    leaderElevatorConfig.softLimit.apply(new SoftLimitConfig().forwardSoftLimit(Constants.RobotConstants.L4Position + .1));
    leaderElevatorMotor.configure(leaderElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

    followerElevatorConfig.idleMode(IdleMode.kBrake);
    followerElevatorConfig.follow(leaderElevatorMotor.getDeviceId(), false);
    followerElevatorMotor.configure(followerElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    L0sensor = leaderElevatorMotor.getReverseLimitSwitch().isPressed();  
  }

  

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("L4 limit Switch", !L4sensor.get());
    SmartDashboard.putBoolean("L0 limit Switch", L0sensor);
    SmartDashboard.putBoolean("L1 limit Switch", !L1sensor.get());
    SmartDashboard.putBoolean("L2 limit Switch", !L2sensor.get());
    SmartDashboard.putBoolean("L3 limit Switch", !L3sensor.get());

    if(L0sensor){
      setL0();
    }else if(!L1sensor.get()){
      setL1();
    }else if(!L2sensor.get()){
      setL2();
    }else if(!L3sensor.get()){
      setL3();
    }else if(!L4sensor.get()){
      setL4AndStop();
    }


    if((topLimitSwitch.isPressed() || !L4sensor.get()) && leaderElevatorMotor.get() > 0){
      leaderElevatorMotor.set(0);
    }
    
    levelToBooleans[EleLevel.L0.ordinal()] = L0sensor;
    levelToBooleans[EleLevel.L1.ordinal()] = !L1sensor.get();
    levelToBooleans[EleLevel.L2.ordinal()] = !L2sensor.get();
    levelToBooleans[EleLevel.L3.ordinal()] = !L3sensor.get();
    levelToBooleans[EleLevel.L4.ordinal()] = !L4sensor.get();


    SmartDashboard.putNumber("Elevator Position", getPosition());
    L0sensor = leaderElevatorMotor.getReverseLimitSwitch().isPressed();

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
    leaderElevatorMotor.getEncoder().setPosition(getDistanceFromLevel(level));
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


  public void setL4AndStop(){
    this.setL4();
    this.l4asLimit();
  }


public boolean getSignalOfLevel(EleLevel level){
 return levelToBooleans[level.ordinal()];
}



public double getPosition() {
  // TODO Auto-generated method stub
 return leaderElevatorMotor.getEncoder().getPosition();
}

public void l4asLimit(){
  if(leaderElevatorMotor.get() > 0){
    leaderElevatorMotor.set(0);
  }
}




}


