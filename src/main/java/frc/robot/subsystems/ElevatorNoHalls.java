// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;

public class ElevatorNoHalls extends SubsystemBase {
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

  /** Creates a new ElevatorNoHalls. */
  public SparkMax followerElevatorMotor = new SparkMax(RobotConstants.elevatorLeftMotorID, MotorType.kBrushless);
  public SparkMax leaderElevatorMotor = new SparkMax(RobotConstants.elevatorRightMotorID, MotorType.kBrushless);
  SparkMaxConfig followerElevatorConfig = new SparkMaxConfig();
  SparkMaxConfig leaderElevatorConfig = new SparkMaxConfig();
  boolean L0sensor;
  public ElevatorNoHalls() {
    leaderElevatorConfig.idleMode(IdleMode.kBrake);
    leaderElevatorConfig.inverted(true);
    leaderElevatorConfig.encoder.positionConversionFactor(RobotConstants.elevatorScalingFactor);
    leaderElevatorConfig.softLimit.apply(new SoftLimitConfig().forwardSoftLimit(Constants.RobotConstants.L4Position+.1));
    leaderElevatorMotor.configure(leaderElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

    followerElevatorConfig.idleMode(IdleMode.kBrake);
    followerElevatorConfig.follow(leaderElevatorMotor.getDeviceId(), true);
    followerElevatorMotor.configure(followerElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    L0sensor = leaderElevatorMotor.getReverseLimitSwitch().isPressed();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("L0 limit Switch", L0sensor);
    L0sensor = leaderElevatorMotor.getReverseLimitSwitch().isPressed();

    // This method will be called once per scheduler run
  }
  public void setSpeed(double speed){
    leaderElevatorMotor.set(speed);
  }
  public void stop(){
    leaderElevatorMotor.set(0);
  }
  public void moveUp(){
    leaderElevatorMotor.set(0.4);
  }
  public void moveDown(){
    leaderElevatorMotor.set(-0.4);
  }
  public double getDistanceFromLevel(EleLevel level) {
    return levelToDistance[level.ordinal()];
  }
  public double getPosition() {
    // TODO Auto-generated method stub
   return leaderElevatorMotor.getEncoder().getPosition();
  }
  public boolean getSignalOfLevel(EleLevel e){
    return leaderElevatorMotor.getReverseLimitSwitch().isPressed();
  }
}
