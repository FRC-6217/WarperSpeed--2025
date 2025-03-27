// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;

public class AlgaeClaw extends SubsystemBase {
  /** Creates a new AlgeClaw. */

  SparkMax algaePositionMotor = new SparkMax(Constants.RobotConstants.algaePositionID, MotorType.kBrushless);
  SparkMaxConfig algaeConfig = new SparkMaxConfig();



  public static enum AlgaeState{
    Idle,
    Intake,
    Place,
  }

  public static AlgaeState algaeState;
  public double position;

  public AlgaeClaw() {
    algaeState = AlgaeState.Idle;
    algaeConfig.idleMode(IdleMode.kBrake);
    algaeConfig.encoder.positionConversionFactor(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    position = algaePositionMotor.getEncoder().getPosition();

    if(algaeState == AlgaeState.Idle){
      setSpeed(0);
    }else if(algaeState == AlgaeState.Intake){
      if (position > 0) {
        setSpeed(-.1);
      }else{
        setSpeed(0);
      }
    }else if(algaeState == AlgaeState.Place){
      if(position < 1){
        setSpeed(.1);
      }else{
        setSpeed(0);
      }
    }


  }

  public void setState(AlgaeState algaeState){

  }
  public void setSpeed(double speed){
    algaePositionMotor.set(speed);
  }
  public void stop(){
    algaePositionMotor.set(0);
  }
}
