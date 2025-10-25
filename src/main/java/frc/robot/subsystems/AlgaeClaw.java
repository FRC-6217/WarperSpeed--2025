// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;

public class AlgaeClaw extends SubsystemBase {
  /** Creates a new AlgeClaw. */

  SparkMax algaePositionMotor = new SparkMax(Constants.RobotConstants.algaePositionID, MotorType.kBrushless);
  SparkMaxConfig algaeConfig = new SparkMaxConfig();
  double speed = 0;
  String algaeName = "Algae Speed";


  public static enum AlgaeState{
    Idle,
    Intake,
    Place,
  }

  public AlgaeState algaeState;
  public double position;

  public AlgaeClaw() {
    SmartDashboard.putNumber(algaeName, 0);
    algaeState = AlgaeState.Idle;
    algaeConfig.idleMode(IdleMode.kBrake);
    algaeConfig.encoder.positionConversionFactor(1);
    algaePositionMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    position = algaePositionMotor.getEncoder().getPosition();
    SmartDashboard.putNumber("Algae Mortor Postition ", position);

    if(algaeState == AlgaeState.Idle){
      setSpeed(0);
    }else if(algaeState == AlgaeState.Intake){
      if (position > 0.3) {
        setSpeed(-SmartDashboard.getNumber(algaeName, 0));
      }else{
        setSpeed(0);
      }
    }else if(algaeState == AlgaeState.Place){
      if(Math.abs(position - 7) < 0.2){
        setSpeed(SmartDashboard.getNumber(algaeName, 0));
      }else{
        setSpeed(0);
      }
    }


  }

  public void setState(AlgaeState algaeState){
    this.algaeState = algaeState;
  }

  public void setIdleState(){
    setState(AlgaeState.Idle);
  }
  public void setIntakeState(){
    setState(AlgaeState.Intake);
  }
  public void setPlaceState(){
    setState(AlgaeState.Place);
  }
  // public void functino that sets algaeState to IDLE, then one for INTAKE and oen for PLACE

  public void setSpeed(double speed){
    algaePositionMotor.set(speed);
  }
  public void stop(){
    algaePositionMotor.set(0);
  }
}
