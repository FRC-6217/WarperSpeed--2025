// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.sim.SparkMaxAlternateEncoderSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.EleLevel;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDElevatorCommand extends Command {
  /** Creates a new PIDElevatorCommand. */
  public Elevator elevator;
  public RobotContainer robotContainer;
  public double setpoint;
  
  public PIDElevatorCommand(Elevator elevator, double setpoint) {
    this.elevator = elevator;
    this.setpoint = setpoint;
    
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   elevator.setSetpoint(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
