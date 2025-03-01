// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.EleLevel;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommand extends Command {

  public Elevator elevator;
  public double setpoint;
  public double speed;
  public RobotContainer robotContainer;
  public Elevator.EleLevel level;
  private String uniqueElevatorStringID = "Elevator Speed";
  /** Creates a new ElevatorCommand. */
  public ElevatorCommand(Elevator elevator, Elevator.EleLevel level, RobotContainer robotContainer) {
    this.elevator = elevator;
    this.level = level;
    this.robotContainer = robotContainer;
    SmartDashboard.putNumber(uniqueElevatorStringID, 0);
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.err.println(level + " Starting");
    setpoint = elevator.getDistanceFromLevel(level);
    if(setpoint < elevator.getPosition()){
      speed = -.7;
    }else if (setpoint > elevator.getPosition()){
      if(robotContainer.intakeBeamBrakeTrigger.getAsBoolean()){
        speed = 0;
      }else{
        speed = .7;
      }
    }else{
      speed = 0;
    }
  }
   

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setSpeed(speed);
    System.out.println(elevator.getSignalOfLevel(level));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
    System.out.println(elevator.getSignalOfLevel(level));
    System.out.println(level + " Ending");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  
    
    return elevator.getSignalOfLevel(level);
   
  }
}
