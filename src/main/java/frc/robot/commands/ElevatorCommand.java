// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommand extends Command {

  public Elevator elevator;
  public double setpoint;
  public Elevator.EleLevel level;
  private String uniqueElevatorStringID = "Elevator Speed";
  /** Creates a new ElevatorCommand. */
  public ElevatorCommand(Elevator elevator, Elevator.EleLevel level) {
    this.elevator = elevator;
    this.level = level;
    SmartDashboard.putNumber(uniqueElevatorStringID, 0);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setpoint = elevator.getDistanceFromLevel(level);
  }
   

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(setpoint < elevator.getPosition()){
      elevator.setSpeed(.2);
    }else if (setpoint > elevator.getPosition()){
      elevator.setSpeed(-0.2);
    }else{
      elevator.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.getSignalOfLevel(level);
  }
}
