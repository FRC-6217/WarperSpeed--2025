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
  public double position;
  public double setpoint;
  public String level;
  boolean error = false;
  private String uniqueElevatorStringID = "Elevator Speed";
  /** Creates a new ElevatorCommand. */
  public ElevatorCommand(Elevator elevator, double position) {
    this.elevator = elevator;
    this.position = position;
    SmartDashboard.putNumber(uniqueElevatorStringID, 0);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(position == 0){
      setpoint = Constants.RobotConstants.L0Position;
    }else if(position == 1){
      setpoint = Constants.RobotConstants.L1Position; 
    }else if(position == 2){
      setpoint = Constants.RobotConstants.L2Position; 
    }else if(position == 3){
      setpoint = Constants.RobotConstants.L3Position; 
    }else if(position == 4){
      setpoint = Constants.RobotConstants.L4Position; 
    }else{
      error = true; 
    }

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
    return elevator.getSensor(position) || error;
  }
}
