// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrain;

public class ResetGyro extends Command {
  /** Creates a new ResetGyro. */
  SwerveDrivetrain sDrivetrain;
  public ResetGyro(SwerveDrivetrain sDrivetrain) {
    this.sDrivetrain = sDrivetrain;
    addRequirements(sDrivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sDrivetrain.pigeon2.reset();
    sDrivetrain.sOdometry.resetRotation(sDrivetrain.getGyroRotation2d());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
