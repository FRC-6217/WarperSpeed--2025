// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoLeaveLine extends Command {
  private SwerveDrivetrain swerveDrivetrain;
  
    /** Creates a new AutoLeaveLine. */
    public AutoLeaveLine(SwerveDrivetrain swerveDrivetrain) {
      this.swerveDrivetrain = swerveDrivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  swerveDrivetrain.setPigeonAngle(180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDrivetrain.drive(new Translation2d(-1, 0), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrivetrain.drive(new Translation2d(0,0), 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swerveDrivetrain.sOdometry.getEstimatedPosition().getX() <= -1.1;
  }
}
