// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.LimeLightSub;
import frc.robot.subsystems.SwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoLineUpReef extends Command {
  /** Creates a new AutoLineUpReef. */
  LimeLightSub frontLimelight = new LimeLightSub("reefLimelight", 0);
  PIDController translationPID;
  PIDController strafePID;
  PIDController rotationPID;

  public static enum TargetSide
  {
    Left,
    Right
  }
  TargetSide targetSide;

  SwerveDrivetrain swerveDrivetrain;

  double rotationSetpoint;
  double strafeSetpoint;
  double translationSetpoint;

  double rotationOutput;
  double strafeOutput;
  double translationOutput;
  
  public AutoLineUpReef(SwerveDrivetrain swerveDrivetrain, TargetSide targetSide) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.targetSide = targetSide;
    this.swerveDrivetrain = swerveDrivetrain;
    addRequirements(swerveDrivetrain);
    translationPID.setP(0);
    translationPID.setI(0);
    translationPID.setD(0);

    strafePID.setP(0);
    strafePID.setI(0);
    strafePID.setD(0);
    
    rotationPID.setP(0);
    rotationPID.setI(0);
    rotationPID.setD(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(targetSide == TargetSide.Left){
      translationSetpoint = Constants.AutoConstants.reefLeftTranslationSetpoint;
      strafeSetpoint = Constants.AutoConstants.reefLeftStrafeSetpoint;
      rotationSetpoint = Constants.AutoConstants.reefLeftRotationSetpoint;
    }else {
      translationSetpoint = Constants.AutoConstants.reefRightTranslationSetpoint;
      strafeSetpoint = Constants.AutoConstants.reefRightStrafeSetpoint;
      rotationSetpoint = Constants.AutoConstants.reefRightRotationSetpoint;
    }

    translationPID.setSetpoint(translationSetpoint);
    strafePID.setSetpoint(strafeSetpoint);
    rotationPID.setSetpoint(rotationSetpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    translationOutput = translationPID.calculate(frontLimelight.getArea());
    rotationOutput = rotationPID.calculate(frontLimelight.getX());
    strafeOutput = strafePID.calculate(frontLimelight.getX());

    swerveDrivetrain.relativeDrive(new Translation2d(translationOutput, strafeOutput), rotationOutput);
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return translationPID.atSetpoint() && strafePID.atSetpoint() && rotationPID.atSetpoint();
  }
}
