// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.LimeLightSub;
import frc.robot.subsystems.SwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoLineUpReef extends Command {
  /** Creates a new AutoLineUpReef. */
  LimeLightSub frontLimelight = new LimeLightSub("limelight-reef", 0);
  PIDController translationPID = new PIDController(0, 0, 0);
  PIDController strafePID = new PIDController(0, 0, 0);
  PIDController rotationPID = new PIDController(0, 0, 0);

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
  double pTranslation = 0.2;
  double iTranslation  = 0;
  double pStrafe = 0.016;
  double iStrafe = 0.001;
  String pTranslationString = "P Translation for Auto Aline";
  String iTranslationString = "I Translation for Auto Aline";
  String pStrafeString = "P Strafe for Auto Aline";
  String iStrafeString = "I Strafe for Auto Aline";

  
  public AutoLineUpReef(SwerveDrivetrain swerveDrivetrain, TargetSide targetSide) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.targetSide = targetSide;
    this.swerveDrivetrain = swerveDrivetrain;
    addRequirements(swerveDrivetrain);
    translationPID.setP(pTranslation);
    translationPID.setI(iTranslation);
    translationPID.setD(0);
    translationPID.setTolerance(2);

    SmartDashboard.putNumber(pTranslationString, pTranslation);
    SmartDashboard.putNumber(iTranslationString, iTranslation);
    SmartDashboard.putNumber(pStrafeString, pStrafe);
    SmartDashboard.putNumber(iStrafeString, iStrafe);

    strafePID.setP(pStrafe);
    strafePID.setI(iStrafe);
    strafePID.setD(0);
    strafePID.setTolerance(2);
    
    // rotationPID.setP(0);
    // rotationPID.setI(0);
    // rotationPID.setD(0);
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
    if (SmartDashboard.getNumber(pTranslationString, 0) != pTranslation){
      pTranslation = SmartDashboard.getNumber(pTranslationString, 0);
      translationPID.setP(pTranslation);
    }
    if (SmartDashboard.getNumber(iTranslationString, 0) != iTranslation){
      iTranslation = SmartDashboard.getNumber(iTranslationString, 0);
      translationPID.setI(iTranslation);
    }
    if (SmartDashboard.getNumber(pStrafeString, 0) != pStrafe){
      pStrafe = SmartDashboard.getNumber(pStrafeString, 0);
      strafePID.setP(pStrafe);
    }
    if (SmartDashboard.getNumber(iStrafeString, 0) != iStrafe){
      iStrafe = SmartDashboard.getNumber(iStrafeString, 0);
      strafePID.setI(iStrafe);
    }
    translationOutput = translationPID.calculate(frontLimelight.getArea());
    SmartDashboard.putNumber("Front Limelight Apriltag Reef", frontLimelight.getArea());
    SmartDashboard.putNumber("Front Limelight Apriltag Reef X", frontLimelight.getX());

    rotationOutput = rotationPID.calculate(frontLimelight.getX());
    strafeOutput = strafePID.calculate(frontLimelight.getX());

    translationOutput = MathUtil.clamp(translationOutput, -0.5, 0.5);
    strafeOutput = MathUtil.clamp(strafeOutput, -0.5, 0.5);

    swerveDrivetrain.relativeDrive(new Translation2d(translationOutput, strafeOutput), 0);
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return translationPID.atSetpoint() && strafePID.atSetpoint();
  }
}
