// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveModule.Constants.encoderType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 1;
    public static final int kOperatorControllerPort = 2;
    public static final int kLeftBackButton = 7;
    public static final int kRightBackButton = 8;
    public static final int leftTriggerAxis = 2;
    public static final int rightTriggerAxis = 3;
    public static final double kFastInc = 0.1;
    public static final double kSlowInc = 0.01;
    public static final double kDefaultFast = 1;
    public static final double kDefaultSlow = 0.3;

  }

  public static class AutoConstants{

   }

  public static class SemiAutoConstants{}


  public static class RobotConstants{
    public static final double trackWidth = Units.inchesToMeters(24.5);
    public static final double trackLength = Units.inchesToMeters(24.5);
    public static final double driveGearRatio = 6.12;
    public static final double steerGearRatio = 1.2;
    public static final double wheelDiameter = Units.inchesToMeters(4);
    public static final double driveSlewTimeInSecond = 0.06;
    public static final double driveMaxVelo = (6000/60/steerGearRatio)*(wheelDiameter)*Math.PI;
    public static final double rotationMaxAngleVelo = 1.5*Math.PI*driveMaxVelo;
    public static final int steerMotorCurrentLimit = 40;
    public static final int driveMotorCurrentLimit = 45;
    public static final double driveWheelGearRatio = 21.4285714;

    //sparkmaxID
    public static final int algeaClawLeftMotorID = 33;
    public static final int algeaClawRightMotorID = 34;
    public static final int elevatorLeftMotorID = 15;
    public static final int elevatorRightMotorID = 16;
    public static final int climberMotorID = 42;  
    public static final int intakeMotorID = 43;
    public static final int placerMoterID = 44;

    //default speeds
    public static final int defaultAlgaeClawSpeed = 0;
    public static final int defaultClimberSpeed = 0;
    public static final int defaultIntakeSpeed = 0;
    public static final int defaultPlacerSpeed = 0;

    //Drive and steer PID
    public static final double drivePIDp = 0.55;
    public static final double drivePIDi = 0.8;
    public static final double drivePIDd = 0;
    public static final double steerPIDp = 3;
    public static final double steerPIDi = 0.6;
    public static final double steerPIDd = 0;

    public static final boolean autoSyncEncoder = false;
    public static final int autoSyncTimer = 10;
    public static final double syncThreshold = 1;
    //Random numberdecided make actually number that the angle velo max actually is. 
    // Competition Robot

    //bevel gear on wheel goes rig
    public static final SwerveModule.Constants frontLeft = new SwerveModule.Constants(0, 3, 2, 21,-0.040283, "Front Left", encoderType.CAN, "CTRSwerve");
    public static final SwerveModule.Constants frontRight = new SwerveModule.Constants(1, 18, 19, 22, -0.166748, "Front Right", encoderType.CAN,"CTRSwerve");
    public static final SwerveModule.Constants backLeft = new SwerveModule.Constants(2, 8, 9, 24,0.067627, "Back Left", encoderType.CAN, "CTRSwerve");
    public static final SwerveModule.Constants backRight = new SwerveModule.Constants(3, 10, 11, 23,.400146, "Back Right", encoderType.CAN,"CTRSwerve");
    public static final double laserNoteThresholdInches = 7.3;
    public static final double indexerStartOffset = .0887;
    public static final double noteDetectorThreshold = 0.000326;
    public static final double feedforward = 0.011;
    public static final String Limelight4Name = "Limelight 4";
    public static final int L1HallID = 0;
    public static final int L2HallID = 1;
    public static final int L3HallID = 2;
    public static final double L0Position = 0;
    public static final double L1Position = 1;
    public static final double L2Position = 2;
    public static final double L3Position = 3;
    public static final double L4Position = 4;
  }


}
                                       
