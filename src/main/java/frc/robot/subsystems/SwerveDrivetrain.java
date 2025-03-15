// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;


public class SwerveDrivetrain extends SubsystemBase {
  /** Creates a new SwerveDrivetrain. */

  public final SwerveModule frontRightModule;// = new SwerveModule(Constants.RobotConstants.frontRight);
  public final SwerveModule backLeftModule;// = new SwerveModule(Constants.RobotConstants.backLeft);
  public final  SwerveModule backRightModule;// = new SwerveModule(Constants.RobotConstants.backRight);
  public final SwerveModule frontLeftModule;// = new SwerveModule(Constants.RobotConstants.frontLeft);

  // public final Pigeon2 pigeon2 = new Pigeon2(50,"CTRSwerve");
   public final Pigeon2 pigeon2 = new Pigeon2(50, "SwerveCAN");
  public SwerveDrivePoseEstimator sOdometry;
  public SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  public SwerveDriveKinematics sKinematics;
  public SwerveModule[] modules;
  public ChassisSpeeds cSpeeds;
  public RobotConfig robotConfig;

  public enum USER_CONTROLLER {
    JOYSTICK,
    XBOX
  }
  public CommandJoystick cj;
  public CommandXboxController cx;
  public USER_CONTROLLER controller = USER_CONTROLLER.JOYSTICK;
  public AllianceSelector allianceSelector;
  public boolean isAbsolute = true;

  public Governor governor = new Governor();
 
  public SwerveDrivetrain(CommandXboxController cx) {

    this.cx = cx;

    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    SmartDashboard.putNumber("Auto xPID P", 0.1);
    SmartDashboard.putNumber("Auto yPID P", 0.1);
    SmartDashboard.putNumber("Auto rotationPID P", 0);
    
    

    frontRightModule = new SwerveModule(Constants.RobotConstants.frontRight);
    backLeftModule = new SwerveModule(Constants.RobotConstants.backLeft);
    backRightModule = new SwerveModule(Constants.RobotConstants.backRight);
    frontLeftModule = new SwerveModule(Constants.RobotConstants.frontLeft);

    
    allianceSelector = new AllianceSelector();

    pigeon2.reset();
    
    cSpeeds = new ChassisSpeeds(0, 0, 0);
    sKinematics = new SwerveDriveKinematics(new Translation2d(Constants.RobotConstants.trackWidth/2, Constants.RobotConstants.trackLength/2),
                                            new Translation2d(Constants.RobotConstants.trackWidth/2, -Constants.RobotConstants.trackLength/2),
                                            new Translation2d(-Constants.RobotConstants.trackWidth/2, Constants.RobotConstants.trackLength/2),
                                            new Translation2d(-Constants.RobotConstants.trackWidth/2, -Constants.RobotConstants.trackLength/2));
    modules = new SwerveModule[4];
    modules[frontLeftModule.operationOrderID] = frontLeftModule;
    modules[frontRightModule.operationOrderID] = frontRightModule;
    modules[backLeftModule.operationOrderID] = backLeftModule;
    modules[backRightModule.operationOrderID] = backRightModule;
    sOdometry = new SwerveDrivePoseEstimator(sKinematics, getGyroRotation2d(), getModulePositions(), new Pose2d(0, 0, new Rotation2d(0)));
    frontLeftModule.driveMotor.setPosition(0);
    frontRightModule.driveMotor.setPosition(0);
    backLeftModule.driveMotor.setPosition(0);
    backRightModule.driveMotor.setPosition(0);
    cSpeeds.omegaRadiansPerSecond = 0;
    cSpeeds.vxMetersPerSecond = 0;
    cSpeeds.vyMetersPerSecond = 0;
  
    


    // path planner init
     
    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(15.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(15.0, 0.0, 0.0) // Rotation PID constants
      ),
      robotConfig, // The robot configuration
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
);






  }
  public void initialize(){
    for(SwerveModule module : modules){
      module.initializeEncoder();
    }
    
    //pigeon2.reset();
    //sOdometry.resetPosition(getGyroRotation2d(), getModulePositions(), new Pose2d(0, 0, getGyroRotation2d()));
    
  }

  private Rotation2d getGyroRotation2d(){
    return Rotation2d.fromDegrees(getAngle()); 
  }

  public double getWrapedAngle(){
    if(getAngle() < 0){
      return (360 - Math.abs(getAngle()) % 360);
    }else{
      return getAngle() % 360;
    }
  }

  private SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] sPosition = new SwerveModulePosition[4];
    for(SwerveModule module : modules){
      //todo remove when module is back
      //if (module.operationOrderID != Constants.RobotConstants.backLeft.position)
      sPosition[module.operationOrderID] = module.getModulePosition();
    }
    return sPosition;
  }

  private SwerveModuleState[] getModuleStates() {
      SwerveModuleState[] sPosition = new SwerveModuleState[4];
    for(SwerveModule module : modules){
      //todo remove when module is back
      //if (module.operationOrderID != Constants.RobotConstants.backLeft.position)
      sPosition[module.operationOrderID] = module.getModuleState();
    }
    return sPosition;
  }

  // make back of robot front
  public void relativeDriveFlippedOrientation(Translation2d desiredTranslation2d, double desiredRotation){
  cSpeeds = new ChassisSpeeds(-desiredTranslation2d.getX(), -desiredTranslation2d.getY(), desiredRotation);
  SwerveModuleState[] states = sKinematics.toSwerveModuleStates(cSpeeds);
      
    for(SwerveModule module : modules){
      module.setState(states[module.operationOrderID]);
    }
  }


  public void relativeDrive(Translation2d desiredTranslation2d, double desiredRotation){
  cSpeeds = new ChassisSpeeds(desiredTranslation2d.getX(), desiredTranslation2d.getY(), desiredRotation);
  SwerveModuleState[] states = sKinematics.toSwerveModuleStates(cSpeeds);
      
  
    for(SwerveModule module : modules){
      module.setState(states[module.operationOrderID]);
    }
  }


  public void absoluteDrive(Translation2d desiredTranslation, double desiredRotation){
    //cSpeeds = new ChassisSpeeds(desiredTranslation.getX(), desiredTranslation.getY(), desiredRotation);
    cSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredTranslation.getX(), desiredTranslation.getY(), desiredRotation, getGyroRotation2d());
    //SwerveModuleState[] states = sKinematics.toSwerveModuleStates(cSpeeds);
  
   // cSpeeds = new ChassisSpeeds(t2D.getX(), t2D.getY() , desiredRotation.getRadians());
    
    // cSpeeds.vxMetersPerSecond = desiredTranslation.getX();
    // cSpeeds.vyMetersPerSecond = desiredTranslation.getY();
    // cSpeeds.omegaRadiansPerSecond = desiredRotation.getRadians();
    
    //cSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(cSpeeds, getGyroRotation2d());
    
    SwerveModuleState[] states = sKinematics.toSwerveModuleStates(cSpeeds);
    // SmartDashboard.putNumber("Desired X", desiredTranslation.getX());
    // SmartDashboard.putNumber("Desired Y", desiredTranslation.getY());
    // SmartDashboard.putNumber("Desired Rotation: (RPS)", desiredRotation);
    // SmartDashboard.putNumber("cSpeeds Y", cSpeeds.vyMetersPerSecond);
    ///SmartDashboard.putNumber("get Angle Pigeon: ", this.getAngle());
    
    for(SwerveModule module : modules){
      module.setState(states[module.operationOrderID]);
    }
  }

  public void drive(Translation2d desiredTranslation, double desiredRotation){
    if (isAbsolute){
      absoluteDrive(desiredTranslation, desiredRotation);
    } else{
      relativeDriveFlippedOrientation(desiredTranslation, desiredRotation);
    }
    
  }
  public void doAbsolute(){
    isAbsolute = true;
  }
  public void doRelative(){
    isAbsolute = false;
  }

  public void stop(){
    drive(new Translation2d(0,0), 0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    sOdometry.update(getGyroRotation2d(), getModulePositions());
    SmartDashboard.putNumber("Odometry X value", Units.metersToFeet(sOdometry.getEstimatedPosition().getX()));
    SmartDashboard.putNumber("Odometry Y value", Units.metersToFeet(sOdometry.getEstimatedPosition().getY()));
    SmartDashboard.putNumber("Odometry rotation", sOdometry.getEstimatedPosition().getRotation().getDegrees());
   // SmartDashboard.putNumber("Gyro Angle", getGyroRotation2d().getDegrees());
   // SmartDashboard.putNumber("Pigeon getYaw Value: ", getAngle());
  
    // SmartDashboard.putNumber("Odometry pose X: ", Units.metersToFeet(sOdometry.getPoseMeters().getX()));
    // SmartDashboard.putNumber("Odometry pose Y: ", Units.metersToFeet(sOdometry.getPoseMeters().getY()));

  }

  public void enableBrakes(){
    for(SwerveModule module: modules){
      if(module != null)
      //Brakes not working look here
      module.toggleBrakes(true);
    }
  }

  public void unenableBrakes(){
    for(SwerveModule module: modules){
      if(module != null)
      module.toggleBrakes(false);
      }
    }

  public void setOdometry(double xSet, double ySet){
    Pose2d pose2d = new Pose2d(xSet, ySet, getGyroRotation2d());
    sOdometry.resetPosition(getGyroRotation2d(), getModulePositions(), pose2d);
  }

  public void setPigeonAngle(double angle){
    pigeon2.setYaw(angle);
  }
    public void reset() {
    pigeon2.reset();
    frontLeftModule.driveMotor.setPosition(0);
    frontRightModule.driveMotor.setPosition(0);
    backLeftModule.driveMotor.setPosition(0);
    backRightModule.driveMotor.setPosition(0);
    cSpeeds.omegaRadiansPerSecond = 0;
    cSpeeds.vxMetersPerSecond = 0;
    cSpeeds.vyMetersPerSecond = 0;
    sOdometry.resetPosition(pigeon2.getRotation2d(), getModulePositions(), new Pose2d());
  }

  public double getAngle() {
      return pigeon2.getYaw().getValueAsDouble();
  }


  // path planner functions

  public Pose2d getPose(){
    return sOdometry.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose){
    sOdometry.resetPosition(getGyroRotation2d(), getModulePositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return sKinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds cSpeeds) {
    SwerveModuleState[] states = sKinematics.toSwerveModuleStates(cSpeeds);
      
    for(SwerveModule module : modules){
      module.setState(states[module.operationOrderID]);
    }
  }

  public class Governor{
    public enum MODE {SLOW, FAST};
    public MODE mode = MODE.FAST;
    public double fastInc = Constants.OperatorConstants.kFastInc;
    public double slowInc = Constants.OperatorConstants.kSlowInc;
    public double fastGovernor = Constants.OperatorConstants.kDefaultFast;
    public double slowGovernor = Constants.OperatorConstants.kDefaultSlow;

    public Governor(){
      SmartDashboard.putString("Governor Mode: ", mode.toString());
    }

    public double getGovernor(){
      switch (mode) {
        case SLOW:
          return slowGovernor;
        case FAST:
          return fastGovernor;
      }
      return 0;
    }
    public void setSlowMode(){
      mode = MODE.SLOW;
      SmartDashboard.putString("Governor Mode: ", mode.toString());
    }
    public void setFastMode(){
      mode = MODE.FAST;
      SmartDashboard.putString("Governor Mode: ", mode.toString());
    }
    public void increment(){ 
      switch (mode) {
        case SLOW:
          slowGovernor += slowInc;
        case FAST:
          fastGovernor += fastInc;
      }
    }
    public void decrement(){ 
      switch (mode) {
        case SLOW:
          slowGovernor -= slowInc;
        case FAST:
          fastGovernor -= fastInc;
      }
    }
  }
}
