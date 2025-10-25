// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.AlgaeClawCommand;
import frc.robot.commands.AutoLeaveLine;
import frc.robot.commands.AutoLineUpReef;
import frc.robot.commands.Drive;
import frc.robot.commands.IntakeUntilBeamBreak;
import frc.robot.commands.PIDElevatorCommand;
import frc.robot.commands.PlacerCommand;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.AutoLineUpReef.TargetSide;
import frc.robot.subsystems.AlgaeClaw;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.EleLevel;
import frc.robot.subsystems.ElevatorNoHalls;
import frc.robot.subsystems.LimeLightSub;
import frc.robot.subsystems.Placer;
import frc.robot.subsystems.SwerveDrivetrain;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandGenericHID m_gameOperatorController = new CommandGenericHID(OperatorConstants.kOperatorControllerPort);
  public Robot robot = new Robot();
 
  public final SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain(m_driverController, robot);
  //public final AlgaeClaw algaeClaw = new AlgaeClaw( );
 
  public final Climber climber = new Climber();
  public final Placer placer = new Placer();
  public final Elevator elevator = new Elevator(placer.getElevatorLimitSwitch());
 // public final AlgaeClaw algaeClaw = new AlgaeClaw();

  public final Trigger intakeBeamBrakeTrigger = new Trigger(() -> !placer.getPlacerBeamBreak().isPressed());
  public final Trigger elevatorBottomLimitTrigger = new Trigger(() -> elevator.getSignalOfLevel(EleLevel.L0));

  public SendableChooser<Command> autoChooser = new SendableChooser<>();
  public final String autoTest = "newyork";
  public final String auto2Test = "New Auto";
  public final String auto3Test = "New New New Auto";

  public final LimeLightSub reefLimeLight = new LimeLightSub("reef", 0);
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
   // SmartDashboard.putData(new PowerDistribution(1, ModuleType.kRev));
    SmartDashboard.putData(CommandScheduler.getInstance());
    
  //negate to match joystick to robot
    swerveDrivetrain.setDefaultCommand(new Drive(swerveDrivetrain, () -> -m_driverController.getLeftX(), () -> -m_driverController.getRightX(), () -> -m_driverController.getLeftY()));

    // path planner named commands
    /* 
    NamedCommands.registerCommand("L4Elevator", new ElevatorCommand(elevator, EleLevel.L4, this));
    NamedCommands.registerCommand("L3Elevator", new ElevatorCommand(elevator, EleLevel.L3, this));
    NamedCommands.registerCommand("L2Elevator", new ElevatorCommand(elevator, EleLevel.L2, this));
    NamedCommands.registerCommand("L1Elevator", new ElevatorCommand(elevator, EleLevel.L1, this));
    NamedCommands.registerCommand("L0Elevator", new ElevatorCommand(elevator, EleLevel.L0, this));
    */

   

    NamedCommands.registerCommand("Place Coral", new PlacerCommand(placer));
    NamedCommands.registerCommand("Grab Coral", new IntakeUntilBeamBreak(placer, this));

    //NamedCommands.registerCommand("autoFindNoteClockWise", autoFindNoteClockWiseCommand);
    //NamedCommands.registerCommand("autoFindNoteCounterClockWise", autoFindNoteCounterClockWiseCommand);
   // NamedCommands.registerCommand("autoSpeakerLineUp", new CameraDrive(swerveDrivetrain, shooterLimeLight, SemiAutoConstants.speaker, this.intake, this.firstBeamBreak));
  
   
   autoChooser.setDefaultOption("Do Nothing", new PathPlannerAuto("Do Nothing"));
   autoChooser.addOption("One Coral Middle", new PathPlannerAuto("One Coral Middle"));
   autoChooser.addOption("Bottom Right 2 Coral Auto", new PathPlannerAuto("Bottom Right 2 Coral Auto"));
   autoChooser.addOption("Top Right 2 Coral Auto", new PathPlannerAuto("Top Right 2 Coral Auto"));
   autoChooser.addOption("3 Coral Top Auto", new PathPlannerAuto("3 Coral Top Auto"));
   autoChooser.addOption("3 Coral Bottom Auto", new PathPlannerAuto("3 Coral Bottom Auto"));
   SmartDashboard.putData(autoChooser);
  }

  /*
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    Trigger button1 = m_gameOperatorController.button(1);
    Trigger button2 = m_gameOperatorController.button(2);
    Trigger button3 = m_gameOperatorController.button(3);
    Trigger button4 = m_gameOperatorController.button(4);
    Trigger button5 = m_gameOperatorController.button(5);
    Trigger button6 = m_gameOperatorController.button(6);
    Trigger button7 = m_gameOperatorController.button(7);
    Trigger button8 = m_gameOperatorController.button(8);
    Trigger button9 = m_gameOperatorController.button(9);
    Trigger button10 = m_gameOperatorController.button(10);
    Trigger button11 = m_gameOperatorController.button(11);
    Trigger button12 = m_gameOperatorController.button(12);
    Trigger button13 = m_gameOperatorController.button(13);
    Trigger button14 = m_gameOperatorController.button(14);
    Trigger button15 = m_gameOperatorController.button(15);
    Trigger button16 = m_gameOperatorController.button(16);


    Trigger driverBackLeft = m_driverController.button(Constants.OperatorConstants.kLeftBackButton);
    Trigger driverBackRight = m_driverController.button(Constants.OperatorConstants.kRightBackButton);
    Trigger driverLeftBumper = m_driverController.leftBumper();
    Trigger driverRightBumper = m_driverController.rightBumper();
    Trigger driverA = m_driverController.a();
    Trigger driverB = m_driverController.b();
    Trigger driverX = m_driverController.x();
    Trigger driverY = m_driverController.y();

    Trigger driverOpPOVDown = m_driverController.povDown();

    
    // Driver mapping

    Trigger driverToggleFieldOriented = driverY;
    Trigger resetDriverEncoder = driverBackRight;
    Trigger resetDriverGyro = driverBackLeft;
    Trigger slowMode = driverLeftBumper;
    Trigger fastMode = driverRightBumper;
    Trigger reduceSpeed = m_driverController.axisGreaterThan(Constants.OperatorConstants.leftTriggerAxis,.6);
    Trigger increaseSpeed = m_driverController.axisGreaterThan(Constants.OperatorConstants.rightTriggerAxis,.6);
    driverA.whileTrue(new PlacerCommand(placer)).onFalse(Commands.runOnce(placer::stop, placer));
    driverX.whileTrue(new AutoLineUpReef(swerveDrivetrain, TargetSide.Left)).onFalse(Commands.runOnce(swerveDrivetrain::stop, swerveDrivetrain));
    driverB.whileTrue(new AutoLineUpReef(swerveDrivetrain, TargetSide.Right)).onFalse(Commands.runOnce(swerveDrivetrain::stop, swerveDrivetrain));


    // Operator Commands
  
    //gameOpB.whileTrue(new ClimberCommand(climber));

    button2.whileTrue(Commands.runOnce(elevator::moveUp, elevator)).onFalse(Commands.runOnce(elevator::stop, elevator));
    button1.whileTrue(Commands.runOnce(elevator::moveDown, elevator)).onFalse(Commands.runOnce(elevator::stop, elevator));

    button3.whileTrue(Commands.runOnce(climber::reverse, climber)).onFalse(Commands.runOnce(climber::stop, climber));
    button4.whileTrue(Commands.runOnce(climber::forward, climber)).onFalse(Commands.runOnce(climber::stop, climber));
    
    
    button5.onTrue(new PIDElevatorCommand(elevator, RobotConstants.L0Position)).onFalse(Commands.runOnce(elevator::setIdle, elevator));
    button6.onTrue(new PIDElevatorCommand(elevator, RobotConstants.L2Position)).onFalse(Commands.runOnce(elevator::setIdle, elevator));
    button7.onTrue(new PIDElevatorCommand(elevator, RobotConstants.L3Position)).onFalse(Commands.runOnce(elevator::setIdle, elevator));
    button8.onTrue(new PIDElevatorCommand(elevator, RobotConstants.L4Position)).onFalse(Commands.runOnce(elevator::setIdle, elevator));
   
    button9.whileTrue(new IntakeUntilBeamBreak(placer, this)).onFalse(Commands.runOnce(placer::stop, placer));
    
    button10.whileTrue(new PlacerCommand(placer)).onFalse(Commands.runOnce(placer::stop, placer));
    //button11.onTrue(Commands.runOnce(algaeClaw::setPlaceState)).onFalse(Commands.runOnce(algaeClaw::setIdleState));
    button13.whileTrue(Commands.runOnce(placer::algaePlace, placer)).onFalse(Commands.runOnce(placer::stop, placer));
    button14.whileTrue(Commands.runOnce(placer::backward, placer)).onFalse(Commands.runOnce(placer::stop, placer));
    //button15.onTrue(Commands.runOnce(algaeClaw::setIntakeState)).onFalse(Commands.runOnce(algaeClaw::setIdleState));
    button16.onTrue(Commands.runOnce(elevator::setPIDOff, elevator));

    driverOpPOVDown.whileTrue(Commands.runOnce(climber::reverse, climber)).onFalse(Commands.runOnce(climber::stop, climber));
    m_driverController.povUp().whileTrue(Commands.runOnce(climber::forward, climber)).onFalse(Commands.runOnce(climber::stop, climber));
    


    //Driver Commands
    resetDriverEncoder.whileTrue(Commands.runOnce(swerveDrivetrain::initialize, swerveDrivetrain));

   // m_driverController.a().whileTrue(new AlgaeClawCommand(algaeClaw));

    // driverToggleFieldOriented.onTrue(Commands.runOnce(swerveDrivetrain::doRelative));
    // driverToggleFieldOriented.onFalse(Commands.runOnce(swerveDrivetrain::doAbsolute));
    
    resetDriverGyro.whileTrue(new ResetGyro(swerveDrivetrain));
    slowMode.onTrue(Commands.runOnce(swerveDrivetrain.governor::setSlowMode, swerveDrivetrain));
    fastMode.onTrue(Commands.runOnce(swerveDrivetrain.governor::setFastMode, swerveDrivetrain));
    reduceSpeed.onTrue(Commands.runOnce(swerveDrivetrain.governor::decrement, swerveDrivetrain));
    increaseSpeed.onTrue(Commands.runOnce(swerveDrivetrain.governor::increment, swerveDrivetrain));
  }

  public Command getAutonomousCommand() {
    Command auto = new AutoLeaveLine(swerveDrivetrain);
    //PathPlannerAuto auto = new PathPlannerAuto(autoChooser.getSelected());
    //SmartDashboard.putString("Auto Selected", autoChooser.getSelected());
    //return autoChooser.getSelected(); 
    //return auto.andThen(Commands.runOnce(swerveDrivetrain::stop));
    return auto.andThen(Commands.runOnce(swerveDrivetrain::stop)).andThen(new PIDElevatorCommand(elevator, RobotConstants.L4Position)).andThen(Commands.waitSeconds(3)).andThen(new PlacerCommand(placer)).andThen(new PIDElevatorCommand(elevator, RobotConstants.L2Position)).andThen(Commands.waitSeconds(2)).andThen(Commands.runOnce(elevator::setIdle, elevator));
   //return auto.andThen(new IntakeUntilBeamBreak(intake, placer, this)).andThen(Commands.runOnce(swerveDrivetrain::stop, swerveDrivetrain)).andThen(new PIDElevatorCommand(elevator, EleLevel.L4, this)).andThen(Commands.waitSeconds(.5));

  }
  public Command driveTenFeetThenStop(){
    Command auto = new AutoLeaveLine(swerveDrivetrain);
    return auto.andThen(Commands.runOnce(swerveDrivetrain::stop, swerveDrivetrain));
  }
  public Command deadReckonL4(){
    Command auto = new AutoLeaveLine(swerveDrivetrain);
    return auto.andThen(Commands.runOnce(swerveDrivetrain::stop, swerveDrivetrain)).andThen(new PIDElevatorCommand(elevator, RobotConstants.L4Position)).andThen(new PlacerCommand(placer)).andThen(new PIDElevatorCommand(elevator, RobotConstants.L0Position));
  }
  public Command deadReckonL3(){
    Command auto = new AutoLeaveLine(swerveDrivetrain);
    return auto.andThen(Commands.runOnce(swerveDrivetrain::stop, swerveDrivetrain)).andThen(new PIDElevatorCommand(elevator, RobotConstants.L3Position)).andThen(new PlacerCommand(placer)).andThen(new PIDElevatorCommand(elevator, RobotConstants.L0Position));
  }
  public Command deadReckonL2(){
    Command auto = new AutoLeaveLine(swerveDrivetrain);
    return auto.andThen(Commands.runOnce(swerveDrivetrain::stop, swerveDrivetrain)).andThen(new PIDElevatorCommand(elevator, RobotConstants.L2Position)).andThen(new PlacerCommand(placer)).andThen(new PIDElevatorCommand(elevator, RobotConstants.L0Position));
  }
  public Command doNothing(){
    return new PrintCommand("Nothing");
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

}