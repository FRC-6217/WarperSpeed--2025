// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SemiAutoConstants;
import frc.robot.commands.AlgaeClawCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.Drive;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ResetDriveTrain;
import frc.robot.commands.ResetGyro;
import frc.robot.subsystems.AlgaeClaw;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Placer;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Elevator.EleLevel;

import java.util.Map;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
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
 
  public final SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain(m_driverController);
  //public final AlgaeClaw algaeClaw = new AlgaeClaw( );
  public final Elevator elevator = new Elevator();
  //public final Climber climber = new Climber();
  public final Intake intake = new Intake();
  public final Placer placer = new Placer();

  public SendableChooser<String> autoChooser = new SendableChooser<>();
  public final String autoTest = "newyork";
  public final String auto2Test = "New Auto";
  public final String auto3Test = "New New New Auto";

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
   // SmartDashboard.putData(new PowerDistribution(1, ModuleType.kRev));
    SmartDashboard.putData(CommandScheduler.getInstance());
    
  //negate to match joystick to robot
    swerveDrivetrain.setDefaultCommand(new Drive(swerveDrivetrain, () -> -m_driverController.getLeftX(), () -> -m_driverController.getRightX(), () -> -m_driverController.getLeftY()));

    // path planner named commands
    NamedCommands.registerCommand("L4Elevator", new ElevatorCommand(elevator, EleLevel.L4));
    NamedCommands.registerCommand("L3Elevator", new ElevatorCommand(elevator, EleLevel.L3));
    NamedCommands.registerCommand("L2Elevator", new ElevatorCommand(elevator, EleLevel.L2));
    NamedCommands.registerCommand("L1Elevator", new ElevatorCommand(elevator, EleLevel.L1));
    NamedCommands.registerCommand("L0Elevator", new ElevatorCommand(elevator, EleLevel.L0));
    //NamedCommands.registerCommand("autoFindNoteClockWise", autoFindNoteClockWiseCommand);
    //NamedCommands.registerCommand("autoFindNoteCounterClockWise", autoFindNoteCounterClockWiseCommand);
   // NamedCommands.registerCommand("autoSpeakerLineUp", new CameraDrive(swerveDrivetrain, shooterLimeLight, SemiAutoConstants.speaker, this.intake, this.firstBeamBreak));
  
   autoChooser.setDefaultOption(autoTest, autoTest); 
   autoChooser.addOption(auto2Test, auto2Test);
   autoChooser.addOption(auto3Test, auto3Test);
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
    Trigger button8 = m_gameOperatorController.button(0);
    Trigger button9 = m_gameOperatorController.button(9);
    Trigger button10 = m_gameOperatorController.button(10);
    Trigger button11 = m_gameOperatorController.button(11);
    Trigger button12 = m_gameOperatorController.button(12);
    Trigger button13 = m_gameOperatorController.button(13);
    Trigger button14 = m_gameOperatorController.button(14);
    Trigger button15 = m_gameOperatorController.button(15);


    Trigger driverBackLeft = m_driverController.button(Constants.OperatorConstants.kLeftBackButton);
    Trigger driverBackRight = m_driverController.button(Constants.OperatorConstants.kRightBackButton);
    Trigger driverLeftBumper = m_driverController.leftBumper();
    Trigger driverRightBumper = m_driverController.rightBumper();
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



    // Operator Commands
  
    //gameOpB.whileTrue(new ClimberCommand(climber));

    button2.whileTrue(Commands.runOnce(elevator::moveUp, elevator)).onFalse(Commands.runOnce(elevator::stop, elevator));
    button1.whileTrue(Commands.runOnce(elevator::moveDown, elevator)).onFalse(Commands.runOnce(elevator::stop, elevator));
    button9.whileTrue(Commands.runOnce(intake::forward, intake)).onFalse(Commands.runOnce(intake::stop, intake));
    button10.whileTrue(Commands.runOnce(placer::forward, placer)).onFalse(Commands.runOnce(placer::stop, placer));
    button13.whileTrue(Commands.runOnce(intake::backward, intake)).onFalse(Commands.runOnce(intake::stop, intake));
    button14.whileTrue(Commands.runOnce(placer::backward, placer)).onFalse(Commands.runOnce(placer::stop, placer));

   
    //Driver Commands
    resetDriverEncoder.whileTrue(Commands.runOnce(swerveDrivetrain::initialize, swerveDrivetrain));

   // m_driverController.a().whileTrue(new AlgaeClawCommand(algaeClaw));

    driverToggleFieldOriented.onTrue(Commands.runOnce(swerveDrivetrain::doRelative));
    driverToggleFieldOriented.onFalse(Commands.runOnce(swerveDrivetrain::doAbsolute));
    
    resetDriverGyro.whileTrue(new ResetGyro(swerveDrivetrain));
    slowMode.onTrue(Commands.runOnce(swerveDrivetrain.governor::setSlowMode, swerveDrivetrain));
    fastMode.onTrue(Commands.runOnce(swerveDrivetrain.governor::setFastMode, swerveDrivetrain));
    reduceSpeed.onTrue(Commands.runOnce(swerveDrivetrain.governor::decrement, swerveDrivetrain));
    increaseSpeed.onTrue(Commands.runOnce(swerveDrivetrain.governor::increment, swerveDrivetrain));





  }

  public Command getAutonomousCommand() {
   
    PathPlannerAuto auto = new PathPlannerAuto(autoChooser.getSelected());
    SmartDashboard.putString("Auto Selected", autoChooser.getSelected());
    return auto;
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

}