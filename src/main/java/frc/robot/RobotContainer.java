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
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PlacerCommand;
import frc.robot.commands.ResetDriveTrain;
import frc.robot.commands.ResetGyro;
import frc.robot.subsystems.AlgaeClaw;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Placer;
import frc.robot.subsystems.SwerveDrivetrain;

import java.util.Map;

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
  private final CommandXboxController m_gameOperatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
 
  public final SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain(m_driverController);
  public final AlgaeClaw algaeClaw = new AlgaeClaw( );
  public final Elevator elevator = new Elevator();
  public final Climber climber = new Climber();
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

    Trigger gameOpA = m_gameOperatorController.a();
    Trigger gameOpB = m_gameOperatorController.b();
    Trigger gameOpX = m_gameOperatorController.x();
    Trigger gameOpY = m_gameOperatorController.y();
    Trigger gameOpLeftBumper = m_gameOperatorController.leftBumper();
    Trigger gameOpRightBumper = m_gameOperatorController.rightBumper();
    Trigger gameOpPOVUp = m_gameOperatorController.povUp();
    Trigger gameOpPOVDown = m_gameOperatorController.povDown();
    Trigger gameOpPOVRight = m_gameOperatorController.povRight();
    Trigger gameOpleftTrigger = m_gameOperatorController.axisGreaterThan(Constants.OperatorConstants.leftTriggerAxis,.6);
    Trigger gameOpRightTrigger = m_gameOperatorController.axisGreaterThan(Constants.OperatorConstants.rightTriggerAxis,.6);
    Trigger gameOpBackLeft = m_gameOperatorController.button(Constants.OperatorConstants.kLeftBackButton);
    Trigger gameOpBackRight = m_gameOperatorController.button(Constants.OperatorConstants.kRightBackButton);





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


    // todo add unused buttons for driver

    // Operator Commands
    gameOpA.whileTrue(new ElevatorCommand(elevator, 0.1));
    gameOpB.whileTrue(new ClimberCommand(climber));
    gameOpX.whileTrue(new IntakeCommand(intake));
    gameOpY.whileTrue(new PlacerCommand(placer));

   
    //Driver Commands
    resetDriverEncoder.whileTrue(Commands.runOnce(swerveDrivetrain::initialize, swerveDrivetrain));

    m_driverController.a().whileTrue(new AlgaeClawCommand(algaeClaw));

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