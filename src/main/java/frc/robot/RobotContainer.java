// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TrackTag;
import frc.robot.subsystems.Spindex;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PWM;
import frc.robot.subsystems.Intake;

import java.io.File;
import java.time.Instant;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  ClimberSubsystem climber = new ClimberSubsystem();
  XboxController driver = new XboxController(0);
  ShooterSubsystem shooter = new ShooterSubsystem();
  Spindex spindex = new Spindex();
  Intake intake = new Intake();
  PWM pwmHook = new PWM(0);
  PWM pwmPin = new PWM(1);
  public Command climbtotal = new InstantCommand(()-> pwmPin.setPosition(0)).andThen(new InstantCommand(()->climber.setWantedPosition(0)).alongWith(new InstantCommand(()->pwmHook.setPosition(45))));
  public Command climbdowntotal = new InstantCommand(()->climber.setWantedPosition(0)).alongWith(new InstantCommand(()->pwmHook.setPosition(120)));
  public Command climber_lock = new InstantCommand(()->pwmPin.setPosition(0));
  public Command climber_unlock = new InstantCommand(()->pwmPin.setPosition(0));
  public Command shooter_total = new InstantCommand(()->spindex.setSpeed(.05)).andThen(new InstantCommand(()->shooter.setSpeed(0.9)).alongWith(new InstantCommand (()->drivebase.autoAlign())).alongWith(new InstantCommand(()->drivebase.autoRange())).alongWith(new InstantCommand(()->drivebase.drive_limelight())));
  public Command shooter_off_total = new InstantCommand(()->spindex.setSpeed(0)).alongWith(new InstantCommand(()->shooter.setSpeed(0)));
  public Command intake_total_on = new InstantCommand(()->intake.setWantedPosition(0));
  private final Trigger climbTrigger = new JoystickButton(driver, XboxController.Button.kA.value);
  private final Trigger climbTriggerno = new JoystickButton(driver, XboxController.Button.kB.value);
  private final Trigger shooteron = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final Trigger intake_on = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final Trigger shooteroff = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final Trigger climb_lock = new JoystickButton(driver, XboxController.Button.kY.value);
  


  private final Trigger TrackTag = new Trigger(driver::getXButton);

  public TrackTag track = new TrackTag(drivebase);

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    configureBindings();
  }
  private void configureBindings()
  {
    // Grady Binding 🥳
    TrackTag.toggleOnTrue(track);
    TrackTag.toggleOnFalse(new InstantCommand(()->CommandScheduler.getInstance().cancel(track)));

    climbTrigger.onTrue(climbtotal);
    climbTriggerno.onTrue(climbdowntotal);
    shooteron.toggleOnTrue(shooter_total);
    shooteroff.toggleOnFalse(shooter_off_total);
    climb_lock.onTrue(climber_lock);
    climb_lock.onFalse(climber_unlock);
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);
    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

  }
 SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * -1,
                                                                () -> m_driverController.getLeftX() * -1)
                                                            .withControllerRotationAxis(m_driverController::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
                                                              m_driverController::getRightY)
                                                           .headingWhile(true);

  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);
  
  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -m_driverController.getLeftY(),
                                                                        () -> -m_driverController.getLeftX())
                                                                    .withControllerRotationAxis(() -> m_driverController.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  m_driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  m_driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));
  
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("Left");
  }
  
}