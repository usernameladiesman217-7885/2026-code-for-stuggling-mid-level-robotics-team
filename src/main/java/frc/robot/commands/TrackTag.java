// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrackTag extends Command {
  SwerveSubsystem swerve;
  HolonomicDriveController controller;
  PIDController xController;
  PIDController yController;
  ProfiledPIDController rotationController;
  Pose2d wantedError = new Pose2d(1.5, 0, Rotation2d.kZero);
  Pose2d currentRobotPosition;
  ChassisSpeeds wantedRobotSpeed;
  State goalState = new State(0, 0, 0, wantedError, 0);
  /** Creates a new TrackTag. */
  public TrackTag(SwerveSubsystem swerve_) {
    swerve = swerve_;
    addRequirements(swerve);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController = new PIDController(1, 0, 0);
    yController = new PIDController(0.3, 0, 0);
    rotationController = new ProfiledPIDController(3, 0, 0, new Constraints(Math.PI, Math.PI));
    controller = new HolonomicDriveController(xController, yController, rotationController);
    controller.setTolerance(new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(5)));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentRobotPosition = new Pose2d(
      -LimelightHelpers.getCameraPose3d_TargetSpace("limelight-panther").getZ(),
      -LimelightHelpers.getTargetPose3d_CameraSpace("limelight-panther").getX(),
      Rotation2d.fromDegrees(-LimelightHelpers.getTX("limelight-panther"))
    );

    wantedRobotSpeed = controller.calculate(currentRobotPosition, goalState, Rotation2d.kZero);
    

    swerve.drive(controller.calculate(currentRobotPosition, goalState, Rotation2d.kZero));
    SmartDashboard.putNumber("Wanted X Speed", currentRobotPosition.getX());
    SmartDashboard.putNumber("Wanted Z Speed", currentRobotPosition.getY());
    SmartDashboard.putNumber("Wanted Rotation", currentRobotPosition.getRotation().getRotations());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (controller.atReference()){
      return true;
    }
    return false;
  }
}
