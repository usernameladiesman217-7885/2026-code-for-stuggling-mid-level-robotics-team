// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import frc.robot.Constants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import java.util.concurrent.atomic.AtomicReference;


import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;



public class SwerveSubsystem extends SubsystemBase 
{
  File directory = new File(Filesystem.getDeployDirectory(),"swerve");
  private final SwerveDrive  swerveDrive;

    private final PIDController controller = new PIDController(1, 0, 0);
  private final PIDController controller_range = new PIDController(1,0, 0);
  private final PIDController forwardPidController = new PIDController(0.1, 0, 0);

   
   
  public SwerveSubsystem(File Directory) {
    controller.enableContinuousInput(-Math.PI, Math.PI);
   double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8);
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 8.14);
    System.out.println("\"conversionFactor\": {");
    System.out.println("\t\"angle\": " + angleConversionFactor + ",");
    System.out.println("\t\"drive\": " + driveConversionFactor);
    System.out.println("}");
    boolean blueAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue;
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
     try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.maxSpeed, new Pose2d(Meter.of(1),
                                                                                                        Meter.of(4),
                                                                                                        Rotation2d.fromDegrees(0)));

  
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false);
    swerveDrive.setCosineCompensator(false);
    swerveDrive.setAngularVelocityCompensation(true,
                                               true,
                                               0.1); 
    swerveDrive.setModuleEncoderAutoSynchronize(false,
                                                1); 
  }

  

      private int     outofAreaReading = 0;
    private boolean initialReading = false;

  
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
  {
    swerveDrive = new SwerveDrive(driveCfg,
                                  controllerCfg,
                                  Constants.maxSpeed,
                                  new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)),
                                             Rotation2d.fromDegrees(0)));
  }
  @Override
  public void periodic() {

  
    
    
  }
  


    // This method will be called once per scheduler run
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setupPathPlanner(){
    RobotConfig config;

    try{
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedForwards = true;
      
      AutoBuilder.configure(
        this::getPose,

        this::resetOdometry,

        this::getRobotVelocity,

        (speedsRobotRelative, moduleFeedForwards) -> {
          if (enableFeedForwards)
        {
        swerveDrive.drive(
          speedsRobotRelative,
          swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
          moduleFeedForwards.linearForces()
          );
        } else
        {
          swerveDrive.setChassisSpeeds(speedsRobotRelative);
        }
      },
      new PPHolonomicDriveController(
        new PIDConstants(5.0,0.0,0.0),
        new PIDConstants(5.0, 0.0, 0.0)
      ),
      config,
      ()-> {
        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent())
        {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this
      );
      }catch (Exception e)
      {
        e.printStackTrace();
      }
      PathfindingCommand.warmupCommand().schedule();
  }
  public Command getAutonomousCommand(String pathName)
  {
    return new PathPlannerAuto(pathName);
  }
  public Command driveToPose(Pose2d pose)
  {
    PathConstraints constraints = new PathConstraints(
      swerveDrive.getMaximumChassisVelocity(), 4.0,
      swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
      return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0)
      );
  }
  private Command driveWithsetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
  throws IOException, ParseException
  {
    SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(RobotConfig.fromGUISettings(),
                                                                            swerveDrive.getMaximumChassisAngularVelocity());
    AtomicReference<SwerveSetpoint> prevSetpoint
      = new AtomicReference<>(new SwerveSetpoint(swerveDrive.getRobotVelocity(),
                                                 swerveDrive.getStates(),
                                                 DriveFeedforwards.zeros(swerveDrive.getModules().length)));
    AtomicReference<Double> previousTime = new AtomicReference<>();

    return startRun(()-> previousTime.set(Timer.getFPGATimestamp()),
                    ()->{
                      double newTime = Timer.getFPGATimestamp();
                      SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(prevSetpoint.get(),
                                                                                      robotRelativeChassisSpeed.get(),
                                                                                      newTime - previousTime.get());
                      swerveDrive.drive(newSetpoint.robotRelativeSpeeds(),
                                        newSetpoint.moduleStates(),
                                        newSetpoint.feedforwards().linearForces());
                      prevSetpoint.set(newSetpoint);
                      previousTime.set(newTime);
                    });
  }
  public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds)
  {
    try
    {
      return driveWithsetpointGenerator(() -> {
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading());
      });
    } catch (Exception e)
    {
      DriverStation.reportError(e.toString(), true);
    }
    return Commands.none();
    }
  public Command sysIdDriveMotorCommand()
    {
      return SwerveDriveTest.generateSysIdCommand(
               SwerveDriveTest.setDriveSysIdRoutine(
                new Config(),
                this, swerveDrive, 12, true),
                3.0,5.0,3.0);
    }
  public Command sysIdAngleMotorCommand()
    {
      return SwerveDriveTest.generateSysIdCommand(
               SwerveDriveTest.setAngleSysIdRoutine(
                new Config(),
                this, swerveDrive),
                3.0,5.0,3.0);
    }
  public Command centerModulesCommand()
  {
    return run(()-> Arrays.asList(swerveDrive.getModules())
                           .forEach(it -> it.setAngle(0.0)));
  }
  public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond)
  {
    return run(()-> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
           .until(()-> swerveDrive.getPose().getTranslation().getDistance(new Translation2d(0,0)) >
                              distanceInMeters);
  }
  public void replaceSwerveModuleFeedForward( double kS, double kV, double kA)
  {
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier AngularRotationX)
  {
    return run(()-> {
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                                  translationX.getAsDouble()*swerveDrive.getMaximumChassisVelocity(),
                                  translationY.getAsDouble()*swerveDrive.getMaximumChassisVelocity()), 0.8),
                            Math.pow(AngularRotationX.getAsDouble(),3)*swerveDrive.getMaximumChassisAngularVelocity(),
                            true,
                            false);
    });
    }
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
                              {
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                 translationY.getAsDouble()), 0.8);
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumChassisVelocity()));
    });
 }
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }
    public SwerveDriveKinematics getKinematics()
  {
    return swerveDrive.kinematics;
  }
   public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }
    public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }
    public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }
    private boolean isRedAlliance()
  {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }
  public void zeroGyroWithAlliance()
  {
    if (isRedAlliance())
    {
      zeroGyro();
      //Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else
    {
      zeroGyro();
    }
  }
  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        headingX,
                                                        headingY,
                                                        getHeading().getRadians(),
                                                        Constants.maxSpeed);
  }
   public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        Constants.maxSpeed);
  }
   public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }
   public SwerveController getSwerveController()
  {
    return swerveDrive.swerveController;
  }
  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }
   public void lock()
  {
    swerveDrive.lockPose();
  }
  public Rotation2d getPitch()
  {
    return swerveDrive.getPitch();
  }

  
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public void diveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
   
  }
  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(()-> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }
  public Command drive(Supplier<ChassisSpeeds> driveAngularVelocity)
  {
    return run(() -> {
      swerveDrive.drive(driveAngularVelocity.get());
    });
  }
  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false);
  }
  public Pose2d getPose(){
    return swerveDrive.getPose();
  }
  public void resetOdometry(Pose2d initialHolonomicPose){
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public ChassisSpeeds getRobotVelocity(){
    return swerveDrive.getRobotVelocity();
  }
  public Command autoAlign()
  {
   return this.driveCommandu(()->0,()-> controller_range.calculate(Units.degreesToRadians(LimelightHelpers.getTX("panther")), 4), ()->0);
  }
  public Command driveCommandu(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisVelocity(),
                                          Math.pow(translationY.getAsDouble(), 3) * swerveDrive.getMaximumChassisVelocity()),
                        Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
    });
  }
    public Command autoRange(){
    return this.driveCommandu(()->0,()-> controller_range.calculate(Units.degreesToRadians(LimelightHelpers.getTX("panther")), 4), ()->0);
  }
  public void addFakeVisionReading()
  {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }
  public double autoforward(){
    var  targetingFowardSpeed = forwardPidController.calculate(Units.degreesToRadians(LimelightHelpers.getTY("panther")), 0.5);
    targetingFowardSpeed *=Constants.maxSpeed;
    targetingFowardSpeed *= -1;
    return targetingFowardSpeed;
  }
   public Command drive_limelight(){
    final var rot_limelight = controller.calculate(Units.degreesToRadians(LimelightHelpers.getTX("panther")),0);
    var rot = rot_limelight;

    final var forward_limelight = autoforward();
    var xspeed = forward_limelight;

    return this.driveCommandu(()->xspeed, ()-> 0,()->rot);
  }
  }



