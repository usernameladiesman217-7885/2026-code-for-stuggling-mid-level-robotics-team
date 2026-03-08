// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot;

import static edu.wpi.first.units.Units.Rotation;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PWM;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class ClimberSubsystem extends SubsystemBase {

   PWM pwmHook = new PWM(0);
   PWM pwmPin = new PWM(1);

   public SparkMax motor1 = new SparkMax(9, MotorType.kBrushless);
   public SparkMax motor2 = new SparkMax(10, MotorType.kBrushless);
   public PIDController controller = new PIDController(0.09, 0.24, 0.0);
   SparkMaxConfig motor1Config = new SparkMaxConfig();
   SparkMaxConfig motor2Config = new SparkMaxConfig();
   PWM pwmExample;


   public ClimberSubsystem() {
      motor1Config.inverted(false);
      motor2Config.inverted(true);
      motor1Config.idleMode(IdleMode.kBrake);
      motor2Config.idleMode(IdleMode.kBrake);

      motor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      motor2.configure(motor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   }
   @Override
   public void periodic() {
      SmartDashboard.putNumber("Wanted Value", controller.getSetpoint());
      SmartDashboard.putNumber("Actual Position", motor2.getEncoder().getPosition());
      double pid = controller.calculate(motor2.getEncoder().getPosition());
      motor1.set(MathUtil.clamp(pid, -.4, .4));
      motor2.set(MathUtil.clamp(pid, -.4,.4));
      System.out.println(pid);
      

      

   }
   public void setWantedPosition(double rotations) 
   {
      controller.setSetpoint(rotations);
   }
   public void addWantedPosition(double rotations) 
   {
      controller.setSetpoint(controller.getSetpoint() + rotations);
   }
   public void getSetpoint(double rotations)
   {
      controller.getSetpoint();
   }
   public void setPosition(double position)
   {
      pwmExample.setPosition(position);
   }
   public boolean atSetpoint()
   {
      return controller.atSetpoint();
   }
}