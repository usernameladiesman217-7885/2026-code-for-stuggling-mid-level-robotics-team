package frc.robot.subsystems;

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

public class Intake extends SubsystemBase{

    SparkMax motor_pivot = new SparkMax(14, MotorType.kBrushless);
    SparkMaxConfig pivot_config = new SparkMaxConfig();
    public PIDController intake_controller = new PIDController(.09,.24,0);

    public Intake()
    {
        pivot_config.inverted(false);
        pivot_config.idleMode(IdleMode.kBrake);
        motor_pivot.configure(pivot_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    @Override public void periodic()
    {
      double pid = intake_controller.calculate(motor_pivot.getEncoder().getPosition());
      motor_pivot.set(MathUtil.clamp(pid, -.4, .4));
      System.out.println(pid);  
    }
     public void setWantedPosition(double rotations) 
   {
      intake_controller.setSetpoint(rotations);
   }
   public void addWantedPosition(double rotations) 
   {
      intake_controller.setSetpoint(intake_controller.getSetpoint() + rotations);
   }
   public void getSetpoint(double rotations)
   {
      intake_controller.getSetpoint();
   }
    
}
