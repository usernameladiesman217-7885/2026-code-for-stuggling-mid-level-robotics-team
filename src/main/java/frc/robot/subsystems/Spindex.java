
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

public class Spindex extends SubsystemBase{
    public SparkMax spindex_motor = new SparkMax(11, MotorType.kBrushless);
    public SparkMaxConfig spindex_config = new SparkMaxConfig();



    public Spindex()
    {
        spindex_config.inverted(true);
        spindex_config.idleMode(IdleMode.kCoast);
        spindex_motor.configure(spindex_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }
    public void setSpeed(double speed){
        spindex_motor.set(speed);
    }
}
