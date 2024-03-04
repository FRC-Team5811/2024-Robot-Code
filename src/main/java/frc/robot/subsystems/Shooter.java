package frc.robot.subsystems;

import java.io.Console;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class Shooter extends SubsystemBase {
    public static final CANSparkMax motor0 = new CANSparkMax(12, MotorType.kBrushless);
    public static final RelativeEncoder motor0Encoder;
    public static final CANSparkMax motor1 = new CANSparkMax(13, MotorType.kBrushless);
    public static final RelativeEncoder motor1Encoder;
    public static final CANSparkMax motor2 = new CANSparkMax(14, MotorType.kBrushless);


    public static PIDController crapController0;
    public static PIDController crapController1;

    public Shooter () {

    }

}