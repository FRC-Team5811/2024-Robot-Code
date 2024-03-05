package frc.robot.subsystems;

import java.io.Console;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class Intake extends SubsystemBase {
    
    public final WPI_VictorSPX intakeMotor = new WPI_VictorSPX(16);
    public final double intakeSpeed = 1;

    public Intake() {
        
    }

    public void pull() {
        intakeMotor.set(intakeSpeed);
    }

    public void push() {
        intakeMotor.set(-intakeSpeed);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }


}