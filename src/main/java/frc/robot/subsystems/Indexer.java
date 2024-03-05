package frc.robot.subsystems;

import java.io.Console;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
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

public class Indexer extends SubsystemBase {

    public final CANSparkMax indexerMotor = new CANSparkMax(15, MotorType.kBrushless);
    private DigitalInput limitSwitch = new DigitalInput(0);
    public final double indexerAmpShotMotorSpeed = 0.3;
    public final double indexerSpeakerShotMotorSpeed = 0.3;
    public final double indexerPushSpeed = -1;
    public final double indexerPullSpeed = 1;

    public Indexer () {
        indexerMotor.setInverted(true);
    }

    public void push() {
        indexerMotor.set(indexerPushSpeed);
    }

    public void pull() {
        indexerMotor.set(indexerPullSpeed);
    }

    public void ampScore() {
        indexerMotor.set(indexerAmpShotMotorSpeed);
    }

    public void speakerScore() {
        indexerMotor.set(indexerSpeakerShotMotorSpeed);
    }

    public void stop() {
        indexerMotor.stopMotor();
    }

    public boolean getLimitBool() {
        return false; // limitSwitch.get();
    }

}