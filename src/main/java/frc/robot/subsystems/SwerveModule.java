package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.PID;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    // private final PIDController turningPidController;
    private final SparkPIDController turningMotorPidController;

    private final DutyCycleEncoder absoluteEncoder;
    private final int absoluteEncoderId;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
    private final String moduleName;
    
    private final PID wheelSpeedPID = new PID(1, 0, 0.1);
    private double wheelPIDOutput = 0;


    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed, String moduleName) {

        this.moduleName = moduleName;
        
        this.absoluteEncoderId = absoluteEncoderId;
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new DutyCycleEncoder(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
        turningEncoder.setPosition(getAbsoluteEncoderRad());

        // turningPidController = new PIDController(ModuleConstants.kPTurning, 0, ModuleConstants.kDTurning);
        // turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        
        turningMotorPidController = turningMotor.getPIDController();
        turningMotorPidController.setPositionPIDWrappingEnabled(true);
        turningMotorPidController.setPositionPIDWrappingMinInput(-Math.PI);
        turningMotorPidController.setPositionPIDWrappingMaxInput(Math.PI);
        turningMotorPidController.setP(Constants.ModuleConstants.kPTurning);
        turningMotorPidController.setD(Constants.ModuleConstants.kDTurning);
        turningMotorPidController.setI(Constants.ModuleConstants.kITurning);
    }

    public void onPeriodic() {
        double motorEncoderAngle = turningEncoder.getPosition();// % (2 * Math.PI);
        if (Math.abs(motorEncoderAngle) > 4 * Math.PI) {
            turningEncoder.setPosition(getAbsoluteEncoderRad());
        }

        SmartDashboard.putNumber("Debug/" + moduleName + " mag enc rads", getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Debug/" + moduleName + " neo enc diff", motorEncoderAngle - getAbsoluteEncoderRad());
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition(); 
    }

    public void setIdleMode(IdleMode idleMode){
        driveMotor.setIdleMode(idleMode);
        turningMotor.setIdleMode(idleMode);
    }
    
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getDriveDistance() {
        return driveEncoder.getPosition();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition();
        
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public double getAbsoluteEncoderDegrees() {
        double angle = absoluteEncoder.getAbsolutePosition();
        
        angle *= 360;
        angle -= (absoluteEncoderOffsetRad * 180 / Math.PI);
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public void printAbsEncoder() {
        
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveDistance(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(getTurningPosition()));
    }

    public static SwerveModuleState betterOptimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double angle0 = currentAngle.getDegrees();
        double angle1 = desiredState.angle.getDegrees();
        var minDelta = (180 - Math.abs(Math.abs(angle1 - angle0) - 180)) % 360;
        if (minDelta > 90.0) {
        return new SwerveModuleState(
            -desiredState.speedMetersPerSecond,
            desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
        } else {
            return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        }
    }

    public double getTurnEncoder() {
        return turningEncoder.getPosition();
    }


    public void setDesiredState(SwerveModuleState state) {
        state = betterOptimize(state, getPosition().angle);
        double turningSetpoint = state.angle.getRadians();
        if (!Constants.DriveConstants.oldWheelSpeedControl) {
            wheelPIDOutput = wheelSpeedPID.calculate(getDriveVelocity(), state.speedMetersPerSecond);
            driveMotor.set(wheelPIDOutput + (state.speedMetersPerSecond / Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond));
        }
        else driveMotor.set(state.speedMetersPerSecond / Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotorPidController.setReference(turningSetpoint, CANSparkMax.ControlType.kPosition);
        SmartDashboard.putString("Debug/Module [" + moduleName + "] desired state", state.toString());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}