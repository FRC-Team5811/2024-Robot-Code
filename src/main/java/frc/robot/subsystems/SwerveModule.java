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
    private final SparkPIDController driveMotorPidController;

    private final DutyCycleEncoder absoluteEncoder;
    private final int absoluteEncoderId;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
    private final String moduleName;
    private double cycles;
    

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

        driveMotorPidController = driveMotor.getPIDController();
        driveMotorPidController.setP(Constants.ModuleConstants.kPDriving);
        driveMotorPidController.setD(Constants.ModuleConstants.kDDriving);
        driveMotorPidController.setI(Constants.ModuleConstants.kIDriving);
        driveMotorPidController.setFF(1/5676);
        driveMotorPidController.setOutputRange(-1, 1);


        cycles = 0;
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
        //Debugging Code for Tuning Driving PID
        if (cycles % 50 == 0) {
            double newP = SmartDashboard.getNumber("Debug/p value", Constants.ModuleConstants.kPDriving);
            double newD = SmartDashboard.getNumber("Debug/d value", Constants.ModuleConstants.kPDriving);
            SmartDashboard.putNumber("Debug/p value", newP);
            SmartDashboard.putNumber("Debug/d value", newD);
            double newFF = SmartDashboard.getNumber("Debug/FF value", Constants.ModuleConstants.kFFDriving);
            SmartDashboard.putNumber("Debug/FF value", newFF);
            driveMotorPidController.setFF(newFF);
            driveMotorPidController.setP(newP);
            driveMotorPidController.setD(newD);
        }
        cycles += 1;
        // end of debugging code
        state = betterOptimize(state, getPosition().angle);
        double turningSetpoint = state.angle.getRadians();
        double drivingSetpoint = state.speedMetersPerSecond*60 / (4*2.54*Math.PI / 100);
        //driveMotor.set(state.speedMetersPerSecond / Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotorPidController.setReference(turningSetpoint, CANSparkMax.ControlType.kPosition);
        driveMotorPidController.setReference(drivingSetpoint, CANSparkMax.ControlType.kVelocity);
        SmartDashboard.putString("Debug/Module [" + moduleName + "] desired state", state.toString());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}