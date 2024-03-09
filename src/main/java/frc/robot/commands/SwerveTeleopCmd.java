package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.PID;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveTeleopCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedToggleFunction;
    private final Supplier<Boolean> resetForwardHeadingFunction;
    private final Supplier<Boolean> slowModeFunction;
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter turningLimiter;
    private final double inputCurveExponent = 3;
    private boolean wasTurningLastFrame = false;
    
    private double turningSetpoint;
    private PID thetaLockController;

    private boolean prevFieldOrientedInput = false;
    private boolean prevResetHeadingInput = false;
    private boolean fieldOrientedActive = true;


    public SwerveTeleopCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedToggleFunction, Supplier<Boolean> resetForwardHeadingFunction,
            Supplier<Boolean> slowModeFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedToggleFunction = fieldOrientedToggleFunction;
        this.resetForwardHeadingFunction = resetForwardHeadingFunction;
        this.slowModeFunction = slowModeFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationMetersPerSecondSquared);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationMetersPerSecondSquared);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationRadiansPerSecondSquared);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        thetaLockController = new PID(Constants.DriveConstants.kPThetaLockTurning, Constants.DriveConstants.kIThetaLockTurning, Constants.DriveConstants.kDThetaLockTurning);
        thetaLockController.enableContinuousInput(-Math.PI, Math.PI);
        turningSetpoint = swerveSubsystem.getPoseAngleRad();
    }

    @Override
    public void execute() {
        // handle inputs
        double xInput = -xSpdFunction.get();
        double yInput = -ySpdFunction.get();
        double turningInput = -turningSpdFunction.get();
        xInput = Math.abs(xInput) > OIConstants.kDeadband ? xInput : 0.0;
        yInput = Math.abs(yInput) > OIConstants.kDeadband ? yInput : 0.0;
        turningInput = Math.abs(turningInput) > OIConstants.kDeadband ? turningInput : 0.0;
        xInput = xInput > 0 ? Math.pow(Math.abs(xInput), inputCurveExponent) : -Math.pow(Math.abs(xInput), inputCurveExponent);
        yInput = yInput > 0 ? Math.pow(Math.abs(yInput), inputCurveExponent) : -Math.pow(Math.abs(yInput), inputCurveExponent);

        if (fieldOrientedToggleFunction.get() && !prevFieldOrientedInput)
            fieldOrientedActive = !fieldOrientedActive;
        prevFieldOrientedInput = fieldOrientedToggleFunction.get();

        if (resetForwardHeadingFunction.get() && !prevResetHeadingInput) {
            Pose2d currentPose = swerveSubsystem.getPose2d();
            swerveSubsystem.resetOdometry(new Pose2d(currentPose.getX(), currentPose.getY(), new Rotation2d()));
            turningSetpoint = 0.0;
        }
        prevResetHeadingInput = resetForwardHeadingFunction.get();

        boolean slowMode = slowModeFunction.get();

        // calculate translational speeds
        double xSpeed = xInput * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        double ySpeed = yInput * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

        // limit translational acceleration
        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
        
        // calculate rotational speed
        double turningSpeed;
        if (Math.abs(turningInput) > 0.0) { // this code is after deadband
            // driver is actively telling the robot to turn
            wasTurningLastFrame = true;
            turningSpeed = turningInput * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        }
        else {
            // driver is not telling the robot to turn

            // if we were turning last frame, update the setpoint to our current heading
            // if not, don't modify the setpoint
            if (wasTurningLastFrame) {
                turningSetpoint = swerveSubsystem.getPoseAngleRad();
                wasTurningLastFrame = false;
            }

            // if drivetrain is moving above threshold, apply theta lock, else, turning speed is 0
            ChassisSpeeds chassisSpeeds = swerveSubsystem.getChassisSpeeds();
            if (
            Math.abs(chassisSpeeds.vxMetersPerSecond) > 0.1 ||
            Math.abs(chassisSpeeds.vyMetersPerSecond) > 0.1 ||
            Math.abs(chassisSpeeds.omegaRadiansPerSecond) > 0.1
            ) {
                turningSpeed = thetaLockController.calculate(swerveSubsystem.getPoseAngleRad(), turningSetpoint);
            }
            else {
                turningSpeed = 0.0;
            }
        }
        
        // limit rotational acceleration, then limit max rotation speed
        turningSpeed = turningLimiter.calculate(turningSpeed);
        if (Math.abs(turningSpeed) > DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond) { // clamp turning speed
            turningSpeed = DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * turningSpeed / Math.abs(turningSpeed);
        }

        // apply slow mode
        if (slowMode) {
            xSpeed = xSpeed * 0.15;
            ySpeed = ySpeed * 0.15;
            turningSpeed = turningSpeed * 0.15;
        }

        // apply field oriented control if active
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedActive) {
            // speeds were relative to field, so convert them back to robot relative
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getPoseRotation2d());
        } else {
            // speeds are relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // pass speeds to swerve subsystem
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);

        SmartDashboard.putString("Debug/Teleop Speeds", chassisSpeeds.toString());
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
