package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.PID;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveTeleopCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final Supplier<Boolean> resetForwardHeadingFunction;
    private final Supplier<Double> slowModeFunction;
    private final Supplier<Double> breakFunction;
    private final Supplier<Boolean> forwardSetpointFunction;
    private final Supplier<Boolean> rightSetpointFunction;
    private final Supplier<Boolean> backSetpointFunction;
    private final Supplier<Boolean> leftSetpointFunction;
    private final Supplier<Boolean> shuttleSetpointFunction;

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
            Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> resetForwardHeadingFunction,
            Supplier<Double> slowModeFunction, Supplier<Double> breakFunction, Supplier<Boolean> forwardSetpointFunction, Supplier<Boolean> rightSetpointFunction, 
            Supplier<Boolean> backSetpointFunction, Supplier<Boolean> leftSetpointFunction, Supplier<Boolean> shuttleSetpointFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.resetForwardHeadingFunction = resetForwardHeadingFunction;
        this.slowModeFunction = slowModeFunction;
        this.breakFunction = breakFunction;
        this.forwardSetpointFunction = forwardSetpointFunction;
        this.rightSetpointFunction = rightSetpointFunction;
        this.backSetpointFunction = backSetpointFunction;
        this.leftSetpointFunction = leftSetpointFunction;
        this.shuttleSetpointFunction = shuttleSetpointFunction;
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

        if (resetForwardHeadingFunction.get() && !prevResetHeadingInput) {
            Pose2d currentPose = swerveSubsystem.getPose2d();
            swerveSubsystem.resetOdometry(new Pose2d(currentPose.getX(), currentPose.getY(), new Rotation2d()));
            turningSetpoint = 0.0;
        }
        prevResetHeadingInput = resetForwardHeadingFunction.get();

        // boolean slowMode = slowModeFunction.get();
        // Get brake percent
        double breakingPercent = 1.0 - 0.9*breakFunction.get();

        // calculate translational speeds
        double xSpeed = xInput * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        double ySpeed = yInput * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

        // limit translational acceleration
        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
        
        // calculate rotational speed
        double turningSpeed;
        //setpoints
        SwerveSubsystem.isShuttlePose = false;
        if (forwardSetpointFunction.get()) {
            turningSetpoint = 0;
            turningSpeed = thetaLockController.calculate(swerveSubsystem.getPoseAngleRad(), turningSetpoint);
        }
        else if (rightSetpointFunction.get()) {
            turningSetpoint = Math.PI;
            turningSpeed = thetaLockController.calculate(swerveSubsystem.getPoseAngleRad(), turningSetpoint);
        }
        else if (backSetpointFunction.get()) {
            turningSetpoint = 3*Math.PI/2;
            turningSpeed = thetaLockController.calculate(swerveSubsystem.getPoseAngleRad(), turningSetpoint);
        }
        else if (leftSetpointFunction.get()) {
            turningSetpoint = Math.PI/2;
            turningSpeed = thetaLockController.calculate(swerveSubsystem.getPoseAngleRad(), turningSetpoint);
        }
        else if (shuttleSetpointFunction.get()) {
            turningSetpoint = Constants.DriveConstants.shuttleSetpointFunction;
            if (Robot.allianceColor == Alliance.Red) turningSetpoint = -Math.PI - turningSetpoint;
            turningSpeed = thetaLockController.calculate(swerveSubsystem.getPoseAngleRad(), turningSetpoint);
            SwerveSubsystem.isShuttlePose = ((swerveSubsystem.getPoseAngleRad() > turningSetpoint - 5*Math.PI/180)
                                            && (swerveSubsystem.getPoseAngleRad() < turningSetpoint + 5*Math.PI/180));
            SmartDashboard.putNumber("Debug/setpoint degrees", turningSetpoint*180/Math.PI);
        }
        //for all else other than setpoints
        else if (Math.abs(turningInput) > 0.0) { // this code is after deadband
            // driver is actively telling the robot to turn
            wasTurningLastFrame = true;
            turningSpeed = turningInput * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * DriveConstants.turningSpeedPercent;
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
            if (
            Math.abs(xInput) > 0.1 ||
            Math.abs(yInput) > 0.1
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
        if (slowModeFunction.get() > 0.1
        || fieldOrientedFunction.get()) {
            xSpeed = xSpeed * Constants.DriveConstants.SlowModeSpeedPercent;
            ySpeed = ySpeed * Constants.DriveConstants.SlowModeSpeedPercent;
            turningSpeed = turningSpeed * Constants.DriveConstants.SlowModeSpeedPercent;
        }

        //apply break
        xSpeed = xSpeed * breakingPercent;
        ySpeed = ySpeed * breakingPercent;
        turningSpeed = turningSpeed * breakingPercent;

        // apply field oriented control if active
        ChassisSpeeds chassisSpeeds;
        if (!fieldOrientedFunction.get()) {
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

    private boolean isShuttlePose() {
        return (shuttleSetpointFunction.get() 
        && (swerveSubsystem.getPoseAngleRad() > Constants.DriveConstants.shuttleSetpointFunction - 5*Math.PI/180)
        && (swerveSubsystem.getPoseAngleRad() < Constants.DriveConstants.shuttleSetpointFunction + 5*Math.PI/180));
    }
}
