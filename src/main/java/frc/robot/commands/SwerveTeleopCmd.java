package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
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
    private final Supplier<Boolean> slowModeFunction;
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter turningLimiter;
    private final double inputCurveExponent = 3;
    private boolean wasTurningLastFrame = false;
    
    private double turningSetpoint;
    private PID thetaLockController;

    private boolean prevFieldOrientedInput = false;
    private boolean fieldOrientedActive = true;


    public SwerveTeleopCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedToggleFunction, Supplier<Boolean> slowModeFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedToggleFunction = fieldOrientedToggleFunction;
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
        turningSetpoint = swerveSubsystem.getHeading();
    }

    @Override
    public void execute() {
        double xInput = -xSpdFunction.get();
        double yInput = -ySpdFunction.get();
        double turningInput = -turningSpdFunction.get();

        if (fieldOrientedToggleFunction.get() && !prevFieldOrientedInput)
            fieldOrientedActive = !fieldOrientedActive;
        prevFieldOrientedInput = fieldOrientedToggleFunction.get();
        boolean slowMode = slowModeFunction.get();

        xInput = Math.abs(xInput) > OIConstants.kDeadband ? xInput : 0.0;
        yInput = Math.abs(yInput) > OIConstants.kDeadband ? yInput : 0.0;
        turningInput = Math.abs(turningInput) > OIConstants.kDeadband ? turningInput : 0.0;

        xInput = xInput > 0 ? Math.pow(Math.abs(xInput), inputCurveExponent) : -Math.pow(Math.abs(xInput), inputCurveExponent);
        yInput = yInput > 0 ? Math.pow(Math.abs(yInput), inputCurveExponent) : -Math.pow(Math.abs(yInput), inputCurveExponent);
        double xSpeed = xInput * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        double ySpeed = yInput * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        
        double turningSpeed;
        if (Math.abs(turningInput) > 0.0) { // this code is after deadband
            // driver is actively telling the robot to turn

            turningSetpoint = swerveSubsystem.getHeading() + turningInput * (0.5 * Math.PI);
            turningSetpoint = turningSetpoint % (2 * Math.PI);
            wasTurningLastFrame = true;
            // create the turning speed based on the rotation lock controller
            turningSpeed = thetaLockController.calculate(swerveSubsystem.getHeading(), turningSetpoint);
            if (Math.abs(turningSpeed) > DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond) { // clamp turning speed
                turningSpeed = DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * turningSpeed / Math.abs(turningSpeed);
            }
        }
        else if (Math.abs(xInput) > 0.0 || Math.abs(yInput) > 0.0) {
            // driver is not telling the robot to turn, but is translating and will need rotation lock

            // if we were turning last frame, update the setpoint to our current heading
            // if not, don't modify the setpoint
            if (wasTurningLastFrame) {
                turningSetpoint = swerveSubsystem.getHeading();
                wasTurningLastFrame = false;
            }
            turningSpeed = thetaLockController.calculate(swerveSubsystem.getHeading(), turningSetpoint);
            if (Math.abs(turningSpeed) > DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond) { // clamp turning speed
                turningSpeed = DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * turningSpeed / Math.abs(turningSpeed);
            }
        }
        else {
            // robot is stopped, we don't need to apply the rotation lock controller and speed should be 0.0

            if (wasTurningLastFrame) {
                turningSetpoint = swerveSubsystem.getHeading();
                wasTurningLastFrame = false;
            }
            turningSpeed = 0.0;
        }

        // limit translational and rotational acceleration
        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
        turningSpeed = turningLimiter.calculate(turningSpeed);

        if (slowMode) {
            xSpeed = xSpeed * 0.33;
            ySpeed = ySpeed * 0.33;
            turningSpeed = turningSpeed * 0.33;
        }

        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedActive) {
            // relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);

        if (Constants.dashboardDebugMode)
            SmartDashboard.putString("Teleop speeds", chassisSpeeds.toString());
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
