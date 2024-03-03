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

public class SwerveJoystickCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter turningLimiter;
    private boolean wasTurningLastFrame;


    private double turningSetpoint;
    private PIDController thetaLockController;

    public static boolean slowJoe = false;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationMetersPerSecondSquared);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationMetersPerSecondSquared);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationRadiansPerSecondSquared);
        wasTurningLastFrame = false;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        thetaLockController = new PIDController(Constants.DriveConstants.kPThetaLockTurning, Constants.DriveConstants.kIThetaLockTurning, Constants.DriveConstants.kDThetaLockTurning);
        thetaLockController.enableContinuousInput(-Math.PI, Math.PI);
        turningSetpoint = swerveSubsystem.getHeading();
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xInput = -xSpdFunction.get();
        double yInput = -ySpdFunction.get();
        double turningInput = -turningSpdFunction.get();
        
        // 2. Apply deadband
        xInput = Math.abs(xInput) > OIConstants.kDeadband ? xInput : 0.0;
        yInput = Math.abs(yInput) > OIConstants.kDeadband ? yInput : 0.0;
        turningInput = Math.abs(turningInput) > OIConstants.kDeadband ? turningInput : 0.0;

        if (xInput > 0){
            xInput = Math.pow(Math.abs(xInput), 3);
        }
        else {
            xInput = -Math.pow(Math.abs(xInput), 3);
        }

        if (yInput > 0){
            yInput = Math.pow(Math.abs(yInput), 3);
        }

        else{
            yInput = -Math.pow(Math.abs(yInput), 3);
        }

        double xSpeed = xInput * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        double ySpeed = yInput * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        
        // create a setpoint for the rotation the robot should turn towards
        if (Math.abs(turningInput) > 0.0) { // this code is after deadband
            turningSetpoint = swerveSubsystem.getHeading() + turningInput * (0.5 * Math.PI);
            turningSetpoint = turningSetpoint % (2 * Math.PI);
            wasTurningLastFrame = true;
        }
        else if (wasTurningLastFrame) {
            turningSetpoint = swerveSubsystem.getHeading();
            wasTurningLastFrame = false;
        }
        
        // create the turning speed based on the rotation lock controller
        double turningSpeed = thetaLockController.calculate(swerveSubsystem.getHeading(), turningSetpoint);
        if (Math.abs(turningSpeed) > DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond) { // clamp turning speed
            turningSpeed = DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * turningSpeed / Math.abs(turningSpeed);
        }

        // 3. Clamp x, y, and rotation accelerations to not have instantaneous motor throttle
        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
        turningSpeed = turningLimiter.calculate(turningSpeed);

        if (slowJoe) {
            xSpeed = xSpeed * 0.33;
            ySpeed = ySpeed * 0.33;
            turningSpeed = turningSpeed * 0.33;
        }

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        if (Constants.dashboardDebugMode)
            SmartDashboard.putString("Teleop speeds", chassisSpeeds.toString());

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
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
