package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.PID;
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
    private PID thetaLockController;

    public static boolean slowJoe = false;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        wasTurningLastFrame = false;
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
        // 1. Get real-time joystick inputs
        double xSpeed = -xSpdFunction.get();
        double ySpeed = -ySpdFunction.get();
        double turningSpeed = -turningSpdFunction.get();

        // if (Math.abs(turningSpeed) > 0.1) {
        //     turningSetpoint = swerveSubsystem.getHeading() + turningSpeed * (Math.PI);
        //     wasTurningLastFrame = true;
        // }
        // else if (wasTurningLastFrame) {
        //     turningSetpoint =  swerveSubsystem.getHeading();
        //     wasTurningLastFrame = false;
        // }


        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        if (xSpeed > 0){
            xSpeed = Math.pow(Math.abs(xSpeed), 3);
        }
        else {
            xSpeed = -Math.pow(Math.abs(xSpeed), 3);
        }

        if (ySpeed > 0){
            ySpeed = Math.pow(Math.abs(ySpeed), 3);
        }

        else{
            ySpeed = -Math.pow(Math.abs(ySpeed), 3);
        }
        


        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        if (Math.abs(turningSpeed) < 0.05)
            turningSpeed = 0.0;
        // turningSpeed = thetaLockController.calculate(swerveSubsystem.getHeading(), turningSetpoint);
        // double turningSpeedMag = Math.abs(turningSpeed);
        // if (turningSpeedMag > DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond) {
        //     turningSpeed = DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * turningSpeed / turningSpeedMag;
        // }


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

        SmartDashboard.putString("teleop chassis speeds", chassisSpeeds.toString());

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.teleopControl(moduleStates);
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
