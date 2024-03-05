package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft;

    private final SwerveModule frontRight;

    private final SwerveModule backLeft;

    private final SwerveModule backRight;

    private final SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };

    final AHRS gyro;
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0), swerveModulePositions);

    private boolean locked = false;

    public SwerveSubsystem() {
        frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
            "frontLeft");
        frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
            "frontRight");
        backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
            "backLeft");
        backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
            "backRight");
        
        gyro = new AHRS(SPI.Port.kMXP);

        resetModuleEncoders();
        zeroHeading();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    // should be counter-clockwise positive!!
    public double getHeading() {
        return Math.IEEEremainder(-gyro.getYaw(), 360) * Math.PI /   180;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromRadians(getHeading());
    }
    
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
    }

    public void resetOdometry(Pose2d pose) {
        swerveModulePositions[0] = frontLeft.getPosition();
        swerveModulePositions[1] = frontRight.getPosition();
        swerveModulePositions[2] = backLeft.getPosition();
        swerveModulePositions[3] = backRight.getPosition();
        odometer.resetPosition(getRotation2d(), swerveModulePositions, pose);
    }

    public void resetModuleEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    @Override
    public void periodic() {
        frontLeft.onPeriodic();
        frontRight.onPeriodic();
        backLeft.onPeriodic();
        backRight.onPeriodic();

        swerveModulePositions[0] = frontLeft.getPosition();
        swerveModulePositions[1] = frontRight.getPosition();
        swerveModulePositions[2] = backLeft.getPosition();
        swerveModulePositions[3] = backRight.getPosition();

        odometer.update(getRotation2d(), swerveModulePositions);

        if (Constants.dashboardDebugMode) {
            SmartDashboard.putString("Robot pose", getPose().toString());
        }
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void setIdleMode(IdleMode idleMode) {
        frontRight.setIdleMode(idleMode);
        frontLeft.setIdleMode(idleMode);
        backRight.setIdleMode(idleMode);
        backLeft.setIdleMode(idleMode);
    }
}
