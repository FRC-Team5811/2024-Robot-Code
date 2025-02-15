package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
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
    public static boolean isShuttlePose = false;

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

        SmartDashboard.putData("Swerve subsystem", this);
    }

    public void zeroGyroAngle() {
        gyro.reset();
    }

    // should be counter-clockwise positive!!
    private double getGyroAngleRad() {
        return Math.IEEEremainder(-gyro.getYaw(), 360) * Math.PI / 180;
    }

    private Rotation2d getGyroRotation2d() {
        return Rotation2d.fromRadians(getGyroAngleRad());
    }

    public double getPoseAngleRad() {
        return odometer.getPoseMeters().getRotation().getRadians();
    }

    public Rotation2d getPoseRotation2d() {
        return odometer.getPoseMeters().getRotation();
    }
    
    public Pose2d getPose2d() {
        return odometer.getPoseMeters();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(
            frontLeft.getState(), frontRight.getState(),
            backLeft.getState(), backRight.getState()
            );
    }

    public double getFrontLeftDriveVelocity() {
        return frontLeft.getDriveVelocity();
    }

    public void resetOdometry(Pose2d pose) {
        swerveModulePositions[0] = frontLeft.getPosition();
        swerveModulePositions[1] = frontRight.getPosition();
        swerveModulePositions[2] = backLeft.getPosition();
        swerveModulePositions[3] = backRight.getPosition();
        odometer.resetPosition(getGyroRotation2d(), swerveModulePositions, pose);
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

        odometer.update(getGyroRotation2d(), swerveModulePositions);

        SmartDashboard.putString("Debug/Robot pose", getPose2d().toString());
        SmartDashboard.putString("Debug/Robot speeds", getChassisSpeeds().toString());
        SmartDashboard.putNumber("Driver/XPose", getPose2d().getX());
        SmartDashboard.putNumber("Driver/YPose", getPose2d().getY());
        SmartDashboard.putNumber("Driver/ThetaPose", getPose2d().getRotation().getDegrees());
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

    public boolean isShuttlePose() {
        return isShuttlePose;
    }
}
