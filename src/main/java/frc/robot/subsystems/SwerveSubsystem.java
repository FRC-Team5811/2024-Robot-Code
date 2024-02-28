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
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);
        frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);
        backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);
        backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);
        
        gyro = new AHRS(SPI.Port.kMXP);
        zeroHeading();

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                //zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    // should be counter-clockwise positive!!
    public double getHeading() {
        // return odometer.getPoseMeters().getRotation().getRadians();
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
        // frontLeft.resetEncoders();
        // frontRight.resetEncoders();
        // backLeft.resetEncoders();
        // backRight.resetEncoders();
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
        printEncoders();

        frontLeft.onPeriodic();
        frontRight.onPeriodic();
        backLeft.onPeriodic();
        backRight.onPeriodic();

        swerveModulePositions[0] = frontLeft.getPosition();
        swerveModulePositions[1] = frontRight.getPosition();
        swerveModulePositions[2] = backLeft.getPosition();
        swerveModulePositions[3] = backRight.getPosition();

        odometer.update(getRotation2d(), swerveModulePositions);
        SmartDashboard.putString("position", getPose().toString());
        SmartDashboard.putNumber("Robot Heading", getHeading());

        SmartDashboard.putNumber("turn enc 0: ", backRight.getTurnEncoder());
        SmartDashboard.putNumber("turn enc 1: ", frontRight.getTurnEncoder());
        SmartDashboard.putNumber("turn enc 2: ", frontLeft.getTurnEncoder());
        SmartDashboard.putNumber("turn enc 3: ", backLeft.getTurnEncoder());

        
        if (locked)
            this.setModulesLock();
        
    }

    public void printEncoders() {
        
        SmartDashboard.putNumber("Front left abs radians", frontLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Front right abs radians", frontRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Back left abs radians", backLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Back right abs radians", backRight.getAbsoluteEncoderRad());
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

    public void teleopControl(SwerveModuleState[] desiredStates) {
        if (locked == false){
            setModuleStates(desiredStates);

        }
        else{
            setModulesLock();

        }
    }

    public void toggleLock() {
        locked = !locked;
        if(locked == true){
            idle(IdleMode.kBrake);
        }
        if(locked == false){
            idle(IdleMode.kCoast);
        }
    }

    public void idle(IdleMode idleMode){
        frontRight.idle(idleMode);
        frontLeft.idle(idleMode);
        backRight.idle(idleMode);
        backLeft.idle(idleMode);
    }

    public void setModulesLock() {
        SwerveModuleState[] lockStates = new SwerveModuleState[4];
        lockStates[0] = new SwerveModuleState(0.01, Rotation2d.fromDegrees(45));
        lockStates[1] = new SwerveModuleState(0.01, Rotation2d.fromDegrees(-45));
        lockStates[2] = new SwerveModuleState(0.01, Rotation2d.fromDegrees(-45));
        lockStates[3] = new SwerveModuleState(0.01, Rotation2d.fromDegrees(45));
        setModuleStates(lockStates);
    }
}
