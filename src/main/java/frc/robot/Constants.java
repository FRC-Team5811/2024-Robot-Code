package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public final class Constants {

    public static boolean dashboardDebugMode = false;

    public static final double kIntakeMax = 0.5;
    public static final double kIntakeMin = -0.8;

    public static final int kLinearMotorPort = 12;
    public static final int kArticulationMotorPort1 = 2;
    public static final int kArticulationMotorPort2 = 3;
    public static final int kIntakeMotorPort = 1;

    public static final double kMaxAngleMotorSpeed = 0.15;
    public static final double kMaxLinearMotorSpeed = 0.8;

    public static final double kArmAngleMin = -70;
    public static final double kArmAngleMax = 190;
    public static final double kArmAngleBuffer = 5;

    public static final double kArmLinearMax = 24.625;
    public static final double kArmLinearMin = 0;
    public static final double kArmLinearBuffer = 1;

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 5.8462;
        public static final double kTurningMotorGearRatio = 1 / 21.42857;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
        public static final double kDTurning = 0.0;
        public static final double kITurning = 0.0;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(22.25);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(22.25);
        // Distance between front and back wheels

        // Positive X = front of robot, Positive Y = left of robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                // new Translation2d(-kTrackWidth / 2, kWheelBase / 2),
                // new Translation2d(kTrackWidth / 2, kWheelBase / 2),
                // new Translation2d(-kTrackWidth / 2, -kWheelBase / 2),
                // new Translation2d(kTrackWidth / 2, -kWheelBase / 2)
                new Translation2d(Units.inchesToMeters(6.5), Units.inchesToMeters(11.5)), // front left
                new Translation2d(Units.inchesToMeters(6.5), -Units.inchesToMeters(11.5)), // front right
                new Translation2d(-Units.inchesToMeters(11.5), Units.inchesToMeters(11.5)), // back left
                new Translation2d(-Units.inchesToMeters(11.5), -Units.inchesToMeters(11.5)) // back right
                );

        public static final int kFrontLeftDriveMotorPort = 6;
        public static final int kBackLeftDriveMotorPort = 7;
        public static final int kFrontRightDriveMotorPort = 5;
        public static final int kBackRightDriveMotorPort = 4;

        public static final int kFrontLeftTurningMotorPort = 10;
        public static final int kBackLeftTurningMotorPort = 11;
        public static final int kFrontRightTurningMotorPort = 9;
        public static final int kBackRightTurningMotorPort = 8;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 8;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 9;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 7;
        public static final int kBackRightDriveAbsoluteEncoderPort = 6;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.805; //4.550; // 4.562; // 2.8463615551277313;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 2.996; //1.240 + 2.2014 - 0.322; // 1.236; // 0.8776053434371365;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 2.412 - 1.571; //1.149 - 0.923; // 1.317; // 6.149893967820428;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 3.369 + 1.571; //5.566; // 5.575; // 0.43207125951125236;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * 0.6;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond * 0.5;  
        public static final double kTeleDriveMaxAccelerationMetersPerSecondSquared = 30;
        public static final double kTeleDriveMaxAngularAccelerationRadiansPerSecondSquared = 8 * Math.PI;

        public static final double kPThetaLockTurning = 2;
        public static final double kIThetaLockTurning = 0;
        public static final double kDThetaLockTurning = 0;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 6;
        public static final double kMaxAccelerationMetersPerSecondSquared = 5;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 32 * Math.PI;
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 2;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
        public static final TrapezoidProfile.Constraints kXControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxSpeedMetersPerSecond,
                        kMaxAccelerationMetersPerSecondSquared);
        public static final TrapezoidProfile.Constraints kYControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxSpeedMetersPerSecond,
                        kMaxAccelerationMetersPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kManipControllerPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 2;
        public static final int kDriverFieldOrientedButtonIdx = 2;

        public static final int kManipRotateAxis = 1;
        public static final int kManipLinearAxis = 3;

        public static final double kDeadband = 0.05;

        //Buttons
        public static final int XButton = 1;
        public static final int YButton = 4;
        public static final int AButton = 2;
        public static final int BButton = 3;
        public static final int StartButton = 10;
        public static final int BackButton = 9;
        public static final int LeftAnalogButton = 11;
        public static final int RightAnalogButton = 12;
        public static final int LeftBumperButton = 5;
        public static final int RightBumperButton = 6;
        public static final int LeftTriggerButton = 7;
        public static final int RightTriggerButton = 8;
    }

    public static class FieldConstants {
        public static final double length = Units.feetToMeters(54);
        public static final double width = Units.feetToMeters(27);
    }
    
    public static class VisionConstants {
        public static final Transform3d robotToCam =
                new Transform3d(
                        new Translation3d(Units.inchesToMeters(4), Units.inchesToMeters(6), Units.inchesToMeters(11)),
                        new Rotation3d(
                                0, 0,
                                0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
        // from center.
        public static final String cameraName = "BONDS5811";
    }
}
