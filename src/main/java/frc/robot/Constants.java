package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static boolean dashboardDebugMode = true;

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

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * 0.9;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond * 0.5;  
        public static final double kTeleDriveMaxAccelerationMetersPerSecondSquared = 30;
        public static final double kTeleDriveMaxAngularAccelerationRadiansPerSecondSquared = 8 * Math.PI;
        public static final double SlowModeSpeedPercent = 0.3;

        public static final double kPThetaLockTurning = 3;
        public static final double kIThetaLockTurning = 0;
        public static final double kDThetaLockTurning = 0.5;
    }

    public static final class ManipConstants {
        
        public static final double shooterMaxRPM = 5500;
        public static final double shooterSpeakerRPMLower = 4000;
        public static final double shooterSpeakerRPMUpper = 4000;
        public static final double shooterControlP = 4;
        public static final double shooterControlI = 0;
        public static final double shooterControlD = 0.5;


        public static final double shooterAnalogMaxRate = 1.0;

    }
    
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 6;
        public static final double kMaxAccelerationMetersPerSecondSquared = 10;
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
        
        public static final Pose2d start1Pose = new Pose2d(0.81, 4.38, Rotation2d.fromDegrees(-60)); //DNC
        public static final Pose2d start2Pose = new Pose2d(1.577, 5.6, new Rotation2d()); //DNC
        public static final Pose2d start3Pose = new Pose2d(0.49, 7.064, Rotation2d.fromDegrees(60)); //DNC
        public static final Pose2d note2Pose = new Pose2d(2.1, 5.588, new Rotation2d()); //DNC
        public static final Pose2d note3Pose = new Pose2d(2.1, 7.088, new Rotation2d()); //DO NOT CHANGE
        public static final Pose2d note1Pose = new Pose2d(2.1, 4.0, new Rotation2d()); //DNC
        public static final Pose2d note4Pose = new Pose2d(8.2, 0.755, new Rotation2d());
        public static final Pose2d note5Pose = new Pose2d(8.2, 2.443, new Rotation2d());
        public static final Pose2d note6Pose = new Pose2d(8.2, 4.131, new Rotation2d());
        public static final Pose2d note7Pose = new Pose2d(8.2, 5.9, new Rotation2d());
        public static final Pose2d note8Pose = new Pose2d(8.2, 7.35, new Rotation2d()); //DO NOT CHANGE
        public static final Pose2d mid1ForwardPose = new Pose2d(5.482, 1.575, new Rotation2d());
        public static final Pose2d mid1BackPose = new Pose2d(1.905, 1.575, new Rotation2d());
        public static final Pose2d mid2ForwardPose = new Pose2d(5.842, 6.7, new Rotation2d());
        public static final Pose2d mid3SneakBackPose = new Pose2d(1.905, 7.747, new Rotation2d());
        public static final Pose2d mid3SneakForwardPose = new Pose2d(6.477, 7.747, new Rotation2d());
        
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kManipControllerPort = 1;
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

        public static final int POVUpButtonAngle = 0;
        public static final int POVDownButtonAngle = 180;
        public static final int POVRightButtonAngle = 90;
        public static final int POVLeftButtonAngle = 270;

        // drive bindings
        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 2;
        public static final int driverFieldOrientedButton = AButton;
        public static final int driverResetHeadingButton = RightTriggerButton;
        public static final int driverSlowModeButton = LeftTriggerButton;

        // intake bindings
        public static final int intakeManualButton = RightBumperButton;
        public static final int intakeSequenceButton = AButton;
        

        // indexer bindings
        public static final int ampScoreManualButton = LeftTriggerButton;
        public static final int speakerScoreManualButton = RightTriggerButton;

        // shooter bindings
        public static final double shooterCustomDeadband = 0.2;
        public static final int shooterManualAxis = 1;
        public static final int shooterManualButton = YButton;
        public static final int ampShotSequenceButton = BButton;
        public static final int speakerShotSequenceButton = XButton;
        public static final int shooterRampUpButton = RightTriggerButton;
    }

    public static class FieldConstants {
        public static final double length = Units.feetToMeters(54);
        public static final double width = Units.feetToMeters(27);
    }
    
    public static class VisionConstants {
        public static final Transform3d robotToCamera =
                new Transform3d(
                        new Translation3d(Units.inchesToMeters(4), Units.inchesToMeters(6), Units.inchesToMeters(11)),
                        new Rotation3d(
                                0, 0,
                                0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
        // from center.
        public static final String cameraName = "Camera0";
        public static final int gamePiecePipeline = 0;
        public static final int aprilTagPipeline = 1;
    }
}
