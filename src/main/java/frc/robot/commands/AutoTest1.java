package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoTest1 extends SequentialCommandGroup{ 

    private final Pose2d startPose = new Pose2d(0, 0, new Rotation2d());
    private final Pose2d rampUpStartPose = new Pose2d();
    private final Pose2d closeNote1Pose = new Pose2d(1.1, 0.1, new Rotation2d());
    private final Pose2d closeNote2Pose = new Pose2d(1.1, 1.6, new Rotation2d());
    private final Pose2d closeNote3Pose = new Pose2d(1.1, -1.65, new Rotation2d());

    public AutoTest1(SwerveSubsystem swerveSubsystem, Intake intake, Indexer indexer, Shooter shooter) {

        addCommands(
            new ResetSwervePoseCmd(swerveSubsystem, startPose),
            new AutoSpeakerSequence(shooter, indexer),
            new AutoDriveToPoint(swerveSubsystem, closeNote1Pose, false),
            new ParallelCommandGroup(
                new AutoIntakeSequence(intake, indexer, true),
                new AutoDriveToPoint(swerveSubsystem, closeNote1Pose.transformBy(new Transform2d(0.3, 0, new Rotation2d())),  0.05, 5, 1, true)
                ),
            new AutoDriveToPoint(swerveSubsystem, startPose.transformBy(new Transform2d(-0.1, 0, new Rotation2d())), true),
            new AutoSpeakerSequence(shooter, indexer),
            //note 3
            new AutoDriveToPoint(swerveSubsystem, closeNote2Pose, false),
            new ParallelCommandGroup(
                new AutoIntakeSequence(intake, indexer, true),
                new AutoDriveToPoint(swerveSubsystem, closeNote2Pose.transformBy(new Transform2d(0.3, 0, new Rotation2d())),  0.05, 5, 1, true)
                ),
            new AutoDriveToPoint(swerveSubsystem, startPose.transformBy(new Transform2d(-0.1, 0, new Rotation2d())), true),
            new AutoSpeakerSequence(shooter, indexer),
            //note 4
            new AutoDriveToPoint(swerveSubsystem, closeNote3Pose, false),
            new ParallelCommandGroup(
                new AutoIntakeSequence(intake, indexer, true),
                new AutoDriveToPoint(swerveSubsystem, closeNote3Pose.transformBy(new Transform2d(0.2, 0, new Rotation2d())),  0.05, 5, 1, true)
                ),
            new AutoDriveToPoint(swerveSubsystem, startPose.transformBy(new Transform2d(-0.1, 0, new Rotation2d())), true),
            new AutoSpeakerSequence(shooter, indexer)
        );
    }
}