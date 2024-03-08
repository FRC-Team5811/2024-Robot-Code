package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class S2n3n2n1 extends SequentialCommandGroup{ 

    public S2n3n2n1(SwerveSubsystem swerveSubsystem, Intake intake, Indexer indexer, Shooter shooter) {

        addCommands(
            //pre-loaded note
            new ResetSwervePoseCmd(swerveSubsystem, Constants.AutoConstants.start2Pose),
            new SpeakerSequence(shooter, indexer),
            //note 3
            new GrabAndSpeakerShoot(swerveSubsystem, intake, indexer, shooter, Constants.AutoConstants.note3Pose, Constants.AutoConstants.start2Pose),
            // new AutoDriveToPoint(swerveSubsystem, Constants.AutoConstants.note2Pose, false),
            // new ParallelCommandGroup(
            //     new AutoIntakeSequence(intake, indexer, true),
            //     new AutoDriveToPoint(swerveSubsystem, Constants.AutoConstants.note2Pose.transformBy(new Transform2d(0.3, 0, new Rotation2d())),  0.05, 5, 1, true)
            //     ),
            // new AutoDriveToPoint(swerveSubsystem, Constants.AutoConstants.startPose.transformBy(new Transform2d(-0.1, 0, new Rotation2d())), true),
            // new AutoSpeakerSequence(shooter, indexer),
            
            //note 2
            new GrabAndSpeakerShoot(swerveSubsystem, intake, indexer, shooter, Constants.AutoConstants.note2Pose, Constants.AutoConstants.start2Pose),
            // new AutoDriveToPoint(swerveSubsystem, Constants.AutoConstants.note3Pose, false),
            // new ParallelCommandGroup(
            //     new AutoIntakeSequence(intake, indexer, true),
            //     new AutoDriveToPoint(swerveSubsystem, Constants.AutoConstants.note3Pose.transformBy(new Transform2d(0.3, 0, new Rotation2d())),  0.05, 5, 1, true)
            //     ),
            // new AutoDriveToPoint(swerveSubsystem, Constants.AutoConstants.startPose.transformBy(new Transform2d(-0.1, 0, new Rotation2d())), true),
            // new AutoSpeakerSequence(shooter, indexer),
            
            //note 1
            new GrabAndSpeakerShoot(swerveSubsystem, intake, indexer, shooter, Constants.AutoConstants.note1Pose, Constants.AutoConstants.start2Pose)
            // new AutoDriveToPoint(swerveSubsystem, Constants.AutoConstants.note1Pose, false),
            // new ParallelCommandGroup(
            //     new AutoIntakeSequence(intake, indexer, true),
            //     new AutoDriveToPoint(swerveSubsystem, Constants.AutoConstants.note1Pose.transformBy(new Transform2d(0.2, 0, new Rotation2d())),  0.05, 5, 1, true)
            //     ),
            // new AutoDriveToPoint(swerveSubsystem, Constants.AutoConstants.startPose.transformBy(new Transform2d(-0.1, 0, new Rotation2d())), true),
            // new AutoSpeakerSequence(shooter, indexer)
        );
    }
}