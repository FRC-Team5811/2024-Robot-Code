package frc.robot.commands;

import java.util.List;
import java.util.ListIterator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class GrabAndSpeakerShoot extends SequentialCommandGroup {

    public GrabAndSpeakerShoot(SwerveSubsystem swerveSubsystem, Intake intake, Indexer indexer, Shooter shooter, Pose2d notePose, Pose2d firePose) {

        addCommands(
            new ParallelDeadlineGroup(
                new DriveToPoint(swerveSubsystem, notePose),
                new IntakeSequence(intake, indexer, false)
            ),
            new ParallelRaceGroup(
                new IntakeSequence(intake, indexer, true),
                new DriveToPoint(swerveSubsystem, notePose.transformBy(new Transform2d(0.3, 0, new Rotation2d())),  0.05, 2.5, 2, true)
            ),
            new ParallelCommandGroup(
                new IntakeSequence(intake, indexer, true),
                new DriveToPoint(swerveSubsystem, firePose, false),
                new SpeakerRampUp(shooter)
            ),
            new SpeakerSequence(shooter, indexer)

            );
    }

    public GrabAndSpeakerShoot(SwerveSubsystem swerveSubsystem, Intake intake, Indexer indexer, Shooter shooter, Pose2d notePose, Pose2d firePose, Pose2d midpoint) {

        addCommands(
            new DriveToPoint(swerveSubsystem, midpoint),
            new ParallelDeadlineGroup(
                new DriveToPoint(swerveSubsystem, notePose),
                new IntakeSequence(intake, indexer, false)
            ),new ParallelRaceGroup(
                new IntakeSequence(intake, indexer, true),
                new DriveToPoint(swerveSubsystem, notePose.transformBy(new Transform2d(0.3, 0, new Rotation2d())),  0.05, 2.5, 1, true)
            ),
            new ParallelCommandGroup(
                new IntakeSequence(intake, indexer, true),
                new DriveToPoint(swerveSubsystem, midpoint, true)
            ),
            new ParallelCommandGroup(
                new IntakeSequence(intake, indexer, true),
                new DriveToPoint(swerveSubsystem, firePose, true),
                new SpeakerRampUp(shooter)
            ),
            new SpeakerSequence(shooter, indexer)

            );
    }

    public GrabAndSpeakerShoot(SwerveSubsystem swerveSubsystem, Intake intake, Indexer indexer, Shooter shooter, Pose2d notePose, Pose2d firePose, List<Pose2d> midpointsList) {
        ListIterator<Pose2d> midpointIterator = midpointsList.listIterator();
        while (midpointIterator.hasNext()) {
            addCommands(
                new DriveToPoint(swerveSubsystem, midpointIterator.next(), false)
            );
        }

        addCommands(
  
            new ParallelDeadlineGroup(
                new DriveToPoint(swerveSubsystem, notePose),
                new IntakeSequence(intake, indexer, false)
            ),
            new ParallelRaceGroup(
                new IntakeSequence(intake, indexer, true),
                new DriveToPoint(swerveSubsystem, notePose.transformBy(new Transform2d(0.3, 0, new Rotation2d())),  0.05, 2.5, 1, true)
            )
        );

        ListIterator<Pose2d> midpointIterator2 = midpointsList.listIterator();
        while (midpointIterator2.hasNext()) {
            addCommands(
                new ParallelCommandGroup(
                    new IntakeSequence(intake, indexer, true),
                    new DriveToPoint(swerveSubsystem, midpointIterator2.next(), false)
                )
            );
        }

        addCommands(
            new ParallelCommandGroup(
                new IntakeSequence(intake, indexer, true),
                new DriveToPoint(swerveSubsystem, firePose, true),
                new SpeakerRampUp(shooter)
            ),
            new SpeakerSequence(shooter, indexer)

            );
    }

}