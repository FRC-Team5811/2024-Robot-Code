package frc.robot.commands;

import java.util.Arrays;
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
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class GrabAndSpeakerShoot extends SequentialCommandGroup {

    public GrabAndSpeakerShoot(SwerveSubsystem swerveSubsystem, Intake intake, Indexer indexer, Shooter shooter, Pose2d notePose, Pose2d firePose) {

        addCommands(
            new ParallelRaceGroup(
                new DriveToPoint(swerveSubsystem, notePose, 0.3, 5, Constants.AutoConstants.kMaxSpeedMetersPerSecond, false),
                //new DriveToPoint(swerveSubsystem, notePose, 0.1, 5, Constants.AutoConstants.kMaxSpeedMetersPerSecond, false, 7, 0.2, 2, 8, 0, 0.1),
                new IntakeSequence(intake, indexer, false)
            ),
            new ParallelRaceGroup(
                new IntakeSequence(intake, indexer, true),
                new DriveToPoint(swerveSubsystem, notePose.transformBy(new Transform2d(1, 0, new Rotation2d())),  0.1, 5, Constants.AutoConstants.kMaxSpeedMetersPerSecond, false)
                //new DriveToPoint(swerveSubsystem, notePose.transformBy(new Transform2d(1, 0, new Rotation2d())),  0.1, 2.5, 5, false)
            ),
            new ParallelDeadlineGroup(
                new ParallelCommandGroup(
                    new DriveToPoint(swerveSubsystem, firePose, false),
                    new IntakeSequence(intake, indexer, true)
                ),
                new SpeakerRampUp(shooter)
            ),
            new SpeakerSequence(shooter, indexer)

            );
    }

    public GrabAndSpeakerShoot(SwerveSubsystem swerveSubsystem, Intake intake, Indexer indexer, Shooter shooter, Pose2d notePose, Pose2d firePose, Pose2d midpoint) {

        addCommands(
            new DriveToPoint(swerveSubsystem, midpoint, 0.5, 10, Constants.AutoConstants.kMaxSpeedMetersPerSecond, false),
            //new DriveToPoint(swerveSubsystem, midpoint, 0.15, 2.5,Constants.AutoConstants.kMaxSpeedMetersPerSecond, false),
            new ParallelDeadlineGroup(
                new DriveToPoint(swerveSubsystem, notePose, 0.3, 5, Constants.AutoConstants.kMaxSpeedMetersPerSecond, false),
                // new DriveToPoint(swerveSubsystem, notePose),
                new IntakeSequence(intake, indexer, false)
            ),new ParallelRaceGroup(
                new IntakeSequence(intake, indexer, true),
                new DriveToPoint(swerveSubsystem, notePose.transformBy(new Transform2d(1, 0, new Rotation2d())),  0.1, 5, Constants.AutoConstants.kMaxSpeedMetersPerSecond, false)
                //new DriveToPoint(swerveSubsystem, notePose.transformBy(new Transform2d(1, 0, new Rotation2d())),  0.05, 2.5, 5, true)
            ),
            new ParallelDeadlineGroup(
                new DriveToPoint(swerveSubsystem, midpoint, 0.5, 10, Constants.AutoConstants.kMaxSpeedMetersPerSecond, false),
                new IntakeSequence(intake, indexer, true)
            ),
            new ParallelDeadlineGroup(
                new ParallelCommandGroup(
                    new DriveToPoint(swerveSubsystem, firePose, true),
                    new IntakeSequence(intake, indexer, true)
                ),
                new SpeakerRampUp(shooter)
            ),
            new SpeakerSequence(shooter, indexer)

            );
    }

    public GrabAndSpeakerShoot(SwerveSubsystem swerveSubsystem, Intake intake, Indexer indexer, Shooter shooter, Pose2d notePose, Pose2d firePose, Pose2d[] midpointsArray) {
        List<Pose2d> midpointsList = Arrays.asList(midpointsArray);
        ListIterator<Pose2d> midpointIterator = midpointsList.listIterator();
        while (midpointIterator.hasNext()) {
            addCommands(
                new DriveToPoint(swerveSubsystem, midpointIterator.next(), 0.5, 10, Constants.AutoConstants.kMaxSpeedMetersPerSecond, false)
                // new DriveToPoint(swerveSubsystem, midpointIterator.next(), 0.15, 2.5, Constants.AutoConstants.kMaxSpeedMetersPerSecond, false)
            );
        }

        addCommands(
  
            new ParallelDeadlineGroup(
                new DriveToPoint(swerveSubsystem, notePose, 0.3, 5, Constants.AutoConstants.kMaxSpeedMetersPerSecond, false),
                // new DriveToPoint(swerveSubsystem, notePose),
                new IntakeSequence(intake, indexer, false)
            ),
            new ParallelRaceGroup(
                new IntakeSequence(intake, indexer, true),
                new DriveToPoint(swerveSubsystem, notePose.transformBy(new Transform2d(1, 0, new Rotation2d())),  0.1, 5, Constants.AutoConstants.kMaxSpeedMetersPerSecond, false)
                // new DriveToPoint(swerveSubsystem, notePose.transformBy(new Transform2d(1, 0, new Rotation2d())),  0.05, 2.5, 5, true)
            )
        );

        while (midpointIterator.hasPrevious()) {
            addCommands(
                new ParallelDeadlineGroup(
                    new DriveToPoint(swerveSubsystem, midpointIterator.previous(), 0.5, 10, Constants.AutoConstants.kMaxSpeedMetersPerSecond, false),
                    // new DriveToPoint(swerveSubsystem, midpointIterator.previous(), 0.15, 2.5,Constants.AutoConstants.kMaxSpeedMetersPerSecond, false),
                    new IntakeSequence(intake, indexer, true)
                )
            );
        }

        addCommands(
            new ParallelDeadlineGroup(
                new ParallelCommandGroup(
                    new DriveToPoint(swerveSubsystem, firePose, true),
                    new IntakeSequence(intake, indexer, true)
                    ),
                new SpeakerRampUp(shooter)
            ),
            new SpeakerSequence(shooter, indexer)

            );
    }

public GrabAndSpeakerShoot(SwerveSubsystem swerveSubsystem, Intake intake, Indexer indexer, Shooter shooter, 
    Pose2d notePose, Pose2d firePose, Pose2d[] midpointsArray, Rotation2d rotation) {
        List<Pose2d> midpointsList = Arrays.asList(midpointsArray);
        ListIterator<Pose2d> midpointIterator = midpointsList.listIterator();
        while (midpointIterator.hasNext()) {
            addCommands(
                new DriveToPoint(swerveSubsystem, midpointIterator.next().transformBy(new Transform2d(0, 0, rotation)),
                 0.5, 10, Constants.AutoConstants.kMaxSpeedMetersPerSecond, false,
                 15, 0, 9, 8, 0, 0.1)
                // new DriveToPoint(swerveSubsystem, midpointIterator.next(), 0.15, 2.5, Constants.AutoConstants.kMaxSpeedMetersPerSecond, false)
            );
        }

        addCommands(
  
            new ParallelDeadlineGroup(
                new DriveToPoint(swerveSubsystem, notePose.transformBy(new Transform2d(0, 0, rotation)), 
                0.3, 5, Constants.AutoConstants.kMaxSpeedMetersPerSecond, true,
                18, 0, 9, 8, 0, 0.1),
                // new DriveToPoint(swerveSubsystem, notePose),
                new IntakeSequence(intake, indexer, false)
            ),
            new ParallelRaceGroup(
                new IntakeSequence(intake, indexer, true),
                new DriveToPoint(swerveSubsystem, notePose.transformBy(new Transform2d(rotation.getCos()*1.3, rotation.getSin()*1.3, rotation)),  
                0.1, 5, Constants.AutoConstants.kMaxSpeedMetersPerSecond, false)
                // new DriveToPoint(swerveSubsystem, notePose.transformBy(new Transform2d(1, 0, new Rotation2d())),  0.05, 2.5, 5, true)
            )
        );

        while (midpointIterator.hasPrevious()) {
            addCommands(
                new ParallelDeadlineGroup(
                    new DriveToPoint(swerveSubsystem, midpointIterator.previous().transformBy(new Transform2d(0, 0, rotation)), 0.5, 10, Constants.AutoConstants.kMaxSpeedMetersPerSecond, false),
                    // new DriveToPoint(swerveSubsystem, midpointIterator.previous(), 0.15, 2.5,Constants.AutoConstants.kMaxSpeedMetersPerSecond, false),
                    new IntakeSequence(intake, indexer, true)
                )
            );
        }

        addCommands(
            new ParallelDeadlineGroup(
                new ParallelCommandGroup(
                    new DriveToPoint(swerveSubsystem, firePose, true),
                    new IntakeSequence(intake, indexer, true)
                    ),
                new SpeakerRampUp(shooter)
            ),
            new SpeakerSequence(shooter, indexer)

            );
    }

}