package frc.robot.commands.autoRoutines;

import java.util.ArrayList;
import java.util.List;

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
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.GrabAndSpeakerShoot;
import frc.robot.commands.ResetSwervePoseCmd;
import frc.robot.commands.SpeakerSequence;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class S3n8n7 extends SequentialCommandGroup{ 
    
    // List<Pose2d> midpointsNote8 = new ArrayList<Pose2d>();
    // midpointsNote8.add(Constants.AutoConstants.note3Pose.transformBy(new Transform2d(0, 0.5, new Rotation2d())));

    // Pose2d[] midpointsNote7Array = {};
    // List<Pose2d> midpointsNote7 = new ArrayList<Pose2d>(midpointsNote7Array.asList());
    // midPointsNote7.add();
    //.add(Constants.AutoConstants.note3Pose.transformBy(new Transform2d(0, 0.5, new Rotation2d())));

    public S3n8n7(SwerveSubsystem swerveSubsystem, Intake intake, Indexer indexer, Shooter shooter) {

        addCommands(
            //pre-loaded note
            new ResetSwervePoseCmd(swerveSubsystem, Constants.AutoConstants.start3Pose),
            new SpeakerSequence(shooter, indexer),

            //note 8
            new GrabAndSpeakerShoot(swerveSubsystem, intake, indexer, shooter, Constants.AutoConstants.note8Pose, Constants.AutoConstants.start3Pose, Constants.AutoConstants.note3Pose.transformBy(new Transform2d(0, 0.5, new Rotation2d(60)))),
            //note 7
            new GrabAndSpeakerShoot(swerveSubsystem, intake, indexer, shooter, Constants.AutoConstants.note7Pose, Constants.AutoConstants.start3Pose, Constants.AutoConstants.note3Pose.transformBy(new Transform2d(0, 0.5, new Rotation2d(60)))),

            //drive to center
            new DriveToPoint(swerveSubsystem, Constants.AutoConstants.note3Pose.transformBy(new Transform2d(0, 0.5, new Rotation2d()))),
            new DriveToPoint(swerveSubsystem, Constants.AutoConstants.note8Pose)
        );
    }
}