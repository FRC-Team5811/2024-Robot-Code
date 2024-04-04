package frc.robot.commands.autoRoutines;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.GrabAndSpeakerShoot;
import frc.robot.commands.ResetSwervePoseCmd;
import frc.robot.commands.SpeakerSequence;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class defense extends SequentialCommandGroup{ 

    public defense(SwerveSubsystem swerveSubsystem, Intake intake, Indexer indexer, Shooter shooter) {

        addCommands(
            //pre-loaded note
            new ResetSwervePoseCmd(swerveSubsystem, Constants.AutoConstants.start1Pose),
            new SpeakerSequence(shooter, indexer, 5000),
            // kill all center notes
            new DriveToPoint(swerveSubsystem, Constants.AutoConstants.mid1ForwardPose.transformBy(new Transform2d(-1, 0, Rotation2d.fromDegrees(-60))), 0.5, 10, 7, 
            false, 10, 0, 2, 8, 0, 0.1),
            new DriveToPoint(swerveSubsystem, Constants.AutoConstants.note4Pose.transformBy(new Transform2d(0.5, 0, Rotation2d.fromDegrees(-60))), 0.2, 10, 5, true),
            new DriveToPoint(swerveSubsystem, Constants.AutoConstants.note8Pose.transformBy(new Transform2d(0.5, 0, Rotation2d.fromDegrees(-60))), 
            0.5, 10, 7, 
            false, 10, 0, 2, 8, 0, 0.1)
        );
    }
}