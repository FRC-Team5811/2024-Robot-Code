package frc.robot.commands.autoRoutines;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.GrabAndSpeakerShoot;
import frc.robot.commands.ResetSwervePoseCmd;
import frc.robot.commands.SpeakerSequence;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class S1n1n2n3 extends SequentialCommandGroup{ 

    public S1n1n2n3(SwerveSubsystem swerveSubsystem, Intake intake, Indexer indexer, Shooter shooter) {

        addCommands(
            //pre-loaded note
            new ResetSwervePoseCmd(swerveSubsystem, Constants.AutoConstants.start1Pose),
            new SpeakerSequence(shooter, indexer, 5000),
            // note 1
            new GrabAndSpeakerShoot(swerveSubsystem, intake, indexer, shooter, 
            Constants.AutoConstants.note1Pose, 
            Constants.AutoConstants.start1Pose.transformBy(new Transform2d(-0.05, -0.075, new Rotation2d(0))), 
            Constants.AutoConstants.note1Pose.transformBy(new Transform2d(-0.5, 0, Rotation2d.fromDegrees(-60)))),
            
            //note 2
            new GrabAndSpeakerShoot(swerveSubsystem, intake, indexer, shooter, 
            Constants.AutoConstants.note2Pose, 
            Constants.AutoConstants.start1Pose.transformBy(new Transform2d(-0.1, -0.125, new Rotation2d(0))), 
            Constants.AutoConstants.note1Pose.transformBy(new Transform2d(-0.5, 0, Rotation2d.fromDegrees(-60)))),
            
            //note 3
            new GrabAndSpeakerShoot(swerveSubsystem, intake, indexer, shooter, 
            Constants.AutoConstants.note3Pose, 
            Constants.AutoConstants.start1Pose.transformBy(new Transform2d(-0.2, -0.3, new Rotation2d(0))), 
            Constants.AutoConstants.note1Pose.transformBy(new Transform2d(-0.5, 0, Rotation2d.fromDegrees(-60))))
        );
    }
}