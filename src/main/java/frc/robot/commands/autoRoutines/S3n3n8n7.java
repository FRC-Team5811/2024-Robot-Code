package frc.robot.commands.autoRoutines;

import edu.wpi.first.math.geometry.Pose2d;
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

public class S3n3n8n7 extends SequentialCommandGroup{ 

    public S3n3n8n7(SwerveSubsystem swerveSubsystem, Intake intake, Indexer indexer, Shooter shooter) {
        addCommands(
            //pre-loaded note
            new ResetSwervePoseCmd(swerveSubsystem, Constants.AutoConstants.start3Pose),
            new SpeakerSequence(shooter, indexer),
            
            // note 3
            new GrabAndSpeakerShoot(swerveSubsystem, intake, indexer, shooter, 
            Constants.AutoConstants.note3Pose, 
            Constants.AutoConstants.start3Pose,
            new Pose2d(Constants.AutoConstants.note3Pose.getX()-1, 
                Constants.AutoConstants.note3Pose.getY(), 
                Rotation2d.fromDegrees(60))),
            
            //note 8
            new GrabAndSpeakerShoot(swerveSubsystem, intake, indexer, shooter, 
            Constants.AutoConstants.note8Pose, Constants.AutoConstants.start3Pose, 
            new Pose2d[] {Constants.AutoConstants.note3Pose.transformBy(new Transform2d(0, 0, 
                Rotation2d.fromDegrees(60))), Constants.AutoConstants.note3Pose} ),
            
            //note 7
            new GrabAndSpeakerShoot(swerveSubsystem, intake, indexer, shooter, 
            Constants.AutoConstants.note7Pose, Constants.AutoConstants.start3Pose, 
            new Pose2d[] {Constants.AutoConstants.note3Pose.transformBy(new Transform2d(0, 0, 
                Rotation2d.fromDegrees(60))), Constants.AutoConstants.note3Pose} )
        );
    }
}