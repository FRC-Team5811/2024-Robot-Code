package frc.robot.commands.autoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.GrabAndSpeakerShoot;
import frc.robot.commands.ResetSwervePoseCmd;
import frc.robot.commands.SpeakerSequence;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class S2n1n2n3 extends SequentialCommandGroup{ 

    public S2n1n2n3(SwerveSubsystem swerveSubsystem, Intake intake, Indexer indexer, Shooter shooter) {

        addCommands(
            //pre-loaded note
            new ResetSwervePoseCmd(swerveSubsystem, Constants.AutoConstants.start2Pose),
            new SpeakerSequence(shooter, indexer),
            
            //note 1
            new GrabAndSpeakerShoot(swerveSubsystem, intake, indexer, shooter, 
            Constants.AutoConstants.note1Pose, 
            Constants.AutoConstants.start2Pose),

            //note 2
            new GrabAndSpeakerShoot(swerveSubsystem, intake, indexer, shooter, 
            Constants.AutoConstants.note2Pose, 
            Constants.AutoConstants.start2Pose),
  
            //note 3
            new GrabAndSpeakerShoot(swerveSubsystem, intake, indexer, shooter, 
            Constants.AutoConstants.note3Pose, 
            Constants.AutoConstants.start2Pose)
        );
    }
}