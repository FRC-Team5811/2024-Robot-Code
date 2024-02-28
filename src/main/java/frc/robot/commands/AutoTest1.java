package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoTest1 extends SequentialCommandGroup{ 

    public AutoTest1(SwerveSubsystem swerveSubsystem, Pose2d pose1, Pose2d pose2, Pose2d pose3) {

        addCommands(
            new AutoDriveToPoint(swerveSubsystem, pose1, true),
            new AutoDriveToPoint(swerveSubsystem, pose2, true),
            new AutoDriveToPoint(swerveSubsystem, pose3, true)
        );
    }
}