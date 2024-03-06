package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveSubsystem;

public class ResetSwervePoseCmd extends InstantCommand {

    private final SwerveSubsystem swerveSubsystem;
    private final Pose2d pose;

    public ResetSwervePoseCmd(SwerveSubsystem swerveSubsystem, Pose2d pose) {
        this.swerveSubsystem = swerveSubsystem;
        this.pose = pose;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        swerveSubsystem.resetOdometry(Robot.processPoseWithAllianceColor(pose));
    }

    @Override
    public void end(boolean interrupted) {}

}
