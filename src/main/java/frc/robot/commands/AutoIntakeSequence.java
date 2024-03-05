package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.PID;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class AutoIntakeSequence extends Command {

    private final boolean timeOut;
    private final Intake intake;
    private final Indexer indexer;
    private int cycles = 0;

    public AutoIntakeSequence(Intake intake, Indexer indexer, boolean timeOut) {
        this.timeOut = timeOut;
        this.indexer = indexer;
        this.intake = intake;
        addRequirements(intake);
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        cycles += 1;
        intake.pull();
        indexer.pull();
    }

    @Override
    public boolean isFinished() {
        if (cycles > 5*50 && timeOut) {
            return true;
        }
        return indexer.getLimitBool();  
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        indexer.stop();

        // reset state
        cycles = 0;
    }
}
