package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class IndexerTeleopCmd extends Command {
    private final Indexer indexer;

    private final Supplier<Boolean> intakeFunction;
    private final Supplier<Boolean> ampScoreFunction;
    private final Supplier<Boolean> speakerScoreFunction;

    public IndexerTeleopCmd(Indexer indexer,
        Supplier<Boolean> intakeFunction, Supplier<Boolean> ampScoreFunction,
        Supplier<Boolean> speakerScoreFunction) {
        this.indexer = indexer;
        this.intakeFunction = intakeFunction;
        this.ampScoreFunction = ampScoreFunction;
        this.speakerScoreFunction = speakerScoreFunction;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (intakeFunction.get()) {
            // manip is manually intaking

            // indexer.runMotorsAtIntakingSpeed();
        }
        else if (ampScoreFunction.get()) {
            // manip is manually amp scoring

            // indexer.runMotorsAtAmpScoringSpeed();
        }
        else if (speakerScoreFunction.get()) {
            // manip is manually speaker scoring

            // indexer.runMotorsAtSpeakerScoringSpeed();
        }
        else {
            // indexer.runMotorsZeroSpeed();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // indexer.runMotorsZeroSpeed();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
