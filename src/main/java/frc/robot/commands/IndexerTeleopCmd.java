package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class IndexerTeleopCmd extends Command {
    private final Indexer indexer;

    private final Supplier<Boolean> intakeFunction;
    private final Supplier<Boolean> expelFunction;
    private final Supplier<Boolean> ampScoreFunction;
    private final Supplier<Boolean> speakerScoreFunction;

    public IndexerTeleopCmd(Indexer indexer,
        Supplier<Boolean> intakeFunction, Supplier<Boolean> expelFunction, 
        Supplier<Boolean> ampScoreFunction, Supplier<Boolean> speakerScoreFunction) {
        this.indexer = indexer;
        this.intakeFunction = intakeFunction;
        this.expelFunction = expelFunction;
        this.ampScoreFunction = ampScoreFunction;
        this.speakerScoreFunction = speakerScoreFunction;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (intakeFunction.get()) {
            // manip is manually intaking

            indexer.pull();
        }
        else if (expelFunction.get()) {
            // manip is manually expelling

            indexer.push();
        }
        else if (ampScoreFunction.get()) {
            // manip is manually amp scoring

            indexer.ampScore();
        }
        else if (speakerScoreFunction.get()) {
            // manip is manually speaker scoring

            indexer.speakerScore();
        }
        else {
            indexer.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
