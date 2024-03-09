package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;

public class AmpSequence extends Command {

    private final Shooter shooter;
    private final Indexer indexer;
    private int cycles = 0;

    public AmpSequence(Shooter shooter, Indexer indexer) {
        this.indexer = indexer;
        this.shooter = shooter;
        addRequirements(shooter);
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (!indexer.getLimitBool()) {
            cycles += 1;
        }
        shooter.runAmpDiverter();
        indexer.ampScore();
    }

    @Override
    public boolean isFinished() {
        if (cycles > 1.2*50) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopDiverter();
        indexer.stop();

        // reset state
        cycles = 0;
    }
}
