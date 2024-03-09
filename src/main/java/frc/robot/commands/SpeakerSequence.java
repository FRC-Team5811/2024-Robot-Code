package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;

public class SpeakerSequence extends Command {

    private final Shooter shooter;
    private final Indexer indexer;
    private int cycles = 0;

    public SpeakerSequence(Shooter shooter, Indexer indexer) {
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
        shooter.autoSpeakerShotRampUp();
        shooter.runSpeakerDiverter();
        if (shooter.getSpeakersRPM() >= 0.5*Constants.ManipConstants.shooterSpeakerRPMLower) {
            indexer.speakerScore();
        }
        if (!indexer.getLimitBool()) {
            cycles += 1;
        }
    }

    @Override
    public boolean isFinished() {
        if (cycles > 0.6*50) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopSpeakerMotors();
        shooter.stopDiverter();
        indexer.stop();

        // reset state
        cycles = 0;
    }
}
