package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;

public class SpeakerSequence extends Command {

    private final Shooter shooter;
    private final Indexer indexer;
    private int cycles = 0;
    private int limitCycles = 0;
    private double speakerRPM = Constants.ManipConstants.shooterSpeakerRPMLower;

    public SpeakerSequence(Shooter shooter, Indexer indexer) {
        this.indexer = indexer;
        this.shooter = shooter;
        addRequirements(shooter);
        addRequirements(indexer);
    }

    public SpeakerSequence(Shooter shooter, Indexer indexer, double speakerRPM) {
        this.indexer = indexer;
        this.shooter = shooter;
        this.speakerRPM = speakerRPM;
        addRequirements(shooter);
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooter.autoSpeakerShotRampUp(speakerRPM);
        shooter.runSpeakerDiverter();
        SmartDashboard.putBoolean("Driver/is firing", false);
        if (shooter.getLowerSpeakerRPM() >= 0.95*speakerRPM
            // && shooter.getUpperSpeakerRPM() >= 0.95*speakerRPM
            ) {
            indexer.speakerScore();
            SmartDashboard.putBoolean("Driver/is firing", true);
        }
        if (!indexer.getLimitBool()) {
            limitCycles += 1;
        }
        cycles++;
    }

    @Override
    public boolean isFinished() {
        if (cycles > 3*50 || limitCycles > 0.6*50) {
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
        limitCycles = 0;
        cycles = 0;
    }
}
