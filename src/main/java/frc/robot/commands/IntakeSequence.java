package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;

public class IntakeSequence extends Command {

    private final boolean timeOut;
    private final Intake intake;
    private final Indexer indexer;
    private int cycles = 0;
    //private int cyclesInPlace = 0;

    public IntakeSequence(Intake intake, Indexer indexer, boolean timeOut) {
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
        if (indexer.getLimitBool()) {
            return true;
        }
        if (cycles > 5*50 && timeOut) {
            return true;
        }
        return false;  
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        indexer.stop();

        // reset state
        cycles = 0;
    }
}
