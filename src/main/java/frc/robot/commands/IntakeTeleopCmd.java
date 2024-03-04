package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeTeleopCmd extends Command {
    private final Intake intake;

    private final Supplier<Boolean> intakeFunction;
    private final Supplier<Boolean> expelFunction;

    public IntakeTeleopCmd(Intake intake,
        Supplier<Boolean> intakeFunction, Supplier<Boolean> expelFunction) {
        this.intake = intake;
        this.intakeFunction = intakeFunction;
        this.expelFunction = expelFunction;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (intakeFunction.get()) {
            // manip is manually intaking

            // intake.runMotorsAtIntakingSpeed();
        }
        if (expelFunction.get()) {
            // manip is manually expelling

            // intake.runMotorsAtExpellingSpeed();
        }
        else {
            // intake.runMotorsZeroSpeed();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // intake.runMotorsZeroSpeed();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
