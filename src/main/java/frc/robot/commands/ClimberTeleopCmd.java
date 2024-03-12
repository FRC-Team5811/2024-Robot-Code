package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberTeleopCmd extends Command {
    private final Climber climber;

    private final Supplier<Boolean> climberFunction;
    private final Supplier<Boolean> climberDownFunction;

    public ClimberTeleopCmd(Climber climber, Supplier<Boolean> climberFunction, Supplier<Boolean> climberDownFunction) {
        this.climber = climber;
        this.climberFunction = climberFunction;
        this.climberDownFunction = climberDownFunction;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (climberFunction.get()) {
            // manip is manually intaking

            climber.up();
        }
        else if (climberDownFunction.get()) {
            // manip is manually expelling

            climber.down();
        }
        else {
            climber.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

