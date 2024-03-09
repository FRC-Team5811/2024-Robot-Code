package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SpeakerRampUp extends Command {

    private final Shooter shooter;
    private int cycles = 0;

    public SpeakerRampUp(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        cycles += 1;
        shooter.autoSpeakerShotRampUp();
    }

    @Override
    public boolean isFinished() {
        if (cycles > 2*50) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopSpeakerMotors();

        // reset state
        cycles = 0;
    }
}
