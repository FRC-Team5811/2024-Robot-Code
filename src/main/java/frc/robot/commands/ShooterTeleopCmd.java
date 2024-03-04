package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Intake;

public class ShooterTeleopCmd extends Command {
    private final Intake shooter;

    private final Supplier<Double> shooterAnalogFunction;
    private final Supplier<Boolean> shooterButtonFunction;

    private final SlewRateLimiter shooterAnalogLimiter;

    public ShooterTeleopCmd(Intake shooter, Supplier<Double> shooterAnalogFunction,
        Supplier<Boolean> shooterHoldFunction) {
        this.shooter = shooter;
        this.shooterAnalogFunction = shooterAnalogFunction;
        this.shooterButtonFunction = shooterHoldFunction;
        shooterAnalogLimiter = new SlewRateLimiter(Constants.ManipConstants.shooterAnalogMaxRate);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (shooterButtonFunction.get()) {
            // manip is pressing shooter button

            // shooter.runMotorsSpeakerPID();
        }
        else {
            double input = shooterAnalogFunction.get();
            input = Math.abs(input) > OIConstants.kDeadband ? input : 0.0;
            input = shooterAnalogLimiter.calculate(input);

            // shooter.runMotorsDirectSpeed(input);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // shooter.runMotorsZeroSpeed();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
