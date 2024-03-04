package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Shooter;

public class ShooterTeleopCmd extends Command {
    private final Shooter shooter;

    private final Supplier<Double> shooterAnalogFunction;
    private final Supplier<Boolean> shooterButtonFunction;
    private final Supplier<Boolean> ampScoreButtonFunction;

    private final SlewRateLimiter shooterAnalogLimiter;

    public ShooterTeleopCmd(Shooter shooter, Supplier<Double> shooterAnalogFunction,
        Supplier<Boolean> shooterHoldFunction, Supplier<Boolean> ampScoreButtonFunction) {
        this.shooter = shooter;
        this.shooterAnalogFunction = shooterAnalogFunction;
        this.shooterButtonFunction = shooterHoldFunction;
        this.ampScoreButtonFunction = ampScoreButtonFunction;
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
            // shooter.runDiverterSpeakerShot();
        }
        else {
            double input = shooterAnalogFunction.get();
            input = Math.abs(input) > OIConstants.shooterManualDeadband ? input : 0.0;
            input = shooterAnalogLimiter.calculate(input);

            // shooter.runMotorsDirectSpeed(input);

            // determine how to run the diverter/amp shooter
            if (input != 0.0) { // shooter is being manually controlled via analog axis
                
                // shooter.runDiverterSpeakerShot();
            }
            else if (ampScoreButtonFunction.get()) { // shooter not being controlled, we can safely make an amp shot
                // shooter.runDiverterAmpShot();
            }
            else {
                // shooter.stopDiverter();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        // shooter.runMotorsZeroSpeed();
        // shooter.stopDiverter();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
