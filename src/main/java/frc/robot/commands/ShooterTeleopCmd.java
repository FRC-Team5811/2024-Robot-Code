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
    private final Supplier<Boolean> shooterRampUpButton;
    private final Supplier<Boolean> diverterSpeakerFireButton;

    private final SlewRateLimiter shooterAnalogLimiter;

    public ShooterTeleopCmd(Shooter shooter, Supplier<Double> shooterAnalogFunction,
        Supplier<Boolean> shooterHoldFunction, Supplier<Boolean> ampScoreButtonFunction, Supplier<Boolean> shooterRampUpButton, Supplier<Boolean> diverterSpeakerFireButton) {
        this.shooter = shooter;
        this.shooterAnalogFunction = shooterAnalogFunction;
        this.shooterButtonFunction = shooterHoldFunction;
        this.ampScoreButtonFunction = ampScoreButtonFunction;
        this.shooterRampUpButton = shooterRampUpButton;
        this.diverterSpeakerFireButton = diverterSpeakerFireButton;
        shooterAnalogLimiter = new SlewRateLimiter(Constants.ManipConstants.shooterAnalogMaxRate);
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        
        if (diverterSpeakerFireButton.get()) {
            shooter.runSpeakerDiverter();
        }
        if (shooterButtonFunction.get()) {
            // manip is pressing speaker shot ramp up button

            shooter.autoSpeakerShotRampUp();
            shooter.runSpeakerDiverter();
        }
        else if (shooterRampUpButton.get()) {
            shooter.autoSpeakerShotRampUp();
        }
        else {
            double input = shooterAnalogFunction.get();
            input = Math.abs(input) > OIConstants.shooterCustomDeadband ? input : 0.0;
            input = shooterAnalogLimiter.calculate(input);

            shooter.manualSpeakerMotors(input);

            // determine how to run the diverter/amp shooter
            if (ampScoreButtonFunction.get()) { // shooter not being controlled, we can safely make an amp shot
                shooter.runAmpDiverter();
            }
            else if (input != 0.0) { // shooter is being manually controlled via analog axis
                
                shooter.runSpeakerDiverter();
            }
            else {
                if (!diverterSpeakerFireButton.get()) {
                    shooter.stopDiverter();
                }
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopDiverter();
        shooter.stopSpeakerMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
