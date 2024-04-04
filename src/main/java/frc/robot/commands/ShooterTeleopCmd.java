package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ShooterTeleopCmd extends Command {
    private final Shooter shooter;
    private final Indexer indexer;

    private final Supplier<Double> shooterAnalogFunction;
    private final Supplier<Boolean> shooterButtonFunction;
    private final Supplier<Boolean> ampScoreButtonFunction;
    private final Supplier<Boolean> shooterRampUpButton;
    private final Supplier<Boolean> diverterSpeakerFireButton;
    private final Supplier<Boolean> shuttleButtonFunction;
    private final Supplier<Boolean> speakerShotSequenceFunction;

    private final SlewRateLimiter shooterAnalogLimiter;

    public ShooterTeleopCmd(Shooter shooter, Indexer indexer, Supplier<Double> shooterAnalogFunction,
        Supplier<Boolean> shooterHoldFunction, Supplier<Boolean> ampScoreButtonFunction, 
        Supplier<Boolean> shooterRampUpButton, Supplier<Boolean> diverterSpeakerFireButton,
        Supplier<Boolean> shuttleButtonFunction, Supplier<Boolean> speakerShotSequanceFunction) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.shooterAnalogFunction = shooterAnalogFunction;
        this.shooterButtonFunction = shooterHoldFunction;
        this.ampScoreButtonFunction = ampScoreButtonFunction;
        this.shooterRampUpButton = shooterRampUpButton;
        this.diverterSpeakerFireButton = diverterSpeakerFireButton;
        this.shuttleButtonFunction = shuttleButtonFunction;
        this.speakerShotSequenceFunction = speakerShotSequanceFunction;
        shooterAnalogLimiter = new SlewRateLimiter(Constants.ManipConstants.shooterAnalogMaxRate);
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        
        if (diverterSpeakerFireButton.get()) {
            shooter.autoSpeakerShotRampUp();
            if (shooter.isShuttleReady()) {
                shooter.runSpeakerDiverter();
            }
        }

        else if (shooterRampUpButton.get()) {
            shooter.autoSpeakerShotRampUp();
            if (diverterSpeakerFireButton.get()) {
            shooter.runSpeakerDiverter();
            }
        }

        else if (shuttleButtonFunction.get() && speakerShotSequenceFunction.get()) {
            shooter.unevenRampUp(Constants.ManipConstants.shooterShuttleRPMUpper, Constants.ManipConstants.shooterShuttleRPMLower);
            if (shooter.isShuttleReady()) {
                shooter.runSpeakerDiverter();
            }
        }

        
        // else if (speakerShotSequenceFunction.get()) {
        //     new SpeakerSequence(shooter, indexer).schedule();
        // }

        else if (shuttleButtonFunction.get()) {
            shooter.unevenRampUp(Constants.ManipConstants.shooterShuttleRPMUpper, Constants.ManipConstants.shooterShuttleRPMLower);
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
