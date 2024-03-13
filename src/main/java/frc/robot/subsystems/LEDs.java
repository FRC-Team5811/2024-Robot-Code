package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class LEDs extends SubsystemBase {

    private final RobotContainer robotContainer;
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private int cycleCount = 0;

    public LEDs(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;

        led = new AddressableLED(Constants.ledPort);
        buffer = new AddressableLEDBuffer(Constants.ledCount);

        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();
    }

    @Override public void periodic() {
        cycleCount++;

        // states in order of most to least importance to display
        // each logic function is responsible for setting the buffer with colors
        if (robotContainer.shooter.isShooterWheelsReady()) { shooterWheelsReadyLogic(); }
        else if (robotContainer.shooter.isShooterWheelsWarmup()) { shooterWheelsWarmupLogic(); }
        else if (robotContainer.shooter.isShootingAmp()) { shootingAmpLogic(); }
        else if (robotContainer.indexer.isNoteLoaded()) { noteLoadedLogic(); }
        else if (robotContainer.intake.isNoteInIntake()) { noteInIntakeLogic(); }
        else if (robotContainer.intake.isIntaking()) { intakingLogic(); }
        else { idleLogic(); }

        // set the data after the buffer is changed
        led.setData(buffer);
    }

    private void idleLogic() {
        if (Robot.isInstanceDisabled()) { // robot is disabled
            // breathe in and out with alliance color

            int hue = Robot.allianceColor == Alliance.Blue ? 240 / 2 : 0;
            if (!Robot.allianceColorKnown)
                hue = 290 / 2; // if we don't know alliance color, set to purple

            int value = 64; // (int)Math.sin(2 * Math.PI * cycleCount / 50 / 4); // breathe in and out every 4 seconds
            for (int i = 0; i < buffer.getLength(); i++) {
                // black, off
                buffer.setHSV(i, hue, 255, value);
            }
        }
        else { // robot is not disabled
            // breathe in and out BONDS color

            int hue = 42 / 2;
            int value = 64; //(int)Math.sin(2 * Math.PI * cycleCount / 50 / 2); // breathe in and out every 2 seconds
            for (int i = 0; i < buffer.getLength(); i++) {
                // black, off
                buffer.setHSV(i, hue, 255, value);
            }
        }
    }

    private void intakingLogic() {
        for (int i = 0; i < buffer.getLength(); i++) {
            // bright red
            buffer.setHSV(i, 0, 255, 255);
        }
    }

    private void noteInIntakeLogic() {
        // int value = cycleCount % 25 < 13 ? 255 : 0;
        for (int i = 0; i < buffer.getLength(); i++) {
            // bright blue
            buffer.setHSV(i, 120 / 2, 255, 255);
        }
    }

    private void noteLoadedLogic() {
        for (int i = 0; i < buffer.getLength(); i++) {
            // bright blue
            buffer.setHSV(i, 240 / 2, 255, 255);
        }
    }

    private void shootingAmpLogic() {
        // flashing blue

        int value = cycleCount % 25 < 13 ? 255 : 0; // flash on and off every 0.5 seconds
        for (int i = 0; i < buffer.getLength(); i++) {
            // bright blue
            buffer.setHSV(i, 240 / 2, 255, value);
        }
    }

    private void shooterWheelsWarmupLogic() {
        int value = 255 * (int)(robotContainer.shooter.getLowerSpeakerRPM() / Constants.ManipConstants.shooterSpeakerRPMUpper); //increases intensity as motors spin up
        for (int i = 0; i < buffer.getLength(); i++) {
            // bright red
            buffer.setHSV(i, 0, 255, value);
        }
    }

    private void shooterWheelsReadyLogic() {
        // int value = cycleCount % 25 < 13 ? 255 : 0; // flash on and off every 0.5 seconds
        for (int i = 0; i < buffer.getLength(); i++) {
            // bright purple
            buffer.setHSV(i, 290 / 2, 255, 255);
        }
    }

}
