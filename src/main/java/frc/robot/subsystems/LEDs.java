package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class LEDs extends SubsystemBase {

    private final RobotContainer robotContainer;
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private int cycleCount = 0;
    private int state = 1;
    private int prevState = 0;

    public LEDs(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;

        led = new AddressableLED(Constants.ledPort);
        buffer = new AddressableLEDBuffer(Constants.ledCount);
        SmartDashboard.putString("Debug/LED State", "LED Start");

        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();
    }

    @Override public void periodic() {
        cycleCount++;

        // states in order of most to least importance to display
        // each logic function is responsible for setting the buffer with colors
        if (!Robot.isAutonomousCustom()) {    
            if (robotContainer.shooter.isSpeakerReady()) { state = 1; }
            else if (robotContainer.shooter.isShooterWheelsWarmup()) { state = 2; }
            else if (robotContainer.shooter.isShootingAmp()) { state = 3; }
            else if (robotContainer.swerveSubsystem.isShuttlePose()) { state = 9;}
            else if (robotContainer.indexer.isNoteLoaded()) { state = 4; }
            else if (robotContainer.intake.isNoteInIntake()) { state = 5; }
            else if (robotContainer.intake.isIntaking()) { state = 6; }
            else if (robotContainer.climber.getLimitBool()) { state = 7; }
            else {state = 8;}
        }
        else state = prevState;
        
        if (state != prevState
            || state == 8
            || state == 2) {
            if (state == 1) { shooterWheelsReadyLogic(); }
            else if (state == 2) { shooterWheelsWarmupLogic(); }
            else if (state == 3) { shootingAmpLogic(); }
            else if (state == 9) { shutlePoseLogic(); }
            else if (state == 4) { noteLoadedLogic(); }
            else if (state == 5) { noteInIntakeLogic(); }
            else if (state == 6) { intakingLogic(); }
            else if (state == 7) { climberLimitReached(); }
            else { idleLogic(); }

            // set the data after the buffer is changed
            led.setData(buffer);
        }
        prevState = state;
    }

    private void idleLogic() {
        if (Robot.isInstanceDisabled()) { // robot is disabled
            // breathe in and out with alliance color


            if (!Robot.allianceColorKnown) {
                int hue = 280 / 2; // if we don't know alliance color, set to purple
                int value = 64;
                if (true) {
                    for (int i = 0; i < buffer.getLength(); i++) {
                        hue = (int)((360 - (280 + 80*Math.sin((i+(cycleCount/10))*2*Math.PI/20)))/2); //spins through colors
                        buffer.setHSV(i, hue, 255, value);
                }
            }
            }
            else {
                int hue = Robot.allianceColor == Alliance.Blue ? 120 / 2 : 0;
                int value = 64; // (int)Math.sin(2 * Math.PI * cycleCount / 50 / 4); // breathe in and out every 4 seconds
                for (int i = 0; i < buffer.getLength(); i++) {
                    // black, off
                    if (i % 2 == 1) buffer.setHSV(i, hue, 255, value);
                    else buffer.setHSV(i, hue, 255, value);
            }
            SmartDashboard.putString("Debug/LED State", "Idle Disabled");
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
            SmartDashboard.putString("Debug/LED State", "Idle Not Disabled");
        }
    }

    private void intakingLogic() {
        for (int i = 0; i < buffer.getLength(); i++) {
            // bright red
            buffer.setHSV(i, 0, 255, 255);
        }
        SmartDashboard.putString("Debug/LED State", "Intaking");
    }

    private void noteInIntakeLogic() {
        int value = cycleCount % 25 < 13 ? 255 : 0;
        for (int i = 0; i < buffer.getLength(); i++) {
            // bright blue
            buffer.setHSV(i, 120 / 2, 255, value);
        }
        SmartDashboard.putString("Debug/LED State", "Note in Intake");
    }

    private void noteLoadedLogic() {
        for (int i = 0; i < buffer.getLength(); i++) {
            // bright blue
            buffer.setHSV(i, 240 / 2, 255, 255);
        }
        SmartDashboard.putString("Debug/LED State", "Limit switch hit");
    }

    private void shootingAmpLogic() {
        // flashing blue

        int value = 255;
        for (int i = 0; i < buffer.getLength(); i++) {
            // bright blue
            buffer.setHSV(i, 240 / 2, 255, value);
        }
        SmartDashboard.putString("Debug/LED State", "Amp Scoring");
    }

    private void shooterWheelsWarmupLogic() {
        // int value = (int)(255 * (0.5 + (0.5 * (robotContainer.shooter.getLowerSpeakerRPM() / Constants.ManipConstants.shooterSpeakerRPMUpper)))); //increases intensity as motors spin up
        int value = 255; //value <= 255 ? value : 255;
        int speedIndexUpper = (int)(18*robotContainer.shooter.getUpperSpeakerRPM() / Constants.ManipConstants.shooterMaxRPM);
        int speedIndexLower = (int)(18*robotContainer.shooter.getLowerSpeakerRPM() / Constants.ManipConstants.shooterMaxRPM);
        int saturation = (int)(255*(Math.abs(Math.sin((cycleCount)*2*Math.PI/50))));
        for (int i = 0; i < buffer.getLength(); i++) {
            int hue = (int)(((27 + 13*Math.sin(i*2*Math.PI/30)))/2);
            // bright red
            if (i > 68 - speedIndexUpper
                && i <= 68) buffer.setHSV(i, 0, 64, 255);
            else if (i <= 10 + speedIndexLower
                && i >= 10) buffer.setHSV(i, 0, 64, 255);
            else buffer.setHSV(i, hue, saturation, value);
     }
        // for (int i = 10; i < 28; i++) {

        // }
        // for (int i = 51; i < 69; i--) {
        // }
        SmartDashboard.putString("Debug/LED State", "Shooter Warming Up");
    }

    private void shooterWheelsReadyLogic() {
        // int value = cycleCount % 25 < 13 ? 255 : 0; // flash on and off every 0.5 seconds
        for (int i = 0; i < buffer.getLength(); i++) {
            // bright purple
            buffer.setHSV(i, 290 / 2, 255, 255);
        }
        SmartDashboard.putString("Debug/LED State", "Shooter Ready");
    }

    private void climberLimitReached() {
        for (int i = 0; i < buffer.getLength(); i++) {
            // bright purple
            buffer.setHSV(i, 0, 255, 255);
        }
        SmartDashboard.putString("Debug/LED State", "clumb");

    }

    private void shutlePoseLogic() {
        for (int i = 0; i < buffer.getLength(); i++) {
            // bright purple
            buffer.setHSV(i, 66 / 2, 255, 255);
        }
        SmartDashboard.putString("Debug/LED State", "clumb");
    }

}
