package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PID;

public class Shooter extends SubsystemBase {
    public final CANSparkMax speakerLowerMotor = new CANSparkMax(12, MotorType.kBrushless);
    public final RelativeEncoder shooterEncoderLower;
    public final CANSparkMax speakerUpperMotor = new CANSparkMax(13, MotorType.kBrushless);
    public final RelativeEncoder shooterEncoderUpper;
    public final CANSparkMax diverterMotor = new CANSparkMax(14, MotorType.kBrushless);
    public PID shooterPIDLower = new PID(Constants.ManipConstants.shooterControlP, 
        Constants.ManipConstants.shooterControlI, 
        Constants.ManipConstants.shooterControlD);
    public PID shooterPIDUpper = new PID(Constants.ManipConstants.shooterControlP, 
        Constants.ManipConstants.shooterControlI, 
        Constants.ManipConstants.shooterControlD);

    public final double diverterAmpShotMotorSpeed = 1;
    public final double diverterSpeakerShotMotorSpeed = -1;

    public Shooter() {
        shooterEncoderLower = speakerLowerMotor.getEncoder();
        shooterEncoderUpper = speakerUpperMotor.getEncoder();

        diverterMotor.setInverted(true);
        speakerLowerMotor.setInverted(true);
    }

    public void autoSpeakerShotRampUp() {
        double shooterSetpointRPMLower = Constants.ManipConstants.shooterSpeakerRPMLower;
        double shooterSetpointRPMUpper = Constants.ManipConstants.shooterSpeakerRPMUpper;
        double maxRPM = Constants.ManipConstants.shooterMaxRPM;

        double rpmLower = shooterEncoderLower.getVelocity();
        double rpmUpper = shooterEncoderUpper.getVelocity();
        double expectedOutputLower = (1.0/maxRPM) * (shooterSetpointRPMLower);
        double expectedOutputUpper = (1.0/maxRPM) * (shooterSetpointRPMUpper);
        double pidOutputLower = (1.0/maxRPM) * shooterPIDLower.calculate(rpmLower, shooterSetpointRPMLower);
        double pidOutputUpper = (1.0/maxRPM) * shooterPIDUpper.calculate(rpmUpper, shooterSetpointRPMUpper);
        
        speakerLowerMotor.set(expectedOutputLower + pidOutputLower);
        speakerUpperMotor.set(expectedOutputUpper + pidOutputUpper);
    }

    public void autoSpeakerShotRampUp(double speakerRPM) {
        double shooterSetpointRPMLower = speakerRPM;
        double shooterSetpointRPMUpper = speakerRPM;
        double maxRPM = Constants.ManipConstants.shooterMaxRPM;

        double rpmLower = shooterEncoderLower.getVelocity();
        double rpmUpper = shooterEncoderUpper.getVelocity();
        double expectedOutputLower = (1.0/maxRPM) * (shooterSetpointRPMLower);
        double expectedOutputUpper = (1.0/maxRPM) * (shooterSetpointRPMUpper);
        double pidOutputLower = (1.0/maxRPM) * shooterPIDLower.calculate(rpmLower, shooterSetpointRPMLower);
        double pidOutputUpper = (1.0/maxRPM) * shooterPIDUpper.calculate(rpmUpper, shooterSetpointRPMUpper);
        
        speakerLowerMotor.set(expectedOutputLower + pidOutputLower);
        speakerUpperMotor.set(expectedOutputUpper + pidOutputUpper);
    }

    public double getLowerSpeakerRPM() {
        return (shooterEncoderLower.getVelocity());
    }

    public double getUpperSpeakerRPM() {
        return (shooterEncoderUpper.getVelocity());
    }

    public boolean isShooterWheelsWarmup() {
        return (
            speakerLowerMotor.get() > 0.1 &&
            speakerUpperMotor.get() > 0.1
            );
    }

    public boolean isShooterWheelsReady() {
        return (
            getLowerSpeakerRPM() > 0.95 * Constants.ManipConstants.shooterSpeakerRPMLower &&
            getUpperSpeakerRPM() > 0.95 * Constants.ManipConstants.shooterSpeakerRPMUpper
            );
    }

    public void manualSpeakerMotors(double speed) {
        speakerUpperMotor.set(speed);
        speakerLowerMotor.set(speed);
    }

    public void stopSpeakerMotors() {
        speakerUpperMotor.stopMotor();
        speakerLowerMotor.stopMotor();
    }

    public void runAmpDiverter() {
        diverterMotor.set(diverterAmpShotMotorSpeed);
    }

    public boolean isShootingAmp() {
        return diverterMotor.get() == diverterAmpShotMotorSpeed;
    }

    public void runSpeakerDiverter() {
        diverterMotor.set(diverterSpeakerShotMotorSpeed);
    }

    public void stopDiverter() {
        diverterMotor.stopMotor();
    }

}