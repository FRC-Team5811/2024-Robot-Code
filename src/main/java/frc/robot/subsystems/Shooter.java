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

    public boolean speakersAtSpeed() {
        return (shooterEncoderLower.getVelocity() >= Constants.ManipConstants.shooterSpeakerRPMLower);
    }

    public double getSpeakersRPM() {
        return (shooterEncoderLower.getVelocity());
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

    public void runSpeakerDiverter() {
        diverterMotor.set(diverterSpeakerShotMotorSpeed);
    }

    public void stopDiverter() {
        diverterMotor.stopMotor();
    }

}