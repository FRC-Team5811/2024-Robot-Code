package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

    public final CANSparkMax indexerMotor = new CANSparkMax(15, MotorType.kBrushless);
    private DigitalInput limitSwitch = new DigitalInput(0);
    public final double indexerAmpShotMotorSpeed = 1;
    public final double indexerSpeakerShotMotorSpeed = 1;
    public final double indexerPushSpeed = -1;
    public final double indexerPullSpeed = 1;

    public Indexer () {
        indexerMotor.setInverted(true);
    }

    public void push() {
        indexerMotor.set(indexerPushSpeed);
    }

    public void pull() {
        indexerMotor.set(indexerPullSpeed);
    }

    public void ampScore() {
        indexerMotor.set(indexerAmpShotMotorSpeed);
    }

    public void speakerScore() {
        indexerMotor.set(indexerSpeakerShotMotorSpeed);
    }

    public void stop() {
        indexerMotor.stopMotor();
    }

    public boolean getLimitBool() {
        return limitSwitch.get();
    }

    public boolean isNoteLoaded() {
        return getLimitBool();
    }

}