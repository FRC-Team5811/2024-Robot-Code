package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    public final CANSparkMax climberMotor = new CANSparkMax(16, MotorType.kBrushless);
    public final double climbSpeed = 1;
    private final DigitalInput limitSwitch = new DigitalInput(1);

    public Climber () {
        climberMotor.setInverted(false);
    }

    public void up() {
        climberMotor.set(climbSpeed);
    }

    public void down() {
        climberMotor.set(-climbSpeed);
    }

    public void stop() {
        climberMotor.stopMotor();
    }

    public boolean getLimitBool() {
        return limitSwitch.get();
    }


}