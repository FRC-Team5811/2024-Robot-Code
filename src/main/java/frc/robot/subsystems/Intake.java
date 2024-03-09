package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    public final WPI_VictorSPX intakeMotor = new WPI_VictorSPX(16);
    public final double intakeSpeed = 1;

    public Intake() {
        
    }

    public void pull() {
        intakeMotor.set(intakeSpeed);
    }

    public void push() {
        intakeMotor.set(-intakeSpeed);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }
}