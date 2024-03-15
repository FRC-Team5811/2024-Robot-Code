package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    public final WPI_VictorSPX intakeMotor = new WPI_VictorSPX(16);
    public final double intakeSpeed = 1;
    private boolean noteInIntake = false;

    public Intake() {
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Debug/Intake Voltage", intakeMotor.getMotorOutputVoltage());
    }

    public void pull() {
        intakeMotor.set(intakeSpeed);
    }

    public void push() {
        intakeMotor.set(-intakeSpeed);
    }

    public void stop() {
        intakeMotor.stopMotor();
        noteInIntake = false;
    }

    public boolean isIntaking() {
        return intakeMotor.get() > 0.1;
    }

    public boolean isNoteInIntake() {
        // TODO fix this with note in intake detection!!
        if (isIntaking() && intakeMotor.getMotorOutputVoltage() < 11.5) {
            noteInIntake = true;
        }
        return noteInIntake;
    }
}