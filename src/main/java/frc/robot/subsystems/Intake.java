package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    public final CANSparkMax intakeMotor = new CANSparkMax(17, MotorType.kBrushed);
    public final double intakeSpeed = 1;
    private boolean noteInIntake = false;
    private int cyclesInIntake = 0;
    private DigitalInput limitSwitch = new DigitalInput(2);


    public Intake() {
        
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Debug/Intake Voltage", intakeMotor.getMotorOutputVoltage());
        if (noteInIntake) cyclesInIntake++;
        else if (cyclesInIntake > (int)4*50) {
            noteInIntake = false;
            cyclesInIntake = 0;
        }
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
        return (getLimitBool() && isIntaking());
    }
    

    public boolean getLimitBool() {
        return (!limitSwitch.get());
    }
}