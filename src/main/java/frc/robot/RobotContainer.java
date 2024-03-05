package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoDriveToPoint;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.AutoTest1;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class RobotContainer {

    // Subsystems
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public final Intake intake = new Intake();
    public final Indexer indexer = new Indexer();
    public final Shooter shooter = new Shooter();

    // Joysticks
    public final Joystick driverController = new Joystick(OIConstants.kDriverControllerPort);
    public final Joystick manipController = new Joystick(OIConstants.kManipControllerPort);
    public final POVButton manipPOVButtonUp = new POVButton(manipController, 0);
    public final POVButton manipPOVButtonDown = new POVButton(manipController, 180);
    public final POVButton manipPOVButtonRight = new POVButton(manipController, 90);
    public final POVButton manipPOVButtonLeft = new POVButton(manipController, 270);
    public boolean manipPOVUpValue = false;
    public boolean manipPOVDownValue = false;
    public boolean manipPOVRightValue = false;
    public boolean manipPOVLeftValue = false;

    public final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    public final Joystick manipJoytick = new Joystick(OIConstants.kManipControllerPort);

    // POVButton UpPov = new POVButton(manipJoytick, 0);
    // POVButton DownPov = new POVButton(manipJoytick, 180);
    // POVButton RightPov = new POVButton(manipJoytick, 90);
    // POVButton LeftPov = new POVButton(manipJoytick, 270);

    public RobotContainer() {
        
        SmartDashboard.putNumber("Auto", 1);

        // POV buttons work differently... let's store the raw value on changed
        // would be cleaner to have a dedicated "controller" wrapper class, but we may not even use this code
        manipPOVButtonUp.onTrue(new InstantCommand(() -> manipPOVUpValue = true));
        manipPOVButtonUp.onFalse(new InstantCommand(() -> manipPOVUpValue = false));
        manipPOVButtonDown.onTrue(new InstantCommand(() -> manipPOVDownValue = true));
        manipPOVButtonDown.onFalse(new InstantCommand(() -> manipPOVDownValue = false));
        manipPOVButtonRight.onTrue(new InstantCommand(() -> manipPOVRightValue = true));
        manipPOVButtonRight.onFalse(new InstantCommand(() -> manipPOVRightValue = false));
        manipPOVButtonLeft.onTrue(new InstantCommand(() -> manipPOVLeftValue = true));
        manipPOVButtonLeft.onFalse(new InstantCommand(() -> manipPOVLeftValue = false));
    }

    public Command getAutonomousCommand(SwerveSubsystem swerveSubsystem, Pose2d pose1, Pose2d pose2, Pose2d pose3, Pose2d pose4, Pose2d pose5) {
        // creates tabs on the SmartDashboard to selecty Auto and to know what each auto does
        double number = SmartDashboard.getNumber("Auto", 1);

        if(number == 1) {
            return new AutoTest1(swerveSubsystem, pose1, pose2, pose3);
        }

        // // place one game piece
        // if(number == 1){
        //     return new Auto1(intakeSubsystem, armMechSubsystem, swerveSubsystem, startPose, secondPose, endPose);
        // }

        // // place one game piece and engage the charging station
        // if(number == 2){
        //     return new Auto2(armMechSubsystem, intakeSubsystem);
        // }

        // // plaace game piece, grab another game peice and place the one 
        // if(number == 3){
        //     return new Auto1(intakeSubsystem, armMechSubsystem, swerveSubsystem, startPose, thirdPose, endPose1);

        // }

        else{
            return null;
        }
    }

}