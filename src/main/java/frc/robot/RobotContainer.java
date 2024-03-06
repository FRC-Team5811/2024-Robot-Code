package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoDriveToPoint;
import frc.robot.commands.SwerveTeleopCmd;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.AutoTest1;
import frc.robot.commands.ResetSwervePoseCmd;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class RobotContainer {

    // Subsystems
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public final Intake intake = new Intake();
    public final Indexer indexer = new Indexer();
    public final Shooter shooter = new Shooter();

    // Controllers
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

    // Auto
    public SendableChooser<Command> choosableAuto = new SendableChooser<>();

    public RobotContainer() {

        SequentialCommandGroup testAuto0 = new SequentialCommandGroup(
            new ResetSwervePoseCmd(swerveSubsystem, new Pose2d(2, 2, new Rotation2d())),
            new WaitCommand(2),
            new AutoDriveToPoint(swerveSubsystem, new Pose2d(3, 2.5, new Rotation2d()), true)
        );
        SequentialCommandGroup testAuto1 = new SequentialCommandGroup(
            new ResetSwervePoseCmd(swerveSubsystem, new Pose2d(2, 2, new Rotation2d())),
            new WaitCommand(2),
            new AutoDriveToPoint(swerveSubsystem, new Pose2d(2.5, 3, new Rotation2d()), true)
        );

        choosableAuto.setDefaultOption("Test Auto 0", testAuto0);
        choosableAuto.addOption("Test Auto 1", testAuto1);
        SmartDashboard.putData("Auto Selection", choosableAuto);

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
}