package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.autoRoutines.S1n1n2n3;
import frc.robot.commands.autoRoutines.S1n1n4n5;
import frc.robot.commands.autoRoutines.S1n4n5;
import frc.robot.commands.autoRoutines.S2n1n2n3;
import frc.robot.commands.autoRoutines.S2n2n3n8;
import frc.robot.commands.autoRoutines.S2n2n7n6;
import frc.robot.commands.autoRoutines.S2n3n2n1;
import frc.robot.commands.autoRoutines.S3n3n2n1;
import frc.robot.commands.autoRoutines.S3n3n8n7;
import frc.robot.commands.autoRoutines.S3n8n7;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

    // Subsystems
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public final Intake intake = new Intake();
    public final Indexer indexer = new Indexer();
    public final Shooter shooter = new Shooter();
    public final Climber climber = new Climber();
    public final LEDs leds = new LEDs(this);

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

        CommandScheduler.getInstance().registerSubsystem(swerveSubsystem);
        CommandScheduler.getInstance().registerSubsystem(intake);
        CommandScheduler.getInstance().registerSubsystem(indexer);
        CommandScheduler.getInstance().registerSubsystem(shooter);
        CommandScheduler.getInstance().registerSubsystem(climber);
        CommandScheduler.getInstance().registerSubsystem(leds);

        var s1n1n2n3 = new S1n1n2n3(swerveSubsystem, intake, indexer, shooter);
        var s1n1n4n5 = new S1n1n4n5(swerveSubsystem, intake, indexer, shooter);
        var s1n4n5 = new S1n4n5(swerveSubsystem, intake, indexer, shooter);
        var s2n1n2n3 = new S2n1n2n3(swerveSubsystem, intake, indexer, shooter);
        var s2n2n3n8 = new S2n2n3n8(swerveSubsystem, intake, indexer, shooter);
        var s2n2n7n6 = new S2n2n7n6(swerveSubsystem, intake, indexer, shooter);
        var s2n3n2n1 = new S2n3n2n1(swerveSubsystem, intake, indexer, shooter);
        var s3n3n2n1 = new S3n3n2n1(swerveSubsystem, intake, indexer, shooter);
        var s3n3n8n7 = new S3n3n8n7(swerveSubsystem, intake, indexer, shooter);
        var s3n8n7 = new S3n8n7(swerveSubsystem, intake, indexer, shooter);

        choosableAuto.setDefaultOption("s1n1n2n3", s1n1n2n3);
        choosableAuto.addOption("s1n1n4n5", s1n1n4n5);
        choosableAuto.addOption("s1n4n5", s1n4n5);
        choosableAuto.addOption("s2n1n2n3", s2n1n2n3);
        choosableAuto.addOption("s2n2n3n8", s2n2n3n8);
        choosableAuto.addOption("s2n2n7n6", s2n2n7n6);
        choosableAuto.addOption("s2n3n2n1", s2n3n2n1);
        choosableAuto.addOption("s3n3n2n1", s3n3n2n1);
        choosableAuto.addOption("s3n3n8n7", s3n3n8n7);
        choosableAuto.addOption("s3n8n7", s3n8n7);

        SmartDashboard.putData("Driver/Auto Selection", choosableAuto);

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