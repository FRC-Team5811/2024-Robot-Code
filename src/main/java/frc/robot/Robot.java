package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoAmpSequence;
import frc.robot.commands.AutoIntakeSequence;
import frc.robot.commands.AutoSpeakerSequence;
import frc.robot.commands.IndexerTeleopCmd;
import frc.robot.commands.IntakeTeleopCmd;
import frc.robot.commands.ShooterTeleopCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;


public class Robot extends TimedRobot {

    private RobotContainer robotContainer;
    private Command autonomousCommand;

    private int count = 0;

    // private Pose2d startPose;
    // private Translation2d secondPose;
    // private Translation2d thirdPose;
    // private Pose2d endPose;
    // private Pose2d endPose1;
    private Pose2d pose1;
    private Pose2d pose2;
    private Pose2d pose3;
    private Pose2d pose4;
    private Pose2d pose5;

    

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        configureButtonBindings();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().cancelAll();

        // zeros robot position
        robotContainer.swerveSubsystem.zeroHeading();
        robotContainer.swerveSubsystem.resetOdometry(new Pose2d(0,0, new Rotation2d(0)));

        // sets positions for auto commands
        // startPose = robotContainer.swerveSubsystem.getPose();
        // Rotation2d startRotation = startPose.getRotation();
        // SmartDashboard.putString("Start pose", startPose.toString());

        // secondPose = new Translation2d(-0.4,0.53);
        // thirdPose = new Translation2d(-0.4, -0.53);
        // endPose = new Pose2d(-4.0, 0.53, startRotation);
        // endPose1 = new Pose2d(-4.0, -0.53, startRotation);
        Rotation2d startRotation = robotContainer.swerveSubsystem.getPose().getRotation();
        pose1 = new Pose2d(1, 0, new Rotation2d(1*Math.PI).plus(startRotation)); //new Rotation2d(0.4*Math.PI).plus(startRotation));
        pose2 = new Pose2d(-1, -1, new Rotation2d(-0.5*Math.PI).plus(startRotation)); //new Rotation2d(-0.4*Math.PI).plus(startRotation));
        pose3 = new Pose2d(0, 0, startRotation);
        pose4 = new Pose2d(0, 0, startRotation);                
        pose5 = new Pose2d(0, 0, startRotation);

        // creats a auto command object
        autonomousCommand = robotContainer.getAutonomousCommand(robotContainer.swerveSubsystem, pose1, pose2, pose3, pose4, pose5);

        // runs auto command if it exist
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
            SmartDashboard.putBoolean("Auto Active", isAutonomous());
        }
    }

    // this function is called periodically during autonomous.
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();

        // swerve drive controls on drive controller
        robotContainer.swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
            robotContainer.swerveSubsystem,
            () -> robotContainer.driverController.getRawAxis(OIConstants.kDriverYAxis),
            () -> robotContainer.driverController.getRawAxis(OIConstants.kDriverXAxis),
            () -> robotContainer.driverController.getRawAxis(OIConstants.kDriverRotAxis),
            () -> !robotContainer.driverController.getRawButton(Constants.OIConstants.RightTriggerButton)
            ));

        robotContainer.intake.setDefaultCommand(new IntakeTeleopCmd(
            robotContainer.intake,
            () -> robotContainer.manipController.getRawButton(OIConstants.intakeManualButton),
            () -> robotContainer.manipPOVDownValue
            ));

        robotContainer.indexer.setDefaultCommand(new IndexerTeleopCmd(
            robotContainer.indexer,
            () -> robotContainer.manipController.getRawButton(OIConstants.intakeManualButton),
            () -> robotContainer.manipPOVDownValue,
            () -> robotContainer.manipController.getRawButton(OIConstants.ampScoreManualButton),
            () -> robotContainer.manipController.getRawButton(OIConstants.speakerScoreManualButton)
            ));

        robotContainer.shooter.setDefaultCommand(new ShooterTeleopCmd(
            robotContainer.shooter,
            () -> robotContainer.manipController.getRawAxis(OIConstants.shooterManualAxis),
            () -> robotContainer.manipController.getRawButton(OIConstants.shooterManualButton),
            () -> robotContainer.manipController.getRawButton(OIConstants.ampScoreManualButton)
            ));

        /* this makes sure that the autonomous stops running when
        teleop starts running. If you want the autonomous to
        continue until interrupted by another command, remove
        this line or comment it out */
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        robotContainer.swerveSubsystem.resetModuleEncoders();

        // temporary, used for reading absolute encoder values
        // robotContainer.swerveSubsystem.printEncoders();
        // System.out.println("ENCOOODEEERRR: " + robotContainer.swerveSubsystem.getHeading());

    }

    // this function is called periodically during operator control
    @Override
    public void teleopPeriodic() {


    }

    @Override
    public void testInit() {
        // cancels all running commands at the start of test mode
        CommandScheduler.getInstance().cancelAll();
    }

    // this function is called periodically during test mode
    @Override
    public void testPeriodic() {
    }

    private void configureButtonBindings() {
        new JoystickButton(robotContainer.driverController, Constants.OIConstants.RightBumperButton).onTrue(new InstantCommand(() -> robotContainer.swerveSubsystem.zeroHeading(), robotContainer.swerveSubsystem));
        new JoystickButton(robotContainer.driverController, Constants.OIConstants.LeftBumperButton).whileTrue(new RunCommand(() -> SwerveJoystickCmd.slowJoe = true));
        new JoystickButton(robotContainer.driverController, Constants.OIConstants.LeftBumperButton).whileFalse(new RunCommand(() -> SwerveJoystickCmd.slowJoe = false));

        new JoystickButton(robotContainer.manipController, Constants.OIConstants.intakeSequenceButton).onTrue(
            new AutoIntakeSequence(robotContainer.intake, robotContainer.indexer, false));
        new JoystickButton(robotContainer.manipController, Constants.OIConstants.ampShotSequenceButton).onTrue(
            new AutoAmpSequence(robotContainer.shooter, robotContainer.indexer));
        new JoystickButton(robotContainer.manipController, Constants.OIConstants.speakerShotSequenceButton).onTrue(
            new AutoSpeakerSequence(robotContainer.shooter, robotContainer.indexer));
    }
}
