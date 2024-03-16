package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AmpSequence;
import frc.robot.commands.ClimberTeleopCmd;
import frc.robot.commands.IntakeSequence;
import frc.robot.commands.SpeakerSequence;
import frc.robot.commands.IndexerTeleopCmd;
import frc.robot.commands.IntakeTeleopCmd;
import frc.robot.commands.ShooterTeleopCmd;
import frc.robot.commands.SwerveTeleopCmd;


public class Robot extends TimedRobot {

    private RobotContainer robotContainer;
    private Timer timer;

    private boolean delayedInitialized = false;

    public static boolean allianceColorKnown = false;
    public static Alliance allianceColor = Alliance.Blue;
    private double lastAllianceColorCheck = -1;

    private Field2d driverField = new Field2d();
    private Field2d debugField = new Field2d();

    private static Robot instance = null;

    @Override
    public void robotInit() {
        instance = this;

        robotContainer = new RobotContainer();

        configureButtonBindings();

        timer = new Timer();
        timer.start();
    }

    private void delayedInit() {
        delayedInitialized = true;
        robotContainer.swerveSubsystem.zeroGyroAngle();
        robotContainer.swerveSubsystem.resetModuleEncoders();
        SmartDashboard.putString("Debug/YESS", "YESSSS");

        SmartDashboard.putData("Driver/Field", driverField);
        SmartDashboard.putData("Debug/Field", debugField);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        if (!delayedInitialized && timer.get() > 5)
            delayedInit();

        SmartDashboard.putBoolean("Driver/limit switch", robotContainer.indexer.getLimitBool());

        updateAllianceColor();
        driverField.setRobotPose(robotContainer.swerveSubsystem.getPose2d());
        debugField.setRobotPose(robotContainer.swerveSubsystem.getPose2d());
    }

    public static boolean isInstanceDisabled() {
        if (instance == null) return true;
        return instance.isDisabled();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        robotContainer.swerveSubsystem.removeDefaultCommand();
        robotContainer.intake.removeDefaultCommand();
        robotContainer.indexer.removeDefaultCommand();
        robotContainer.shooter.removeDefaultCommand();
        robotContainer.climber.removeDefaultCommand();

        CommandScheduler.getInstance().cancelAll();

        Command autoCommand = robotContainer.choosableAuto.getSelected();
        autoCommand.schedule();
    }

    // this function is called periodically during autonomous
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        
        CommandScheduler.getInstance().cancelAll();
        /* this makes sure that the autonomous stops running when
        teleop starts running. If you want the autonomous to
        continue until interrupted by another command, remove
        this line or comment it out */

        robotContainer.swerveSubsystem.resetModuleEncoders();

        robotContainer.swerveSubsystem.resetOdometry(Robot.processPoseWithAllianceColor(Constants.AutoConstants.start2Pose));

        setupDefaultCommands();
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

    private void setupDefaultCommands() {
        // manual swerve drive controls on drive controller
        robotContainer.swerveSubsystem.setDefaultCommand(new SwerveTeleopCmd(
            robotContainer.swerveSubsystem,
            () -> robotContainer.driverController.getRawAxis(OIConstants.kDriverYAxis),
            () -> robotContainer.driverController.getRawAxis(OIConstants.kDriverXAxis),
            () -> robotContainer.driverController.getRawAxis(OIConstants.kDriverRotAxis),
            () -> robotContainer.driverController.getRawButton(OIConstants.driverFieldOrientedButton),
            () -> robotContainer.driverController.getRawButton(OIConstants.driverResetHeadingButton),
            () -> robotContainer.driverController.getRawAxis(OIConstants.driverSlowModeButton),
            () -> robotContainer.driverController.getRawAxis(OIConstants.driverBreakButton),
            () -> robotContainer.driverController.getRawButton(OIConstants.driverForwardSetpointButton),
            () -> robotContainer.driverController.getRawButton(OIConstants.driverLeftSetpointButton),
            () -> robotContainer.driverController.getRawButton(OIConstants.driverBackSetpointButton),
            () -> robotContainer.driverController.getRawButton(OIConstants.driverRightSetpointButton)
            ));

        // manual intake controls on manip controller
        robotContainer.intake.setDefaultCommand(new IntakeTeleopCmd(
            robotContainer.intake,
            () -> robotContainer.manipController.getRawButton(OIConstants.intakeManualButton),
            () -> false
            ));

        // manual indexer controls on manip controller
        robotContainer.indexer.setDefaultCommand(new IndexerTeleopCmd(
            robotContainer.indexer,
            () -> robotContainer.manipController.getRawButton(OIConstants.intakeManualButton),
            () -> false,
            () -> robotContainer.manipController.getRawButton(OIConstants.ampScoreManualButton),
            () -> false
            ));

        // manual shooter controls on manip controller
        robotContainer.shooter.setDefaultCommand(new ShooterTeleopCmd(
            robotContainer.shooter,
            () -> robotContainer.manipController.getRawAxis(OIConstants.shooterManualAxis),
            () -> robotContainer.manipController.getRawButton(OIConstants.speakerShotSequenceButton),
            () -> robotContainer.manipController.getRawButton(OIConstants.ampScoreManualButton),
            () -> robotContainer.manipController.getRawButton(OIConstants.shooterRampUpButton),
            () -> robotContainer.manipController.getRawButton(OIConstants.diverterSpeakerFireButton)
            ));
        // manual climber controls on manip controller
        robotContainer.climber.setDefaultCommand(new ClimberTeleopCmd(
            robotContainer.climber,
            () -> robotContainer.manipPOVUpValue, 
            () -> robotContainer.manipPOVDownValue));
    }

    private void configureButtonBindings() {
        new JoystickButton(robotContainer.manipController, Constants.OIConstants.intakeSequenceButton).onTrue(
            new IntakeSequence(robotContainer.intake, robotContainer.indexer, true));
        new JoystickButton(robotContainer.manipController, Constants.OIConstants.ampShotSequenceButton).onTrue(
            new AmpSequence(robotContainer.shooter, robotContainer.indexer));
        new JoystickButton(robotContainer.manipController, Constants.OIConstants.speakerShotSequenceButton).onTrue(
            new SpeakerSequence(robotContainer.shooter, robotContainer.indexer));
    }

    private void updateAllianceColor() {
        if (timer.get() - lastAllianceColorCheck > 1.0) {
            Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
            lastAllianceColorCheck = timer.get();

            boolean blue = false;
            boolean red = false;
            if (optionalAlliance.isPresent()) {
                allianceColor = optionalAlliance.get();
                allianceColorKnown = true;

                if (allianceColor == Alliance.Blue)
                    blue = true;
                else if (allianceColor == Alliance.Red)
                    red = true;  
            }
            else {
                allianceColor = Alliance.Blue;
                allianceColorKnown = false;

                blue = false;
                red = false;
            }

            SmartDashboard.putBoolean("Driver/Blue Alliance", blue);
            SmartDashboard.putBoolean("Driver/Red Alliance", red);
        }
    }

    public static Pose2d processPoseWithAllianceColor(Pose2d pose) {
        if (allianceColor == Alliance.Blue)
            return pose;

        double x = pose.getX();
        double y = pose.getY();
        Rotation2d rotation = pose.getRotation();

        return new Pose2d(16.591 - x, y, Rotation2d.fromDegrees(180).minus(rotation));
    }
}
