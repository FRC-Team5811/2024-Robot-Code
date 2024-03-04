package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;

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
        // robotContainer.swerveSubsystem.gyro.calibrate();
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
            () -> robotContainer.driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
            () -> robotContainer.driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
            () -> robotContainer.driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
            () -> !robotContainer.driverJoytick.getRawButton(Constants.OIConstants.RightTriggerButton)));

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
        double axis0 = robotContainer.manipJoytick.getRawAxis(0);
        double axis1 = robotContainer.manipJoytick.getRawAxis(1);
        double axis2 = robotContainer.manipJoytick.getRawAxis(2);
        double axis3 = robotContainer.manipJoytick.getRawAxis(3);

        if (axis0 < -0.5)
            runMotorsSpeakerPID();
        else
        {
            robotContainer.motor0.set(0);
            robotContainer.motor1.set(0);
        }

        robotContainer.motor2.set(axis0);
        robotContainer.motor3.set(axis3);
        robotContainer.motor4.set(axis2);

        SmartDashboard.putNumber("axis0", axis0);
        SmartDashboard.putNumber("axis1", axis1);
        SmartDashboard.putNumber("axis2", axis2);
        SmartDashboard.putNumber("axis3", axis3);
    }

    private void runMotorsSpeakerPID() {
        double shooterSetpointRPMLower = Constants.ManipConstants.shooterSpeakerRPMLower;
        double shooterSetpointRPMUpper = Constants.ManipConstants.shooterSpeakerRPMUpper;
        double maxRPM = Constants.ManipConstants.shooterMaxRPM;

        double rpmLower = robotContainer.shooterEncoderLower.getVelocity();
        double rpmUpper = robotContainer.shooterEncoderUpper.getVelocity();
        double expectedOutputLower = (1.0/maxRPM) * (shooterSetpointRPMLower);
        double expectedOutputUpper = (1.0/maxRPM) * (shooterSetpointRPMUpper);
        double pidOutputLower = (1.0/maxRPM) * robotContainer.shooterPIDLower.calculate(rpmLower, shooterSetpointRPMLower);
        double pidOutputUpper = (1.0/maxRPM) * robotContainer.shooterPIDUpper.calculate(rpmUpper, shooterSetpointRPMUpper);
        
        robotContainer.motor0.set(expectedOutputLower + pidOutputLower);
        robotContainer.motor1.set(expectedOutputUpper + pidOutputUpper);
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
}
