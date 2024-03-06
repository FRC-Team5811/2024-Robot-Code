package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.PID;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class AutoDriveToPoint extends Command {

    SwerveSubsystem swerveSubsystem;
    Pose2d targetPose;
    PID xController;
    PID yController;
    PID thetaController;
    ChassisSpeeds prevChassisSpeeds;
    boolean stop;  

    public AutoDriveToPoint(
      SwerveSubsystem swerveSubsystem,
      Pose2d targetPose
      )
        {
    this.swerveSubsystem = swerveSubsystem;
    this.targetPose = targetPose;
    stop = false;
    addRequirements(swerveSubsystem);
  }

    public AutoDriveToPoint(
      SwerveSubsystem swerveSubsystem,
      Pose2d targetPose, boolean stop
      )
        {
    this.swerveSubsystem = swerveSubsystem;
    this.targetPose = targetPose;
    this.stop = stop;
    addRequirements(swerveSubsystem);
  }

@Override
  public void initialize() {
  // xController = new ProfiledPIDController(2, 0, 0, new TrapezoidProfile.Constraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
  // Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
  // yController = new ProfiledPIDController(2, 0, 0, new TrapezoidProfile.Constraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
  // Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
  // thetaController = new ProfiledPIDController(2, 0, 0, new TrapezoidProfile.Constraints(
  //                       Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond,
  //                       Constants.AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared));
  // thetaController.enableContinuousInput(-Math.PI, Math.PI);
  xController = new PID(5, 0, 1);
  yController = new PID(5, 0, 1);
  thetaController = new PID(8, 0, 0.1);
  thetaController.enableContinuousInput(-Math.PI, Math.PI);

  prevChassisSpeeds = swerveSubsystem.getChassisSpeeds();

  targetPose = Robot.processPoseWithAllianceColor(targetPose);
  SmartDashboard.putString("target pose", targetPose.toString());


  // Pose2d currentPose = swerveSubsystem.getPose();
  // xController.reset(currentPose.getX());
  // yController.reset(currentPose.getY());
  // thetaController.reset(currentPose.getRotation().getRadians());

  // xController.setGoal(targetPose.getX());
  // yController.setGoal(targetPose.getY());
  // thetaController.setGoal(targetPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    
    Pose2d currentPose = swerveSubsystem.getPose();
    double xPower = xController.calculate(currentPose.getX(), targetPose.getX());
    double yPower = yController.calculate(currentPose.getY(), targetPose.getY());
    SmartDashboard.putNumber("xPower", xPower);
    SmartDashboard.putNumber("yPower", yPower);

    double thetaPower = thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    
    var targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xPower, yPower, thetaPower, swerveSubsystem.getPose().getRotation());
    SmartDashboard.putString("Auto speeds relative", targetChassisSpeeds.toString());

    Pose2d vD = new Pose2d(targetChassisSpeeds.vxMetersPerSecond - prevChassisSpeeds.vxMetersPerSecond, 
                           targetChassisSpeeds.vyMetersPerSecond - prevChassisSpeeds.vyMetersPerSecond,
                           new Rotation2d());

    double accMagnitude = Math.sqrt(vD.getX()*vD.getX() + vD.getY()*vD.getY());

    if (accMagnitude > Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared*0.02) {
      targetChassisSpeeds.vxMetersPerSecond = prevChassisSpeeds.vxMetersPerSecond + Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared*0.02*vD.getX() / accMagnitude;
      targetChassisSpeeds.vyMetersPerSecond = prevChassisSpeeds.vyMetersPerSecond + Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared*0.02*vD.getY() / accMagnitude;
    }

    if (Math.abs(prevChassisSpeeds.omegaRadiansPerSecond - targetChassisSpeeds.omegaRadiansPerSecond) > Constants.AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared*0.02) {
      targetChassisSpeeds.omegaRadiansPerSecond = prevChassisSpeeds.omegaRadiansPerSecond + Constants.AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared*0.02*targetChassisSpeeds.omegaRadiansPerSecond / Math.abs(targetChassisSpeeds.omegaRadiansPerSecond);
    }



    double speedMagnitude = Math.sqrt(targetChassisSpeeds.vxMetersPerSecond*targetChassisSpeeds.vxMetersPerSecond + targetChassisSpeeds.vyMetersPerSecond*targetChassisSpeeds.vyMetersPerSecond);
    double omegaMagnitude = Math.abs(targetChassisSpeeds.omegaRadiansPerSecond);
    if (speedMagnitude > Constants.AutoConstants.kMaxSpeedMetersPerSecond) {
          targetChassisSpeeds.vxMetersPerSecond = Constants.AutoConstants.kMaxSpeedMetersPerSecond*targetChassisSpeeds.vxMetersPerSecond/speedMagnitude;
          targetChassisSpeeds.vyMetersPerSecond = Constants.AutoConstants.kMaxSpeedMetersPerSecond*targetChassisSpeeds.vyMetersPerSecond/speedMagnitude;
        }
    if (omegaMagnitude > Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond) {
          targetChassisSpeeds.omegaRadiansPerSecond = Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond*targetChassisSpeeds.omegaRadiansPerSecond/omegaMagnitude;
        }

    prevChassisSpeeds = targetChassisSpeeds;
    
    // targetChassisSpeeds.vxMetersPerSecond = 0;
    // targetChassisSpeeds.vyMetersPerSecond = 0;
    
    var targetModuleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds);
    swerveSubsystem.setModuleStates(targetModuleStates);
    if (Constants.dashboardDebugMode)
        SmartDashboard.putString("Auto speeds", targetChassisSpeeds.toString());
  }

  @Override
  public void end(boolean interrupted) {
    System.err.println("end reached");
    var targetChassisSpeeds = new ChassisSpeeds(0, 0, 0);
    var targetModuleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds);
    swerveSubsystem.setModuleStates(targetModuleStates);
  }

  @Override
  public boolean isFinished() {
    Pose2d currentPose = swerveSubsystem.getPose();
    if (Math.abs(currentPose.getX() - targetPose.getX()) < 0.05 
    && Math.abs(currentPose.getY() - targetPose.getY()) < 0.05 
    && Math.abs(currentPose.getRotation().getDegrees() - targetPose.getRotation().getDegrees()) < 5) {
      if (stop) {
        if (Math.abs(prevChassisSpeeds.vxMetersPerSecond) > 0.1
            || Math.abs(prevChassisSpeeds.vyMetersPerSecond) > 0.1
            || Math.abs(prevChassisSpeeds.omegaRadiansPerSecond) > 0.2) {
          return false;
      }
    }
    return true;
    }
    return false;
  }
}
