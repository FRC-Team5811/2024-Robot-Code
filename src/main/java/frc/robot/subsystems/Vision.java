package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
    PhotonCamera photonCamera;
    int pipeline = 0;

    PhotonPoseEstimator poseEstimator;

    public Vision() {
        photonCamera = new PhotonCamera(Constants.VisionConstants.cameraName);

        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
        poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.CLOSEST_TO_LAST_POSE, Constants.VisionConstants.robotToCamera);
    }

    @Override
    public void periodic() {
    }

    public void setGamePiecePipeline() {
        photonCamera.setPipelineIndex(Constants.VisionConstants.gamePiecePipeline);
    }

    public void setAprilTagPipeline() {
        photonCamera.setPipelineIndex(Constants.VisionConstants.aprilTagPipeline);
    }


}
