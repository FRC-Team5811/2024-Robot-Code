// package frc.robot.subsystems;

// import org.photonvision.PhotonCamera;

// // import org.photonvision.PhotonCamera;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.CvSink;
// import edu.wpi.first.cscore.CvSource;
// import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.net.PortForwarder;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class Vision extends SubsystemBase {
//     PhotonCamera photonCamera;
//     UsbCamera usbCamera;
//     // int pipeline = 0;

//     // PhotonPoseEstimator poseEstimator;

//     public Vision() {
//         photonCamera = new PhotonCamera(Constants.VisionConstants.cameraName);
//         CameraServer.startAutomaticCapture();
//         // CvSink sink = CameraServer.getVideo();
//         // CvSource output = CameraServer.putVideo("DriverCam", 320, 240);

//         // AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
//         // PortForwarder.add(5800, "10.58.11.11", 5800);
//         // PortForwarder.add(1181, "10.58.11.11", 1181);
//         // PortForwarder.add(1182, "10.58.11.11", 1182);
//         // poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.CLOSEST_TO_LAST_POSE, Constants.VisionConstants.robotToCamera);
//     }

//     @Override
//     public void periodic() {
//     }

//     public void setGamePiecePipeline() {
//         // photonCamera.setPipelineIndex(Constants.VisionConstants.gamePiecePipeline);
//     }

//     public void setAprilTagPipeline() {
//         // photonCamera.setPipelineIndex(Constants.VisionConstants.aprilTagPipeline);
//     }


// }
