import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class Vision extends SubsystemBase {

private final PhotonCamera mainCamera;
private final PhotonCamera noteCamera;

private final PhotonPoseEstimator visionEstimator;
public Drivetrain swerveDrive;
public Vision (Drivetrain swerveDrive) {
 this.swerveDrive =  swerveDrive;
this.mainCamera = new PhotonCamera(DrivetrainConstants.kMainCameraName);
this.noteCamera = new PhotonCamera(DrivetrainConstants.kNoteCameraName);

this.visionEstimator =
    new PhotonPoseEstimator(
        DrivetrainConstants.kAprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        mainCamera,
        DrivetrainConstants.kRobotToCameraTransform);

 }
        private void fuseVision() {
            Optional<EstimatedRobotPose> visionEstimate = visionEstimator.update();
        
            if (visionEstimate.isEmpty()) return;
        
            this.swerveDrive.addVisionMeasurement(
                visionEstimate.get().estimatedPose.toPose2d(), visionEstimate.get().timestampSeconds);
        
            SmartDashboard.putNumber("vision estimate x", visionEstimate.get().estimatedPose.getX());
            SmartDashboard.putNumber("vision estimate y", visionEstimate.get().estimatedPose.getY());
            SmartDashboard.putNumber("vision estimate z", visionEstimate.get().estimatedPose.getZ());
          }

          @Override
          public void periodic() {
            // this.odometry.update(this.gyro.getRotation2d(), this.getModulePositions());
            this.fuseVision();

          }
          }