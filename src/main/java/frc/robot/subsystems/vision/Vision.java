package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class Vision extends SubsystemBase {
  
  private static Vision instance = null;

  public static Vision getInstance() {
    if (instance == null)
      try {
        instance = new Vision();
      } catch (IOException e) {
        e.printStackTrace();
      }

    return instance;
  }

  private final PhotonCamera noteCamera;
  
  private final PhotonPoseEstimator visionEstimator;
  public Drivetrain swerveDrive;
  public Vision (Drivetrain swerveDrive) {
   this.swerveDrive =  swerveDrive;
   
  this.noteCamera = new PhotonCamera(DrivetrainConstants.kNoteCameraName);
  }
  private Vision() throws IOException {
    this.swerveDrive = swerveDrive;
    this.noteCamera = new PhotonCamera(DrivetrainConstants.kNoteCameraName);
  }

  private double getNoteAngle() {
    PhotonPipelineResult latestResult = this.noteCamera.getLatestResult();
    if (!latestResult.hasTargets()) {
      return 0.0;
    }

    PhotonTrackedTarget trackedNote = new PhotonTrackedTarget();
    return -0.5 * trackedNote.getYaw();
  }
  
  public boolean seesNote() {
   return this.noteCamera.getLatestResult().hasTargets();
  }

  public Command turnToAngle(double angle) {
        return turnToAngle(() -> angle);
  }
    private Command turnToAngle(DoubleSupplier angleSupplier) {

  return run(() -> {
        ChassisSpeeds speeds =
              this.swerveDrive.swerveController.getTargetSpeeds(
                0,
                0,
                Math.toRadians(angleSupplier.getAsDouble()),
                swerveDrive.getOdometryHeading().getRadians(),
                swerveDrive.getMaximumVelocity());
        swerveDrive.driveFieldRelative(speeds);
      })
      .until(this.swerveDrive.swerveController.thetaController::atSetpoint);
  }

  public Command turnToNote() {
    return this.turnToAngle(this::getNoteAngle);
  }
  
@Override
  public void periodic() {
  }
}       