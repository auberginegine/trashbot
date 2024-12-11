package frc.robot.subsystems.vision;

import java.io.File;
import java.io.IOException;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class Vision extends SubsystemBase {
  
  private static Vision instance = null;

  public static Vision getInstance() {
    if (instance == null)
      try {
        instance = new Vision();
      } 
        catch (IOException e) {
        e.printStackTrace();
      }

    return instance;
  }

  private final PhotonCamera noteCamera;
  
  private final PhotonPoseEstimator vision;
  private SwerveDrive swerveDrive;
  public Vision (Drivetrain swerveDrive) {
  this.swerveDrive =  swerveDrive; 
  this.noteCamera = new PhotonCamera(DrivetrainConstants.kNoteCameraName);
  }
  
  private Vision() throws IOException {
    this.swerveDrive = swerveDrive;
    this.noteCamera = new PhotonCamera(DrivetrainConstants.kNoteCameraName);
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.DrivetrainConstants.kMaxSpeedMetersPerSecond);
  }

  new HolonomicPathFollowerConfig(
            new PIDConstants(
                // 0,0,0
                DrivetrainConstants.kTranslationP,
                DrivetrainConstants.kTranslationI,
                DrivetrainConstants.kTranslationD),
            new PIDConstants(
                // 0,0,0
                DrivetrainConstants.kRotationP,
                DrivetrainConstants.kRotationI,
                DrivetrainConstants.kRotationD),
            DrivetrainConstants.kMaxSpeedMetersPerSecond,
            0.5
                * Math.hypot(
                    DrivetrainConstants.kTrackWidthMeters, DrivetrainConstants.kWheelBaseMeters),
            new ReplanningConfig()),
        MyAlliance::isRed,
        this);
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
    return turnToAngle(getNoteAngle());
  }

  public Command turnToNote() {
    return this.turnToAngle(getNoteAngle());
  }

  private Translation2d getRelativeNoteLocation() {
    PhotonPipelineResult latestResult = this.noteCamera.getLatestResult();

    if (!latestResult.hasTargets()) return new Translation2d();

    PhotonTrackedTarget bestTarget = latestResult.getBestTarget();

    double dz =
      Units.inchesToMeters(10) / Math.tan((-bestTarget.getPitch() + 24) * Math.PI / 180.0);
      double dx = dz * Math.tan(bestTarget.getYaw() * Math.PI / 180.0);

      return new Translation2d(dx, dz);
  }

  public Command driveToNote() {
    return run(() -> {
      double headingControllerOutput = this.headingController.calculate(getNoteAngle, 0.0);
      driveFromInput.(0.0, 1.5, headingControllerOutput, false);
    }).until(!seesNote()).withTimeout(2.5);
  //pls commit this
  }
  
@Override
  public void periodic() {
  }
}       