package frc.robot.subsystems.drivetrain;

import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.util.MyAlliance;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Drivetrain extends SubsystemBase {
    
    private static Drivetrain instance = null;

    public static Drivetrain getInstance() {
        if (instance == null) {
            try {
                instance = new Drivetrain();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        return instance;
    }

  
    private SwerveDrive swerveDrive;

    private Drivetrain() throws IOException {

        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        swerveDrive =
            new SwerveParser(swerveJsonDirectory)
                .createSwerveDrive(Constants.DrivetrainConstants.kMaxSpeedMetersPerSecond);


        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH; 

    this.configurePathPlanner();
    this.configureSwerve();

  }

  private void configureSwerve() {
    swerveDrive.swerveController.setMaximumAngularVelocity(Constants.DrivetrainConstants.kMaxOmegaRadiansPerSecond);
    swerveDrive.setHeadingCorrection(
        false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(
        false); // !SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for
    // simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(
        true, false,
        0.1); // Correct for skew that gets worse as angular velocity increases. Start with a
    // coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(
        false, 1); // Enable if you want to resynchronize your absolute encoders and motor encoders
    // periodically when they are not moving.
    // swerveDrive.setChassisDiscretization(true, false, Constants.DrivetrainConstants.kSecondOrderKinematicsDt);
    swerveDrive
        .pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder
    // and push the offsets onto it. Throws warning if not possible
  }

   private void configurePathPlanner() {
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetPose,
        this::getChassisSpeeds,
        this::driveRobotRelative,
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
                    DrivetrainConstants.kTrackWidthMeters, 
                    DrivetrainConstants.kWheelBaseMeters
                ),
            new ReplanningConfig()),
        MyAlliance::isRed,
        this);
  }

  public void zeroHeading() {
    this.swerveDrive.zeroGyro(); 
  }

    public Pose2d getPose() {
        return this.swerveDrive.getPose();
    }

    public void resetPose(Pose2d pose) {
        this.swerveDrive.resetOdometry(pose);
    }

    private SwerveModuleState[] getModuleStates() {
        return this.swerveDrive.getStates();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return this.swerveDrive.getRobotVelocity();
    }

    public SwerveModulePosition[] getModulePositions() {
        return this.swerveDrive.getModulePositions();
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.drive(
            speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
    }

    public void driveFieldRelative(ChassisSpeeds speeds) {
        this.drive(
            speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, true);
    }

    public void drive(double forward, double strafe, double rotate, boolean fieldRelative) {
        this.swerveDrive.drive(new Translation2d(forward, strafe), rotate, fieldRelative, false);
    }

    public Command driveCommand(XboxController controller) {
        return run(() -> {
          double multiplier = controller.getRightTriggerAxis() > 0.5 ? 0.4 : 1.0;

          double omega =
              DrivetrainConstants.kMaxTeleopRotationPercent
                  * -Math.pow(controller.getRightX(), 3) * swerveDrive.getMaximumAngularVelocity()
                  * multiplier;

          Translation2d strafeVec =
              SwerveMath.scaleTranslation(
                  new Translation2d(
                      -MathUtil.applyDeadband(controller.getLeftY(), 0.03) * swerveDrive.getMaximumVelocity() * DrivetrainConstants.kMaxTeleopSpeedPercent,
                      -MathUtil.applyDeadband(controller.getLeftX(), 0.03) * swerveDrive.getMaximumVelocity() * DrivetrainConstants.kMaxTeleopSpeedPercent),
                  0.8);

          if (MyAlliance.isRed()) strafeVec = strafeVec.rotateBy(Rotation2d.fromDegrees(180));

          this.swerveDrive.drive(
              strafeVec,
              omega,
              true,
              false);
        })
        .finallyDo(this::stop);
    }

    public void stop() {
        driveRobotRelative(new ChassisSpeeds());
    }


}
