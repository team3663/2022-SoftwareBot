package frc.robot.subsystems;

import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.Pigeon;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {

  public static final double MAX_VOLTAGE = 12.0;
  public static final double WHEEL_DIAMETER_METERS = 0.1016;

  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK4_L4.getDriveReduction() * WHEEL_DIAMETER_METERS * Math.PI;

  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  public static final double METERS_TO_ROBOT_METERS_RATIO = 1;

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), new Pose2d());
  private double accumulatedChassisAngle = 0;
  private double time = 0;

  private final Pigeon pigeon = new Pigeon(DRIVETRAIN_PIGEON_ID);
        // counter-clockwise rotation increases angle

  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  NetworkTableEntry poseXEntry;
  NetworkTableEntry poseYEntry;
  NetworkTableEntry poseAngleEntry;
  NetworkTableEntry driveSignalXEntry;
  NetworkTableEntry driveSignalYEntry;
  NetworkTableEntry driveSignalRotationEntry;

  public DrivetrainSubsystem() {
    ShuffleboardTab drivetrainModuletab = Shuffleboard.getTab("drivetrain_modules");

    frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            drivetrainModuletab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            Mk4SwerveModuleHelper.GearRatio.L4,
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            FRONT_LEFT_MODULE_STEER_MOTOR,
            FRONT_LEFT_MODULE_STEER_ENCODER,
            FRONT_LEFT_MODULE_STEER_OFFSET
    );

    frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
            drivetrainModuletab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk4SwerveModuleHelper.GearRatio.L4,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            drivetrainModuletab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk4SwerveModuleHelper.GearRatio.L4,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    backRightModule = Mk4SwerveModuleHelper.createFalcon500(
            drivetrainModuletab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk4SwerveModuleHelper.GearRatio.L4,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );

ShuffleboardTab drivetrainRobotTab = Shuffleboard.getTab("drivetrain_robot");
        poseXEntry = drivetrainRobotTab.add("Pose X", 0.0)
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();
        poseYEntry = drivetrainRobotTab.add("Pose Y", 0.0)
                .withPosition(0, 1)
                .withSize(1, 1)
                .getEntry();
        poseAngleEntry = drivetrainRobotTab.add("Pose Angle", 0.0)
                .withPosition(0, 2)
                .withSize(1, 1)
                .getEntry();
        ShuffleboardLayout driveSignalContainer = drivetrainRobotTab.getLayout("Drive Signal", BuiltInLayouts.kGrid)
                        .withPosition(0, 3)
                        .withSize(3, 1);
                driveSignalYEntry = driveSignalContainer.add("Drive Signal Strafe", 0.0).getEntry();
                driveSignalXEntry = driveSignalContainer.add("Drive Signal Forward", 0.0).getEntry();
                driveSignalRotationEntry = driveSignalContainer.add("Drive Signal Rotation", 0.0).getEntry();

  }

  public void resetPosition() {
          odometry.resetPosition(new Pose2d(), new Rotation2d());
  }

  public Pose2d getPose() {
          return odometry.getPoseMeters();
  }

  public void resetGyroscope() {
          pigeon.reset();
  }

  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(pigeon.getAngle());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
          /*
          double vx_robotmeters = chassisSpeeds.vxMetersPerSecond / METERS_TO_ROBOT_METERS_RATIO;
          double vy_robotmeters = chassisSpeeds.vyMetersPerSecond / METERS_TO_ROBOT_METERS_RATIO;
          double vo = chassisSpeeds.omegaRadiansPerSecond;
          this.chassisSpeeds = new ChassisSpeeds(vx_robotmeters, vy_robotmeters, vo);*/
          this.chassisSpeeds = chassisSpeeds;
  }

@Override
  public void periodic() {

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    
    odometry.updateWithTime(Timer.getFPGATimestamp(), getGyroscopeRotation(), states);

    double currentTime = Timer.getFPGATimestamp();
    double lastTime = time;
    double dt = currentTime - lastTime;
    accumulatedChassisAngle = Math.toDegrees(accumulatedChassisAngle + chassisSpeeds.omegaRadiansPerSecond * dt);

    if (accumulatedChassisAngle - getGyroscopeRotation().getDegrees() > 0) {
    }

    driveSignalYEntry.setDouble(chassisSpeeds.vyMetersPerSecond * METERS_TO_ROBOT_METERS_RATIO);
    driveSignalXEntry.setDouble(chassisSpeeds.vxMetersPerSecond * METERS_TO_ROBOT_METERS_RATIO);
    driveSignalRotationEntry.setDouble(chassisSpeeds.omegaRadiansPerSecond);

    poseXEntry.setDouble(getPose().getTranslation().getX());
    poseYEntry.setDouble(getPose().getTranslation().getY());
    poseAngleEntry.setDouble(getPose().getRotation().getDegrees());
  }
}
