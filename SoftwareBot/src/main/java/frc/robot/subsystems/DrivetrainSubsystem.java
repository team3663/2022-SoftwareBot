package frc.robot.subsystems;

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
        public static final double WHEEL_DIAMETER_METERS = 0.106325;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
                        SdsModuleConfigurations.MK4_L4.getDriveReduction() * WHEEL_DIAMETER_METERS * Math.PI;

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

        private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Front right
                        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back left
                        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back right
                        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

        private final Pigeon pigeon = new Pigeon(DRIVETRAIN_PIGEON_ID);
        private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, pigeon.getRotation2d(),
                        new Pose2d());

        // counter-clockwise rotation increases angle

        private final SwerveModule frontLeftModule;
        private final SwerveModule frontRightModule;
        private final SwerveModule backLeftModule;
        private final SwerveModule backRightModule;
        private Pose2d robotPosition;

        private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        NetworkTableEntry poseXEntry;
        NetworkTableEntry poseYEntry;
        NetworkTableEntry poseAngleEntry;
        NetworkTableEntry driveSignalXEntry;
        NetworkTableEntry driveSignalYEntry;
        NetworkTableEntry driveSignalRotationEntry;
        NetworkTableEntry pigeonCompassHeadingEntry;
        NetworkTableEntry odometryCompassHeadingEntry;
        
        NetworkTableEntry xEntry;
        NetworkTableEntry yEntry;
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
                                FRONT_LEFT_MODULE_STEER_OFFSET);

                frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                                drivetrainModuletab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(2, 0),
                                Mk4SwerveModuleHelper.GearRatio.L4,
                                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                                FRONT_RIGHT_MODULE_STEER_MOTOR,
                                FRONT_RIGHT_MODULE_STEER_ENCODER,
                                FRONT_RIGHT_MODULE_STEER_OFFSET);

                backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                                drivetrainModuletab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(4, 0),
                                Mk4SwerveModuleHelper.GearRatio.L4,
                                BACK_LEFT_MODULE_DRIVE_MOTOR,
                                BACK_LEFT_MODULE_STEER_MOTOR,
                                BACK_LEFT_MODULE_STEER_ENCODER,
                                BACK_LEFT_MODULE_STEER_OFFSET);

                backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                                drivetrainModuletab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(6, 0),
                                Mk4SwerveModuleHelper.GearRatio.L4,
                                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                                BACK_RIGHT_MODULE_STEER_MOTOR,
                                BACK_RIGHT_MODULE_STEER_ENCODER,
                                BACK_RIGHT_MODULE_STEER_OFFSET);

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
                pigeonCompassHeadingEntry = drivetrainRobotTab.add("Pigeon heading", 0.0)
                                .withPosition(2, 0)
                                .withSize(1,1)
                                .getEntry();
                odometryCompassHeadingEntry = drivetrainRobotTab.add("Odometry heading", 0.0)
                                .withPosition(2,1)
                                .withSize(1, 1)
                                .getEntry();
                xEntry = drivetrainRobotTab.add("X Entry", 0.0)
                                .withPosition(3, 0)
                                .withSize(1, 1)
                                .getEntry();
                yEntry = drivetrainRobotTab.add("Y Entry", 0.0)
                                .withPosition(3, 1)
                                .withSize(1, 1)
                                .getEntry();
                ShuffleboardLayout driveSignalContainer = drivetrainRobotTab
                                .getLayout("Drive Signal", BuiltInLayouts.kGrid)
                                .withPosition(0, 3)
                                .withSize(3, 1);
                driveSignalYEntry = driveSignalContainer.add("Drive Signal Strafe", 0.0).getEntry();
                driveSignalXEntry = driveSignalContainer.add("Drive Signal Forward", 0.0).getEntry();
                driveSignalRotationEntry = driveSignalContainer.add("Drive Signal Rotation", 0.0).getEntry();
        }
        public void calibratePigeon(){
                pigeon.calibrate();
        }

        public void resetPosition() {
                
                odometry.resetPosition(new Pose2d(), new Rotation2d(0));
        }

        public Pose2d getPose() {
                return robotPosition;
        }

        public SwerveDriveKinematics getKinematics() {
                return kinematics;
        }

        public void resetGyroscope() {
                pigeon.reset();
        }

        public Rotation2d getGyroscopeRotation() {
                return Rotation2d.fromDegrees(pigeon.getAngle());
        }

        public void drive(ChassisSpeeds chassisSpeeds) {
                this.chassisSpeeds = chassisSpeeds;
        }

        public void setModuleStates(SwerveModuleState[] states) {

                frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[0].angle.getRadians());
                frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[1].angle.getRadians());
                backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[2].angle.getRadians());
                backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[3].angle.getRadians());


               /*  updates odometry over time via timeStamp */
               //odometry.updateWithTime(Timer.getFPGATimestamp(), getGyroscopeRotation(), states);
               


               //THIS CHANGED ---^ ORIGINAL----^
               robotPosition = odometry.update(pigeon.getRotation2d(), states);
               System.out.println("updated ");


        }

        @Override
        public void periodic() {

                SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
                setModuleStates(states);

                driveSignalYEntry.setDouble(chassisSpeeds.vyMetersPerSecond);
                driveSignalXEntry.setDouble(chassisSpeeds.vxMetersPerSecond);
                driveSignalRotationEntry.setDouble(chassisSpeeds.omegaRadiansPerSecond);

                poseXEntry.setDouble(getPose().getTranslation().getX());
                poseYEntry.setDouble(getPose().getTranslation().getY());
                poseAngleEntry.setDouble(getPose().getRotation().getDegrees());

                xEntry.setDouble(getPose().getX());
                yEntry.setDouble(getPose().getY());

                pigeonCompassHeadingEntry.setDouble(pigeon.getAngle());
                odometryCompassHeadingEntry.setDouble(odometry.getPoseMeters().getRotation().getDegrees());
        }
}
