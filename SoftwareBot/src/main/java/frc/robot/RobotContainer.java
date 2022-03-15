package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.drivers.Pigeon;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.utils.ControllerUtils;
import frc.robot.utils.SwerveDriveConfig;
import frc.robot.utils.SwerveModuleConfig;

import frc.robot.Constants;

// declare subsystems, commands, and button mappings
public class RobotContainer {

  // private final Pixy pixy = new Pixy(Pixy.TEAM_RED);

  // OLD private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(); // pixy
  private final XboxController controller = new XboxController(0);

  private DrivetrainSubsystem drivetrain;

  private DriveCommand drive;


  Pigeon pigeon = new Pigeon(Constants.DRIVETRAIN_PIGEON_ID);

  public RobotContainer() {

    createSubsystems();

    drive = new DriveCommand(
        drivetrain,
        () -> -ControllerUtils.modifyAxis(controller.getLeftY()) * drivetrain.maxVelocity,
        () -> -ControllerUtils.modifyAxis(controller.getLeftX()) * drivetrain.maxVelocity,
        () -> -ControllerUtils.modifyAxis(controller.getRightX()) * drivetrain.maxAngularVelocity * 0.9);
    drivetrain.setDefaultCommand(drive);

    configureButtonBindings();
  }

  private void createSubsystems() {
    SwerveModuleConfig fl = new SwerveModuleConfig(Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR, Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
    Constants.FRONT_LEFT_MODULE_STEER_ENCODER, Constants.FRONT_LEFT_MODULE_STEER_OFFSET);
    SwerveModuleConfig fr = new SwerveModuleConfig(Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR, Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
    Constants.FRONT_RIGHT_MODULE_STEER_ENCODER, Constants.FRONT_RIGHT_MODULE_STEER_OFFSET);
    SwerveModuleConfig bl = new SwerveModuleConfig(Constants.BACK_LEFT_MODULE_DRIVE_MOTOR, Constants.BACK_LEFT_MODULE_STEER_MOTOR,
    Constants.BACK_LEFT_MODULE_STEER_ENCODER, Constants.BACK_LEFT_MODULE_STEER_OFFSET);
    SwerveModuleConfig br = new SwerveModuleConfig(Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR, Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
    Constants.BACK_RIGHT_MODULE_STEER_ENCODER, Constants.BACK_RIGHT_MODULE_STEER_OFFSET);
    SwerveDriveConfig swerveConfig = new SwerveDriveConfig(fl, fr, bl, br, Constants.DRIVETRAIN_TRACKWIDTH_METERS,
    Constants.DRIVETRAIN_WHEELBASE_METERS, Constants.DRIVE_TRAIN_WHEEL_DIAMETER_METERS);
    drivetrain = new DrivetrainSubsystem(swerveConfig, pigeon);
  }

  private void configureButtonBindings() {
    new Button(controller::getBackButton).whenPressed(drivetrain::resetPosition);
  }

  public Command getAutonomousCommand() {

    Pose2d start = new Pose2d(2.77, -0.65, new Rotation2d());
    Pose2d ball2 = new Pose2d(3.92, -0.65, new Rotation2d());
    Pose2d ball3 = new Pose2d(2.15, -3.26, Rotation2d.fromDegrees(302.13));
    Pose2d aim3 = new Pose2d(2.15, -3.26, Rotation2d.fromDegrees(270));
    Pose2d station = new Pose2d(3.02, -7.24, new Rotation2d(257.72));

    /*
     * return new SequentialCommandGroup(
     * new AutoDriveCommand(drivetrainSubsystem, start, ball2),
     * new AutoDriveCommand(drivetrainSubsystem, ball2, ball3),
     * new AutoDriveCommand(drivetrainSubsystem, ball3, aim3),
     * new AutoDriveCommand(drivetrainSubsystem, aim3, station),
     * new AutoDriveCommand(drivetrainSubsystem, station, aim3)
     * );
     */

    return new SequentialCommandGroup(
        new InstantCommand(() -> drivetrain.setAutoInitCommand()),
        new AutoDriveCommand(drivetrain, 3.92, -0.65, 0),
        new AutoDriveCommand(drivetrain, 2.15, -3.26, 302.13),
        new AutoDriveCommand(drivetrain, 2.15, -3.26, 270),
        new AutoDriveCommand(drivetrain, 3.02, -7.24, 257.72),
        new AutoDriveCommand(drivetrain, 2.15, -3.26, 270));
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.1) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    value = deadband(value, 0.1);
    value = Math.copySign(value * value, value);

    return value;
  }

  /*
   * public Command createTrajectoryCommand() {
   * TrajectoryConfig config = new TrajectoryConfig(1.5, 1);
   * config.setReversed(true);
   * 
   * Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
   * new Pose2d(0, 0, new Rotation2d(0)), // Start at the origin facing the +X
   * direction
   * List.of(new Translation2d(2, 0),new Translation2d(2, 2)), // Pass through
   * these two interior waypoints, making an 's' curve path
   * new Pose2d(4, 2, new Rotation2d(0)), // End 3 meters straight ahead of where
   * we started, facing forward
   * config);
   * 
   * double totalTimeSeconds = trajectory.getTotalTimeSeconds();
   * System.out.println("-----------------------------> totalTimeSeconds: " +
   * totalTimeSeconds);
   * 
   * double kpX = 0.0000000001; //.01 for both X and Y
   * double kpY = 0.0000000001;
   * double kiX = 0.0;
   * double kiY = 0.0;
   * double kdX = 0.0;
   * double kdY = 0.0;
   * PIDController xController = new PIDController(kpX, kiX, kdX);
   * PIDController yController = new PIDController(kpY, kiY, kdY);
   * 
   * 
   * double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
   * double kMaxAngularSpeedRadiansPerSecond =
   * kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
   * double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
   * 
   * TrapezoidProfile.Constraints thetaControllerConstraints = new
   * TrapezoidProfile.Constraints(
   * kMaxAngularSpeedRadiansPerSecond,
   * kMaxAngularAccelerationRadiansPerSecondSquared);
   * 
   * double kpTheta = 0.04; // .04
   * double kiTheta = 0.0;
   * double kdTheta = 0.0;
   * ProfiledPIDController thetaController = new ProfiledPIDController(kpTheta,
   * kiTheta, kdTheta, thetaControllerConstraints);
   * 
   * Command autoCommand = new SwerveControllerCommand(
   * trajectory,
   * drivetrainSubsystem::getPose,
   * drivetrainSubsystem.getKinematics(),
   * xController,
   * yController,
   * thetaController,
   * drivetrainSubsystem::setModules,
   * drivetrainSubsystem);
   * 
   * return autoCommand;
   * }
   */
}
