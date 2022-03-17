package frc.robot;

import java.util.List;

import org.frcteam2910.common.control.SimplePathBuilder;
import org.frcteam2910.common.control.SplinePathBuilder;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.control.TrajectoryConstraint;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.control.CentripetalAccelerationConstraint;
import org.frcteam2910.common.control.MaxAccelerationConstraint;
import org.frcteam2910.common.control.MaxVelocityConstraint;
import org.frcteam2910.common.control.Path;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.FollowerCommand;
import frc.robot.drivers.Pigeon;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.utils.ControllerUtils;
import frc.robot.utils.SwerveDriveConfig;
import frc.robot.utils.SwerveModuleConfig;
import frc.robot.utils.Terjectory.AutoDrive;

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

    /*
    Pose2d start = new Pose2d(2.77, -0.65, new Rotation2d());
    Pose2d ball2 = new Pose2d(3.92, -0.65, new Rotation2d());
    Pose2d ball3 = new Pose2d(2.15, -3.26, Rotation2d.fromDegrees(302.13));
    Pose2d aim3 = new Pose2d(2.15, -3.26, Rotation2d.fromDegrees(270));
    Pose2d station = new Pose2d(3.02, -7.24, new Rotation2d(257.72));


    Path path = new SimplePathBuilder(Vector2.ZERO, Rotation2.ZERO).lineTo(new Vector2(5, 0)).build();

    path = new SplinePathBuilder(Vector2.ZERO, Rotation2.ZERO, Rotation2.ZERO)
      .hermite(new Vector2(5, 3), Rotation2.ZERO)
      .build();
    Path secPath = new SplinePathBuilder(new Vector2(5, 3), new Rotation2(-5,-3, true), Rotation2.ZERO)
      .hermite(new Vector2(0, 0), new Rotation2(-5,-3, true), Rotation2.fromDegrees(180)).build();

    Path testPath = new SimplePathBuilder(Vector2.ZERO, Rotation2.ZERO).lineTo(new Vector2(3, 0),Rotation2.fromDegrees(180)).build();

    TrajectoryConstraint[] constraints = {
      new MaxAccelerationConstraint(3),
      new MaxVelocityConstraint(7),
      new CentripetalAccelerationConstraint(5.0)
    };

    Trajectory trajectory = new Trajectory(path, constraints, Units.inchesToMeters(0.1));
    Trajectory secondTrajectory = new Trajectory(secPath, constraints, Units.inchesToMeters(0.1));
    Trajectory testingRoation = new Trajectory(testPath, constraints, Units.inchesToMeters(0.1));

    return new SequentialCommandGroup(new InstantCommand(() -> drivetrain.resetPosition()), new FollowerCommand(drivetrain, trajectory), new FollowerCommand(drivetrain, secondTrajectory));

    */

    Path ball2 = new SplinePathBuilder(new Vector2(0, 0), Rotation2.ZERO, Rotation2.ZERO)
      .hermite(new Vector2(-1.184, 0.475), Rotation2.ZERO, Rotation2.ZERO)
      .build();
    Path ball3 = new SplinePathBuilder(new Vector2(-1.184, 0.475), Rotation2.fromDegrees(-52), Rotation2.ZERO) // heading
      .hermite(new Vector2(-2.427, 1.469), Rotation2.fromDegrees(-123.22), Rotation2.ZERO) // heading, rotation
      .hermite(new Vector2(-0.433, 2.772), Rotation2.fromDegrees(112.5), Rotation2.ZERO) // heading, rotation
      .build();

    Path aim3 = new SplinePathBuilder(new Vector2(0, 0), Rotation2.ZERO, Rotation2.ZERO)
      .hermite(new Vector2(-1.184, 0.475), Rotation2.ZERO, Rotation2.ZERO) // rotation
      .build();

      TrajectoryConstraint[] constraints = {
        new MaxAccelerationConstraint(2),
        new MaxVelocityConstraint(1),
        new CentripetalAccelerationConstraint(5.0)
      };
  
      Trajectory t1 = new Trajectory(ball2, constraints, Units.inchesToMeters(0.1));
      Trajectory t2 = new Trajectory(ball3, constraints, Units.inchesToMeters(0.1));
  

      return new SequentialCommandGroup(
        new InstantCommand(() -> drivetrain.resetPosition()),
        new FollowerCommand(drivetrain, t1), new FollowerCommand(drivetrain, t2));

    /*
     * return new SequentialCommandGroup(
     * new AutoDriveCommand(drivetrainSubsystem, start, ball2),
     * new AutoDriveCommand(drivetrainSubsystem, ball2, ball3),
     * new AutoDriveCommand(drivetrainSubsystem, ball3, aim3),
     * new AutoDriveCommand(drivetrainSubsystem, aim3, station),
     * new AutoDriveCommand(drivetrainSubsystem, station, aim3)
     * );
     */

    // AutoDrive auto = new AutoDrive(drivetrain, List.of(), List.of());
    // auto.addPath(new Pose2d(), new Pose2d(3,3,new Rotation2d()), List.of(new Translation2d(1,1)));
    // auto.addPath(new Pose2d(-0,-0,new Rotation2d()), new Pose2d(-3,-3, new Rotation2d()), List.of(new Translation2d(-2,-2)));
    // return auto.getPath().Run();


    

    // return new SequentialCommandGroup(
    //     new InstantCommand(() -> drivetrain.setAutoInitCommand()),
    //     new AutoDriveCommand(drivetrain, 3.92, -0.65, 0),
    //     new AutoDriveCommand(drivetrain, 2.15, -3.26, 302.13),
    //     new AutoDriveCommand(drivetrain, 2.15, -3.26, 270),
    //     new AutoDriveCommand(drivetrain, 3.02, -7.24, 257.72),
    //     new AutoDriveCommand(drivetrain, 2.15, -3.26, 270));
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
