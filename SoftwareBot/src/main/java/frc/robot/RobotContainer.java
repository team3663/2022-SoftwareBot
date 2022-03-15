package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

// declare subsystems, commands, and button mappings
public class RobotContainer {
  
  // private final Pixy pixy = new Pixy(Pixy.TEAM_RED);

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(); // pixy
  private final XboxController controller = new XboxController(0);

  public RobotContainer() {
    drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            drivetrainSubsystem,
            () -> -modifyAxis(controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new Button(controller::getBackButton).whenPressed(drivetrainSubsystem::resetGyroscope);
    new Button(controller::getStartButton).whenPressed(drivetrainSubsystem::resetPose);
  }

  public Command getAutonomousCommand() {

    Pose2d start = new Pose2d(2.77, -0.65, new Rotation2d());
    Pose2d ball2 = new Pose2d(3.92, -0.65, new Rotation2d());
    Pose2d ball3 = new Pose2d(2.15, -3.26, Rotation2d.fromDegrees(302.13));
    Pose2d aim3 = new Pose2d(2.15, -3.26, Rotation2d.fromDegrees(270));
    Pose2d station = new Pose2d(3.02, -7.24, new Rotation2d(257.72));

    return new AutoDriveCommand(drivetrainSubsystem, start, ball2);
    
    /*
    return new SequentialCommandGroup(
      new AutoDriveCommand(drivetrainSubsystem, start, ball2),
      new AutoDriveCommand(drivetrainSubsystem, ball2, ball3),
      new AutoDriveCommand(drivetrainSubsystem, ball3, aim3),
      new AutoDriveCommand(drivetrainSubsystem, aim3, station),
      new AutoDriveCommand(drivetrainSubsystem, station, aim3)
      );
    */
      
    // drive through balls
    /*
    return new SequentialCommandGroup(
            new InstantCommand(() -> drivetrainSubsystem.setInitPose(new Pose2d(2.77, -0.65, new Rotation2d()))),
            new AutoDriveCommand(drivetrainSubsystem, 3.92, -0.65, 0),
            new Wait(drivetrainSubsystem, 3),
            new AutoDriveCommand(drivetrainSubsystem, 2.15, -3.26, 302.13),
            new AutoDriveCommand(drivetrainSubsystem, 0, 0, 270),
            new Wait(drivetrainSubsystem, 3),
            new AutoDriveCommand(drivetrainSubsystem, 3.02, -7.24, 257.72),
            new Wait(drivetrainSubsystem, 1),
            new AutoDriveCommand(drivetrainSubsystem, 3.02, -3.02, 225));
            */

    /* 
    return new SequentialCommandGroup(
            new InstantCommand(() -> drivetrainSubsystem.setInitPose(new Pose2d(2.77, -0.65, new Rotation2d()))),
            new AutoDriveCommand(drivetrainSubsystem, 3.79, -0.65, 0),
            new Wait(drivetrainSubsystem, 3),
            new AutoDriveCommand(drivetrainSubsystem, 2.22, -3.15, -45),
            new Wait(drivetrainSubsystem, 3),
            new AutoDriveCommand(drivetrainSubsystem, 2.99, -7.12, 0),
            new Wait(drivetrainSubsystem, 2),
            new AutoDriveCommand(drivetrainSubsystem, 2.22, -3.15, 0));
    */
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
  public Command createTrajectoryCommand() {
    TrajectoryConfig config = new TrajectoryConfig(1.5, 1);
    config.setReversed(true);
    
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),                        // Start at the origin facing the +X direction       
        List.of(new Translation2d(2, 0),new Translation2d(2, 2)), // Pass through these two interior waypoints, making an 's' curve path
        new Pose2d(4, 2, new Rotation2d(0)),                        // End 3 meters straight ahead of where we started, facing forward
        config);

    double totalTimeSeconds = trajectory.getTotalTimeSeconds();
    System.out.println("-----------------------------> totalTimeSeconds: " + totalTimeSeconds);

    double kpX = 0.0000000001;  //.01 for both X and Y
    double kpY = 0.0000000001;
    double kiX = 0.0;
    double kiY = 0.0;
    double kdX = 0.0;
    double kdY = 0.0;
    PIDController xController = new PIDController(kpX, kiX, kdX); 
    PIDController yController = new PIDController(kpY, kiY, kdY); 


    double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
    double kMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
    double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;

    TrapezoidProfile.Constraints thetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond,
        kMaxAngularAccelerationRadiansPerSecondSquared);

    double kpTheta = 0.04;   // .04
    double kiTheta = 0.0;
    double kdTheta = 0.0;
    ProfiledPIDController thetaController = new ProfiledPIDController(kpTheta, kiTheta, kdTheta, thetaControllerConstraints); 

    Command autoCommand = new SwerveControllerCommand(
      trajectory,
      drivetrainSubsystem::getPose,
      drivetrainSubsystem.getKinematics(),
      xController,
      yController,
      thetaController,
      drivetrainSubsystem::setModules,
      drivetrainSubsystem);

      return autoCommand;
  }
  */
}
