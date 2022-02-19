package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

import static frc.robot.Constants.*;

import java.util.List;

// declare subsystems, commands, and button mappings
public class RobotContainer {

  // Controllers
  private final XboxController driveController = new XboxController(DRIVE_CONTROLLER_PORT);
  @SuppressWarnings("unused")
  private final XboxController operatorController = new XboxController(OPERATOR_CONTROLLER_PORT);

  // Subsystems
  private DrivetrainSubsystem drivetrainSubsystem;

  // Commands
  private Command autoCommand;


  public RobotContainer() {

    createSubsystems();
    createCommands();
    configureButtonBindings();
  }

  private void createSubsystems() {

    // Create our drive base subsystem and attach the default drive command to it.
    drivetrainSubsystem = new DrivetrainSubsystem();
    drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        drivetrainSubsystem,
        () -> -modifyAxis(driveController.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(driveController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(driveController.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));    

  }

  private void createCommands() {

    TrajectoryConfig config = new TrajectoryConfig(.25, .01);
    config.setReversed(true);
    
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(90)),                        // Start at the origin facing the +X direction       
        List.of(new Translation2d(1, 1), new Translation2d(2, 2)), // Pass through these two interior waypoints, making an 's' curve path
        new Pose2d(3, 3, new Rotation2d(0)),                        // End 3 meters straight ahead of where we started, facing forward
        config);

    double totalTimeSeconds = trajectory.getTotalTimeSeconds();
    System.out.println("-----------------------------> totalTimeSeconds: " + totalTimeSeconds);

    PIDController xController = new PIDController(1.5, 0, 0);
    PIDController yController = new PIDController(1.5, 0, 0);


    double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
    double kMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
    double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;

    TrapezoidProfile.Constraints thetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond,
        kMaxAngularAccelerationRadiansPerSecondSquared);

    ProfiledPIDController thetaController = new ProfiledPIDController(3, 0, 0, thetaControllerConstraints);

    autoCommand = new SwerveControllerCommand(
      trajectory,
      drivetrainSubsystem::getPose,
      drivetrainSubsystem.getKinematics(),
      xController,
      yController,
      thetaController,
      drivetrainSubsystem::setModuleStates,
      drivetrainSubsystem);
  }

  private void configureButtonBindings() {
    new Button(driveController::getBackButton).whenPressed(drivetrainSubsystem::resetGyroscope);
    new Button(driveController::getStartButton).whenPressed(drivetrainSubsystem::resetPosition);
  }

  public Command getAutonomousCommand() {

    return autoCommand;
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
}
