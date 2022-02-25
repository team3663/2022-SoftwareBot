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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
        // () -> -cubeRoot(driveController.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        // () -> -cubeRoot(driveController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        // () -> -cubeRoot(driveController.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));    

        () -> -deadband(driveController.getLeftY(), 0.05) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -deadband(driveController.getLeftX(), 0.05) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -deadband(driveController.getRightX(), 0.05) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));    
  }

  private void createCommands() {


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
    new Button(driveController::getStartButton).whenPressed(drivetrainSubsystem::resetGyroscope);
  }

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(new InstantCommand(() -> drivetrainSubsystem.resetPosition()), new InstantCommand(() -> drivetrainSubsystem.resetGyroscope()), autoCommand);

  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) < deadband) {
      value = 0.0;
    }
    return value;
  }

  private static double cubeRoot(double value, double deadband) {
    value = deadband(value, deadband);
    value = Math.copySign(Math.cbrt(value), value);
    return value;
  }

  private static double squared(double value, double deadband) {
    value = deadband(value, deadband);
    value = Math.copySign(value * value, value);
    return value;
  }
}
