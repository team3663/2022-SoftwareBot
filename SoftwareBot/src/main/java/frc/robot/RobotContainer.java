package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.AutoFollowCargoCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.Wait;
import frc.robot.helpers.Pixy;
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

  public SequentialCommandGroup getAutonomousCommand() {
    return new SequentialCommandGroup(
            new InstantCommand(() -> drivetrainSubsystem.setInitPose(new Pose2d(2.77, -0.65, new Rotation2d()))),
            new AutoDriveCommand(drivetrainSubsystem, 3.79, -0.65, 0),
            new Wait(drivetrainSubsystem, 3),
            new AutoDriveCommand(drivetrainSubsystem, 2.22, -3.15, -45),
            new Wait(drivetrainSubsystem, 3),
            new AutoDriveCommand(drivetrainSubsystem, 2.99, -7.12, 0),
            new Wait(drivetrainSubsystem, 2),
            new AutoDriveCommand(drivetrainSubsystem, 2.22, -3.15, 0));
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
