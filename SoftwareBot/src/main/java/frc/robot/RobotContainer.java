package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

// declare subsystems, commands, and button mappings
public class RobotContainer {

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
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
    new Button(controller::getBackButton).whenPressed(drivetrainSubsystem::zeroGyroscope);
  }

  // FIXME add autonomous
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    value = deadband(value, 0.0);
    value = Math.copySign(value * value, value);

    return value;
  }
}
