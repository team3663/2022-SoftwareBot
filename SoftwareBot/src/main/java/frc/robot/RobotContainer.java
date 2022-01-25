package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.AutoDriveBackward;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

// declare subsystems, commands, and button mappings
public class RobotContainer {

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final XboxController controller = new XboxController(0);
  private double snapTargetAngle = 361;

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
    new Button(controller::getStartButton).whenPressed(drivetrainSubsystem::resetPosition);
    new Button(getDPad()).whenPressed(() -> drivetrainSubsystem.setSnapTargetAngle(snapTargetAngle));
  }

  public Command getAutonomousCommand() {
    return new AutoDriveBackward(drivetrainSubsystem, -2);
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

  private BooleanSupplier getDPad() {
    snapTargetAngle = 360 - controller.getPOV();
    BooleanSupplier isSnap = () -> (snapTargetAngle != 361);
    return isSnap;
  }
}
