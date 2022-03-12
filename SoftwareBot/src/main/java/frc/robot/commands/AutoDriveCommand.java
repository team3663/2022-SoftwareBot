package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class AutoDriveCommand extends CommandBase {

  private TrapezoidProfile.Constraints translationConstraints = new Constraints(2, 2); // TODO tune
  private TrapezoidProfile.Constraints rotationConstraints = new Constraints(2, 2);

  private ProfiledPIDController controllerX = new ProfiledPIDController(5, 0, 0, translationConstraints); // TODO tune
  private ProfiledPIDController controllerY = new ProfiledPIDController(5, 0, 0, translationConstraints);
  private ProfiledPIDController controllerT = new ProfiledPIDController(0, 0, 0, rotationConstraints);

  private DrivetrainSubsystem drivetrain;

  public AutoDriveCommand(DrivetrainSubsystem drivetrain, double targetX, double targetY, double targetT) {
      this.drivetrain = drivetrain;
      addRequirements(drivetrain);

      controllerX.setGoal(targetX); 
      controllerX.setTolerance(0.01);

      controllerY.setGoal(targetY); 
      controllerY.setTolerance(0.01);

      controllerT.setGoal(Math.toRadians(targetT));
      controllerT.setTolerance(0.01);
      controllerT.enableContinuousInput(0, 2 * Math.PI);
  }

  @Override
  public void initialize(){
    drivetrain.resetGyroscope();
  }

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();
    double currentX = currentPose.getX();
    double currentY = currentPose.getY();
    double currentT = currentPose.getRotation().getRadians();

    double speedX = controllerX.calculate(currentX);
    double speedY = controllerY.calculate(currentY);
    double speedT = controllerT.calculate(currentT);

    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, speedT, drivetrain.getGyroscopeRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    return controllerX.atGoal() && controllerY.atGoal() && controllerT.atGoal();
  }
}