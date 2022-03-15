package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;

public class AutoDriveCommand extends CommandBase {

  private TrapezoidProfile.Constraints translationConstraints = new Constraints(2, 2); // 3, 3
  private ProfiledPIDController controllerX = new ProfiledPIDController(2.5, 0, 0, translationConstraints);
  private ProfiledPIDController controllerY = new ProfiledPIDController(2.5, 0, 0, translationConstraints);
  private PIDController controllerT = new PIDController(3, 0, 0);

  private double speedX;
  private double speedY;
  private double speedT;

  private DrivetrainSubsystem drivetrain;

  private Timer timer = new Timer();

  public AutoDriveCommand(DrivetrainSubsystem drivetrain, Pose2d startingCoordinate, Pose2d targetCoordinate) {
      this.drivetrain = drivetrain;
      addRequirements(drivetrain);

      double targetX = targetCoordinate.getX() - startingCoordinate.getX();
      controllerX.setGoal(targetX); 
      controllerX.setTolerance(0.01);

      double targetY = targetCoordinate.getY() - startingCoordinate.getY();
      controllerY.setGoal(targetY); 
      controllerY.setTolerance(0.01);

      double targetT = targetCoordinate.getRotation().getRadians() - startingCoordinate.getRotation().getRadians();
      controllerT.setSetpoint(targetT);
      controllerT.setTolerance(Math.toRadians(2));
      controllerT.enableContinuousInput(0, 2 * Math.PI);
  }

  @Override
  public void initialize(){
    drivetrain.resetGyroscope();
    drivetrain.resetPose();
    timer.start();
  }

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();
    double currentX = currentPose.getX();
    double currentY = currentPose.getY();
    double currentT = currentPose.getRotation().getRadians();

    speedX = controllerX.calculate(currentX);
    speedY = controllerY.calculate(currentY);
    speedT = controllerT.calculate(currentT);

    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, speedT, drivetrain.getGyroscopeRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("drive finished");
    drivetrain.drive(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    boolean atGoal = false;
    boolean slowSpeed = false;

    if (controllerX.atGoal() && controllerY.atGoal() && controllerT.atSetpoint()) {
      atGoal = true;
    }
    if (timer.hasElapsed(1) && speedX < 0.01 && speedY < 0.01 && speedT < 0.01) {
      slowSpeed = true;
    }

    return atGoal || slowSpeed;
  }
}