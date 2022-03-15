package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;

public class AutoDriveCommand extends CommandBase {

  private TrapezoidProfile.Constraints translationConstraints = new Constraints(1, 1); // 3, 3
  private TrapezoidProfile.Constraints rotationConstraints = new Constraints(1, 1); // 3, 3
  
  private ProfiledPIDController controllerX = new ProfiledPIDController(5, 0, 0, translationConstraints); //1
  private ProfiledPIDController controllerY = new ProfiledPIDController(5, 0, 0, translationConstraints); //1
  private ProfiledPIDController controllerT = new ProfiledPIDController(8, 0, 0, rotationConstraints); // 

  private double speedX;
  private double speedY;
  private double speedT;

  private DrivetrainSubsystem drivetrain;

  private boolean zeroHub = false;

  private Timer timer = new Timer();

  public AutoDriveCommand(DrivetrainSubsystem drivetrain, Pose2d startingCoordinate, Pose2d targetCoordinate) {
      this.drivetrain = drivetrain;
      addRequirements(drivetrain);

      double targetX = targetCoordinate.getX() - startingCoordinate.getX();
      controllerX.setGoal(targetX); 
      controllerX.setTolerance(0.1);

      double targetY = targetCoordinate.getY() - startingCoordinate.getY();
      controllerY.setGoal(targetY); 
      controllerY.setTolerance(0.1);

      double targetT = targetCoordinate.getRotation().getRadians() - startingCoordinate.getRotation().getRadians();
      controllerT.setGoal(targetT);
      controllerT.setTolerance(Math.toRadians(2));
      controllerT.enableContinuousInput(0, 2 * Math.PI);
  }

  public AutoDriveCommand(DrivetrainSubsystem drivetrain, double targetX, double targetY, double targetT) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    controllerX.setGoal(targetX); 
    controllerX.setTolerance(0.1);

    controllerY.setGoal(targetY); 
    controllerY.setTolerance(0.1);

    controllerT.setGoal(targetT);
    controllerT.setTolerance(Math.toRadians(5));
    controllerT.enableContinuousInput(0, 2 * Math.PI);

    zeroHub = true;
}
  

  @Override
  public void initialize(){

    if (!zeroHub) {
      drivetrain.resetGyroscope();
      drivetrain.resetPose();
    }
    timer.start();

    Pose2d startingPose = drivetrain.getPose();
    controllerX.reset(startingPose.getX());
    controllerY.reset(startingPose.getY());
    controllerT.reset(startingPose.getRotation().getRadians());
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

    System.out.println(speedX + ", " + speedY + "," + speedT);

    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, speedT, currentPose.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("drive finished");
    drivetrain.drive(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    return (controllerX.atGoal() && controllerY.atGoal() && controllerT.atGoal());
      // || (timer.hasElapsed(1) && speedX <= 0.1 && speedY <= 0.1 && speedT <= 0.1);
  }
}