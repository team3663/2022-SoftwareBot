package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class AutoDriveCommand extends CommandBase {

  private PIDController translationXController = new PIDController(1, 0, 0); 
  private PIDController translationYController = new PIDController(1, 0, 0);
  private PIDController rotationController = new PIDController(1, 0, 0);

  private double currentAngle;
  private double currentX;
  private double currentY;

  private double targetAngle;
  private double targetX;
  private double targetY;

  private double translationXSpeed;
  private double translationYSpeed;
  private double rotationSpeed;

  private DrivetrainSubsystem drivetrainSubsystem;

  public AutoDriveCommand(DrivetrainSubsystem drivetrainSubsystem, Pose2d target) {
      this.drivetrainSubsystem = drivetrainSubsystem;
      addRequirements(drivetrainSubsystem);

      targetAngle = target.getRotation().getRadians();
      targetX = target.getX();
      targetY = target.getY();
      
      translationXController.setSetpoint(targetX); 
      translationYController.setSetpoint(targetY);

      rotationController.setSetpoint(targetAngle);
      rotationController.enableContinuousInput(0, 2 * Math.PI);
  }

  @Override
  public void initialize(){
    drivetrainSubsystem.resetPose();
    drivetrainSubsystem.resetGyroscope();
  }

  @Override
  public void execute() {
    Pose2d currentPose = drivetrainSubsystem.getPose();
    currentX = currentPose.getX();
    currentY = currentPose.getY();
    currentAngle = currentPose.getRotation().getRadians();

    translationXSpeed = translationXController.calculate(currentX);
    translationYSpeed = translationYController.calculate(currentY);
    rotationSpeed = rotationController.calculate(currentAngle);

    System.out.println(translationXSpeed + ", " + translationYSpeed + ", " + rotationSpeed);

    drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(translationXSpeed, translationYSpeed, rotationSpeed,
                                                                    drivetrainSubsystem.getGyroscopeRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
    System.out.println("auto drive forward ends");
  }

  @Override
  public boolean isFinished() {
    boolean atTarget = false;
    if ((Math.abs(targetAngle - currentAngle) < Math.toRadians(2)) && (Math.abs(targetX - currentX) < 0.02) && (Math.abs(targetY - currentY) < 0.02)) {
      atTarget = true;
    }

    boolean slowSpeed = false;
    if (Math.abs(translationXSpeed) < 0.05 && Math.abs(translationYSpeed) < 0.05 && Math.abs(rotationSpeed) < 0.5) {
      slowSpeed = true;
    }
    
    return (atTarget || slowSpeed);
  }
}