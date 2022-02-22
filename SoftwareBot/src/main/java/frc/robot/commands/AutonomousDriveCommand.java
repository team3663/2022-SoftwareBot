package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class AutonomousDriveCommand extends CommandBase {

  private PIDController translationXController = new PIDController(0.6, 0, 0); //10
  private PIDController translationYController = new PIDController(0, 0, 0);
  private PIDController rotationController = new PIDController(0, 0, 0);

  private Pose2d currentPose;
  private double currentAngle;
  private double currentX;
  private double currentY;

  private double translationXSpeed;
  private double translationYSpeed;
  private double rotationSpeed;

  private DrivetrainSubsystem drivetrainSubsystem;

  public AutonomousDriveCommand(DrivetrainSubsystem drivetrainSubsystem, Translation2d targetTranslation, Rotation2d targetRotation) {
      this.drivetrainSubsystem = drivetrainSubsystem;
      addRequirements(drivetrainSubsystem);
      
      translationXController.setSetpoint(targetTranslation.getX()); 
      translationYController.setSetpoint(targetTranslation.getY());

      rotationController.setSetpoint(targetRotation.getRadians());
      rotationController.enableContinuousInput(0, 2 * Math.PI);
  }

  @Override
  public void initialize(){
    drivetrainSubsystem.resetPosition();
  }

  // TODO see if I can get rid of the cap method
  @Override
  public void execute() {
    currentPose = drivetrainSubsystem.getPose();
    currentX = currentPose.getX();
    currentY = currentPose.getY();
    currentAngle = currentPose.getRotation().getRadians();

    translationXSpeed = cap(translationXController.calculate(currentX));
    translationYSpeed = cap(translationYController.calculate(currentY));
    rotationSpeed = cap(rotationController.calculate(currentAngle));

    drivetrainSubsystem.drive(new ChassisSpeeds(translationXSpeed, translationYSpeed, rotationSpeed));
  }

  private double cap(double value) {
    return Math.max(-1, Math.min(value, 1));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}