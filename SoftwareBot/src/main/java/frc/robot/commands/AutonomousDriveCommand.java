package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class AutonomousDriveCommand extends CommandBase {
  //private Vector2 targetTranslation;
  private Translation2d targetTranslation;
  private Rotation2d targetRotation;

  // TODO tune pid
  private PIDController translationXController = new PIDController(0.7, 0, 0);
  private PIDController translationYController = new PIDController(0.5, 0, 0);
  private PIDController rotationController = new PIDController(0.5, 0, 0);

  private Pose2d currentPose;
  private double currentAngle;
  private double currentX;
  private double currentY;

  private double translationXSpeed;
  private double translationYSpeed;
  private double rotationSpeed;

  private double translationPercentOutput;
  private double rotationPercentOutput;

  private DrivetrainSubsystem drivetrainSubsystem;

  public AutonomousDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                                Translation2d targetTranslation, double translationPercentOutput,
                                Rotation2d targetRotation, double rotationPercentOutput) {
      this.drivetrainSubsystem = drivetrainSubsystem;
      addRequirements(drivetrainSubsystem);
      
      this.targetTranslation = targetTranslation;
      this.targetRotation = targetRotation;
      
      this.translationPercentOutput = translationPercentOutput;
      this.rotationPercentOutput = rotationPercentOutput;
      
      translationXController.setSetpoint(targetTranslation.getX());
      translationXController.setTolerance(0.1);

      translationYController.setSetpoint(targetTranslation.getY());
      translationYController.setTolerance(0.1);

      rotationController.setSetpoint(targetRotation.getRadians());
      rotationController.enableContinuousInput(0, 2 * Math.PI);
      rotationController.setTolerance(0.1);
  }

  @Override
  public void initialize(){
    drivetrainSubsystem.resetPosition();
  }

  @Override
  public void execute() {
    currentPose = drivetrainSubsystem.getPose();
    currentX = currentPose.getX();
    currentY = currentPose.getY();
    currentAngle = currentPose.getRotation().getRadians();

    translationXSpeed = cap(translationXController.calculate(currentX));
    translationYSpeed = cap(translationYController.calculate(currentY));
    rotationSpeed = cap(rotationController.calculate(currentAngle));

    System.out.println("calculated speed: " + translationXSpeed);

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
    return translationXController.atSetpoint() && translationYController.atSetpoint() && rotationController.atSetpoint();
  }
}