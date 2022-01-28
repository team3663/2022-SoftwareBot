package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.SS_Drivebase;
import edu.wpi.first.math.Vector;
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
  private PIDController translationXController = new PIDController(0.013, 0, 0);
  private PIDController translationYController = new PIDController(0.013, 0, 0);
  private PIDController rotationController = new PIDController(0.0293, 0, 0);

  private Pose2d currentPose;
  private double currentAngle;
  private double currentX;
  private double currentY;

  private double translationXSpeed;
  private double translationYSpeed;
  private double rotationSpeed;

  private double translationPercentOutput;
  private double rotationPercentOutput;

  private SS_Drivebase drivetrainSubsystem;

  public AutonomousDriveCommand(SS_Drivebase drivetrainSubsystem,
                                Translation2d targetTranslation, double translationPercentOutput,
                                Rotation2d targetRotation, double rotationPercentOutput) {
      this.drivetrainSubsystem = drivetrainSubsystem;
      
      this.targetTranslation = targetTranslation;
      this.targetRotation = targetRotation;
      
      this.translationPercentOutput = translationPercentOutput;
      this.rotationPercentOutput = rotationPercentOutput;
      
      translationXController.setSetpoint(targetTranslation.getX());
      translationXController.setTolerance(0.025);

      translationYController.setSetpoint(targetTranslation.getY());
      translationYController.setTolerance(0.025);

      rotationController.setSetpoint(targetRotation.getRadians());
      rotationController.enableContinuousInput(0, 2 * Math.PI);
      rotationController.setTolerance(0.01);
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

    translationXSpeed = translationXController.calculate(currentX) * translationPercentOutput;
    translationYSpeed = translationYController.calculate(currentY) * translationPercentOutput;
    rotationSpeed = rotationController.calculate(currentAngle) * rotationPercentOutput;

    drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(translationXSpeed, translationYSpeed, rotationSpeed,
                                                                    drivetrainSubsystem.getGyroscopeRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("auto ended");
    drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  @Override
  public boolean isFinished() {
    return translationXController.atSetpoint() && translationYController.atSetpoint() && rotationController.atSetpoint();
  }
}