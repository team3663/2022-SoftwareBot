package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoDriveBackward extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;
  double targetXTranslation;

  public AutoDriveBackward(DrivetrainSubsystem drivetrainSubsystem, double targetXTranslation) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.targetXTranslation = targetXTranslation;

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    drivetrainSubsystem.resetPosition();
  }

  @Override
  public void execute() {
    drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-0.3, 0, 0,
                                  drivetrainSubsystem.getGyroscopeRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    double difference = Math.abs(drivetrainSubsystem.getPose().getX() - targetXTranslation);
    return (difference < 0.01);
  }
}
