package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoDriveBackward extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;
  double targetYTranslation;

  public AutoDriveBackward(DrivetrainSubsystem drivetrainSubsystem, double targetYranslation) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.targetYTranslation = targetYTranslation;

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    drivetrainSubsystem.resetPosition();
  }

  @Override
  public void execute() {
    drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0.3, 0, 0,
                                  drivetrainSubsystem.getGyroscopeRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(drivetrainSubsystem.getPose().getY() - targetYTranslation) < 0.1);
  }
}
