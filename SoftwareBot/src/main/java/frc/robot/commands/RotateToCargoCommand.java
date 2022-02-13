package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.helpers.Pixy;
import frc.robot.subsystems.DrivetrainSubsystem;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

// Rotate and drive to the closest cargo
public class RotateToCargoCommand extends CommandBase {
  private DrivetrainSubsystem drivetrainSubsystem;
  private Pixy pixy;
  private PIDController translationXPidController = new PIDController(0, 0, 0); // tune
  private PIDController rotationPidController = new PIDController(0.025, 0, 0); // acceptable, but needs fine tune

  private double lastXOffset = 0;

  public RotateToCargoCommand(DrivetrainSubsystem drivetrainSubsystem) {

    pixy = new Pixy(Pixy.TEAM_RED);
    pixy.initialize();

    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);

    translationXPidController.setSetpoint(0); //change
    rotationPidController.setSetpoint(155);
  }

  @Override
  public void initialize() {
    drivetrainSubsystem.resetPosition();
  }

  @Override
  public void execute() {
    Block largestBlock = pixy.getLargestBlock();

    // if no ball is detected, rotate towards the edge from which the ball disappears
    if (largestBlock == null) {
      if (lastXOffset < 155){
        drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 1));
      }
      else {
        drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, -1));
      }
    }
    // rotate to the largest ball seen by the pixy camera
    else {
      double cargoXOffset = largestBlock.getX();
      lastXOffset = cargoXOffset;

      double rotationSpeed = rotationPidController.calculate(cargoXOffset);
      drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, rotationSpeed));
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // get ball area method, send to drivetrainSubsystem periodic for shuffleboard update
}
