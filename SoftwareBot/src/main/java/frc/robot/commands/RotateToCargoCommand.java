package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.helpers.Pixy;
import frc.robot.subsystems.DrivetrainSubsystem;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

public class RotateToCargoCommand extends CommandBase {
  private DrivetrainSubsystem drivetrainSubsystem;
  private static Pixy pixy;
  private PIDController translationXPidController = new PIDController(0.001, 0, 0); // tune
  private PIDController rotationPidController = new PIDController(0.025, 0, 0); // tune

  private static Block largestBlock = null;
  private double lastXOffset = 0;

  public RotateToCargoCommand(DrivetrainSubsystem drivetrainSubsystem) {

    pixy = new Pixy(Pixy.TEAM_RED);
    pixy.initialize();

    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);

    translationXPidController.setSetpoint(2000);
    rotationPidController.setSetpoint(155);
  }

  @Override
  public void initialize() {
    drivetrainSubsystem.resetPosition();
  }

  @Override
  public void execute() {
    Block largestBlock = pixy.getLargestBlock();
    this.largestBlock = largestBlock;

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

      double rotationSpeed;
      if (Math.abs(cargoXOffset - 155) > 3) {
        rotationSpeed = rotationPidController.calculate(cargoXOffset);
      }
      else {
        rotationSpeed = 0;
      }

      double translationXSpeed = translationXPidController.calculate(pixy.getArea(largestBlock));
      System.out.println("x speed" + translationXSpeed);

      drivetrainSubsystem.drive(new ChassisSpeeds(translationXSpeed, 0, rotationSpeed));
    }
  }

  // for the drivetrainSubsystem shuffleboard
  public static double getBallArea() {
    if (largestBlock == null) {
      return 0;
    }
    return pixy.getArea(largestBlock);
  }

  public static double getX(){
    if (largestBlock == null){
      return 0;
    }
    return largestBlock.getX();
  }

  public static double getY(){
    if (largestBlock == null){
      return 0;
    }
    return largestBlock.getY();
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
