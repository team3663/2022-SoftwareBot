package frc.robot.commands;
/*
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d; */

/*import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.HolonomicDriveSignal;*/


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.paths.ExampleTrajectory;
import frc.robot.subsystems.SS_Drivebase;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;


public class C_TrajectoryFollower extends CommandBase {

  private SS_Drivebase drivebase;

  private Translation2d targetTranslation;
  private double targetRotation;


  private double b = 2.0;
  private double zeta = 0.7;
  //TODO tune these values

  private RamseteController ramseteController = new RamseteController(b, zeta);

  private double currentAngle;
  private Pose2d currentPose;

  private double rotationPercentOutput;
  private double translationPercentOutput;


  private double translationSpeed;
  private double rotationSpeed;

  private double translationPercentTolerance = .025;
  private double rotationPercentTolerance = .01; 

  /**
   * @param drivebase the drivebase subsystem
   * @param targetTranslation inches for the robot to travel, negative inches are backwards
   * @param translationPercentOutput the maximum percent output for translation
   * @param rotation angle to turn to in radians
   * @param rotationPercentOutput the maximum percent output for rotation */
   
  public C_TrajectoryFollower(SS_Drivebase drivebase, Pose2d startPoint, double translationPercentOutput, double rotationPercentOutput, Pose2d endPoint, ArrayList<Translation2d> interiorWaypoints, TrajectoryConfig config) {
    this.drivebase = drivebase;
    addRequirements(drivebase);
    this.translationPercentOutput = translationPercentOutput;
    this.rotationPercentOutput = rotationPercentOutput;

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
          startPoint,
          interiorWaypoints,
          endPoint,
          config);


  
    
    
  }

  @Override
  public void initialize(){
    drivebase.resetPoseTranslation();
  }

  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  @Override
  public boolean isFinished() {
    return ((Math.abs(targetRotation - currentAngle) <= 2 * Math.PI * rotationPercentTolerance ||
        targetRotation == 0.0) &&
        (Math.abs(targetTranslation.getY() - currentPose.getY()) <= targetTranslation.getY() * translationPercentTolerance ||
        targetTranslation.getY() == 0.0)&&
        (Math.abs(targetTranslation.getX() - currentPose.getX()) <= targetTranslation.getX() * translationPercentTolerance ||
        targetTranslation.getX() == 0.0))
        ||(translationSpeed <= .001 && rotationSpeed <= .0005);
  }
}
