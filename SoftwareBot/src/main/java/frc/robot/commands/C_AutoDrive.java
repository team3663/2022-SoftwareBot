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
import frc.robot.subsystems.SS_Drivebase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;


public class C_AutoDrive extends CommandBase {

  private SS_Drivebase drivebase;

  //private Vector2 targetTranslation;
  private Translation2d targetTranslation;
  private double targetRotation;

// TODO tune pid with Yvonne
  // private double translationKp = .013;
  // private double translationKi = 0;
  // private double translationKd = 0;
  //private PidConstants translationConstants = new PidConstants(translationKp, translationKi, translationKd);

  // private double rotationKp = .0293;
  // private double rotationKi = 0;
  // private double rotationKd = 0;
  //private PidConstants rotationConstants = new PidConstants(rotationKp, rotationKi, rotationKd);

  private double b = 2.0;
  private double zeta = 0.7;
  //TODO tune these values

  private RamseteController ramseteController = new RamseteController(b, zeta);

  //private PIDController translationController = new PIDController(translationKp, translationKi, translationKd);
  private double translationPercentTolerance = .025;
  
  //private PIDController rotationController = new PIDController(rotationKp, rotationKi, rotationKd);*/
  private double rotationPercentTolerance = .01; 

  private double currentAngle;
  private Pose2d currentPose;


  private double translationSpeed;
  private double rotationSpeed;

  private double translationPercentOutput;
  private double rotationPercentOutput;

  /**
   * @param drivebase the drivebase subsystem
   * @param targetTranslation inches for the robot to travel, negative inches are backwards
   * @param translationPercentOutput the maximum percent output for translation
   * @param rotation angle to turn to in radians
   * @param rotationPercentOutput the maximum percent output for rotation */
   
  public C_AutoDrive(SS_Drivebase drivebase, Translation2d targetTranslation, double translationPercentOutput, double targetRotation, double rotationPercentOutput) {
    this.drivebase = drivebase;
    this.targetTranslation = targetTranslation;
    this.targetRotation = targetRotation;
    addRequirements(drivebase);
    this.translationPercentOutput = translationPercentOutput;
    this.rotationPercentOutput = rotationPercentOutput;

    this.
    //finds hypot of the translation and sets its desired point to it
    //translationController.setSetpoint(targetTranslation.getNorm());


    
    //sets the desired angle 
    //rotationController.setSetpoint(targetRotation);
    //rotationController.setInputRange(0, 2 * Math.PI);
    //combines the previous methods setInputRange and setContinuous

    //TODO find how to setPoint and inputRange using ramseteController

  
    rotationController.enableContinuousInput(0, 2 * Math.PI);

    
  }

  @Override
  public void initialize(){
    drivebase.resetPoseTranslation();
  }

  @Override
  public void execute() {
    

    currentPose = drivebase.getPose();
    currentAngle = currentPose.getRotation().getRadians();
    translationSpeed = translationController.calculate(currentPose.getTranslation().getNorm()) * translationPercentOutput;

    
    Pose2d pose = new Pose2d()
    ChassisSpeeds speeds = ramseteController.calculate(drivebase.getPose(), /*new Pose2d(targetTranslation.getX(), 
    targetTranslation.getY(), Rotation2d.fromDegrees(targetRotation)) */ new Trajectory.State(double timeSeconds,
double velocityMetersPerSecond, double accelerationMetersPerSecondSq, Pose2d poseMeters, double curvatureRadPerMeter));
    //TODO find out what Trajectory.State is

    //second translation speed
    double tSpeed = Math.max(-1, Math.min(translationSpeed, 1));
    
    // this line is the attempt at recreating the old code
    Translation2d translationVector = targetTranslation.div(targetTranslation.getNorm()).times(translationSpeed);
    //.times(1 / targetTranslation.getNorm()).times(translationSpeed);
    //TODO scale to translationSpeed
      //targetTranslation.getX() / targetTranslation.getNorm()) 

      //this line is from the original code
    //Vector2 translationVector = Vector2.fromAngle(targetTranslation.getAngle()).normal().scale(translationSpeed);

    rotationSpeed = rotationController.calculate(currentAngle) * rotationPercentOutput;
    //second rotation speed
    double rSpeed = Math.max(-1, Math.min(rotationSpeed, 1));
    
    drivebase.drive(new ChassisSpeeds(translationVector.getX(), translationVector.getY(), rSpeed));
  }

  @Override
  public void end(boolean interrupted) {
    //drives a ChassisSpeeds with all values being zero
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
