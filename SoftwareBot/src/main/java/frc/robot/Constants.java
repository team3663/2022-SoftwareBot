package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
    // left to right wheel, center to center
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(21); // 21.5 inches

    // front to back wheel, center to center
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(21); // 21.5 inches
    
    public static final double DRIVE_TRAIN_WHEEL_DIAMETER_METERS = 0.106325;

    public static final int DRIVETRAIN_PIGEON_ID = 13;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 5;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 9;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(297.7734375); 

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 6; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 10;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(23.291015625); 

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 3; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 7; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(95.27343750000001);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 4;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(310.60546875); 
}
