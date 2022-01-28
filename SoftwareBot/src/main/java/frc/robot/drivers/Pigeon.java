package frc.robot.drivers;

import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;


public class Pigeon implements Gyro, Accelerometer {
    
    private final PigeonIMU pigeon;
    private Range range;
    double[] tiltAngles = new double[3];

    public Pigeon (int deviceNumber){
        pigeon = new PigeonIMU(deviceNumber);
    }

    
    @Override
    public void setRange(Range range) {
        this.range = range;
    }
    @Override
    /**
     *  The acceleration along the x axis in g-forces
     */
    public double getX() {
        pigeon.getAccelerometerAngles(tiltAngles);
        return Math.sin(tiltAngles[0]);
    }
    @Override
    /**
     * The acceleration along the y axis in g-forces
     */
    public double getY() {
        pigeon.getAccelerometerAngles(tiltAngles);
        return Math.sin(tiltAngles[1]);
    }
    @Override
    /**
     * The acceleratoin along the z axis in g-forces
     */
    public double getZ() {
        pigeon.getAccelerometerAngles(tiltAngles);
        return Math.sin(tiltAngles[2]);
    }
    @Override
    public void calibrate() {
        pigeon.enterCalibrationMode(CalibrationMode.BootTareGyroAccel);
    }
    @Override
    public void reset() {
        pigeon.setFusedHeading(0.0);
    }
    @Override
    public double getAngle() {       
        return pigeon.getFusedHeading();
    }
    @Override
    public double getRate() {
        // TODO
        return 0; //TODO fix this
    }


    @Override
    public void close() throws Exception {
        // TODO 
    }

}
