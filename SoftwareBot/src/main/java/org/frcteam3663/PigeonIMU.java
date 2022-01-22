package org.frcteam3663;

import edu.wpi.first.wpilibj.interfaces.Gyro;

public class PigeonIMU implements Gyro
{
    private com.ctre.phoenix.sensors.PigeonIMU imu;

    public PigeonIMU(int canId)
    {
        imu = new com.ctre.phoenix.sensors.PigeonIMU(canId);
    }

    @Override
    public void close() throws Exception {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void calibrate() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void reset() {
        imu.setFusedHeading(0.0);
    }

    @Override
    public double getAngle() {
        return imu.getFusedHeading();
    }

    @Override
    public double getRate() {
        // TODO Auto-generated method stub
        return 0;
    }

}