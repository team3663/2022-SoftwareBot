package Power_Turret.Utils;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import Power_Turret.Math.Vector2;
import Power_Turret.Math.Radions;

public class RevControler implements ControlerInterface {
    private CANSparkMax m_sparkMax;
    private CANSparkMax m_motor = new CANSparkMax(4, MotorType.kBrushless);
    private RelativeEncoder m_encoder;
    private SparkMaxPIDController m_PIDControler;
    public int motionSlot = 0;
    private static final int motorID = 4;

    @Override
    public void setControler(int id) {
        m_motor = new CANSparkMax(motorID, MotorType.kBrushless);
           //m_encoder = m_sparkMax.getEncoder();
        //m_encoder = m_sparkMax.getEncoder();
        //m_PIDControler = m_sparkMax.getPIDController();
    }

    @Override
    public void setPower(double power) {
        m_sparkMax.set(power);
    }

    @Override
    public void setRPM(double rpm) {
        m_PIDControler.setReference(rpm, CANSparkMax.ControlType.kSmartVelocity);
    }

    @Override
    public void setConfiguration(Configurations config) {
        m_encoder.setPositionConversionFactor(config.GetGearRatio());
    }

    @Override
    public void setTargetAngle(Radions rad) {
        m_PIDControler.setReference(rad.GetRads(), CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public void setTargetPosition(Vector2 vec2) {
        m_PIDControler.setReference(vec2.thada.GetRads(), CANSparkMax.ControlType.kVelocity);

    }

    @Override
    public void setMagicMotion(double MinVel, double MaxVel, double MaxAcc) {
        m_PIDControler.setSmartMotionMaxAccel(MinVel, motionSlot);
        m_PIDControler.setSmartMotionMaxVelocity(MaxVel, motionSlot);
        m_PIDControler.setSmartMotionMinOutputVelocity(MinVel, motionSlot);
    }

    @Override
    public void setPIDControler(double kP, double kI, double kD) {
        m_PIDControler.setP(kP);
        m_PIDControler.setI(kI);
        m_PIDControler.setD(kD);
    }

    @Override
    public void setHomePosition(Radions homePosition) {
        m_encoder.setPosition(homePosition.GetRads() / (Math.PI * 2));
    }

    @Override
    public Radions getPosition() {
        return new Radions(1, m_encoder.getPosition());
    }

    @Override
    public double getRPM() {
        return m_encoder.getVelocity();
    }

    @Override
    public double getPower() {
        return m_sparkMax.get();
    }

}
