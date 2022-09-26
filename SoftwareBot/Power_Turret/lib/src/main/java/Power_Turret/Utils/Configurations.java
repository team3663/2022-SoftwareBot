package Power_Turret.Utils;

public class Configurations {
    private double m_gearRatio = -1;
    private double m_maxRPM = -1;
    
    public Configurations(double gearRatio, double maxRPM){
        m_gearRatio = gearRatio;
        m_maxRPM = maxRPM;
    }
    public double GetGearRatio() { return m_gearRatio; }
    public double GetMaxRPM()    { return m_maxRPM; }
}

