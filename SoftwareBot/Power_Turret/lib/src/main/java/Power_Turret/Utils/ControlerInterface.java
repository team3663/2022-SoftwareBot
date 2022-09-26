package Power_Turret.Utils;

import Power_Turret.Math.Radions;
import Power_Turret.Math.Vector2;


public interface ControlerInterface {

    public void setControler(int id);
    public void setPower(double power);
    public void setRPM(double rmp);
    public void setConfiguration(Configurations config);
    public void setTargetAngle(Radions rad); 
    public void setTargetPosition(Vector2 vec2);
    public void setMagicMotion(double MinVel, double MaxVel, double MaxAcc);
    public void setPIDControler(double kP, double kI, double kD);
    public void setHomePosition(Radions homePosition);
    public Radions getPosition();
    public double getRPM();
    public double getPower();
}
