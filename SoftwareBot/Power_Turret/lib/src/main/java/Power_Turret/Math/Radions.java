package Power_Turret.Math;
import java.lang.Math;

public class Radions {
    private double rads;

    public Radions(double conversion, double val) {
        rads = (conversion/val) * (180 / Math.PI);
    }
    
    public Radions() {
    rads = 0;
    }

    public static Radions ZERO(){return new Radions();}

    // Converters 
    private double ConverterRads(double deg)    { return (deg * Math.PI) / 180;}
    // Converters Radions to degrease
    // @rads Radions
    // @return Degrease
    private double ConverterDegs(double rads)   { return (rads * (180 / Math.PI));} 

    public double GetRads() {return rads*(1/(Math.PI * 2));}
    public double GetDeg() {return ConverterDegs(rads);}
    //
    public void SetInDeg(double deg){
        rads = ConverterRads(deg);
    }

    public void SetInRads(double rads){
        this.rads = rads;
    }
    public void SetInRads(Radions rads){
        this.rads = rads.GetRads();
    }
    public void SetTargetVec(Vector2 vec2) {
        this.rads = Math.atan2(vec2.y, vec2.x);
    }
}
