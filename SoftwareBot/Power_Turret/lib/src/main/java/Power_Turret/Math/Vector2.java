package Power_Turret.Math;

public class Vector2 {
    public double x = 0;
    public double y = 0;
    public Radions thada = new Radions();

    public Vector2(double x,double y) {
        this.x = x;
        this.y = y;
    }

    public static Vector2 position(double x, double y) { 
        return new Vector2(x,y);
    }

    public double getThada() {
        return thada.GetRads();
    }

    public static double Dot(Vector2 a, Vector2 b) {
        double xd = a.x * b.x;
        double yd = a.y * b.y;

        return xd + yd;

    }

    public Vector2 Mult(double... vecs) {
        double tmpX = this.x;
        double tmpY = this.y;
        Vector2 tmpV;
        for (int i = 0; i < vecs.length; i++) {
            tmpX *= vecs[i];
            tmpY *= vecs[i];
        }
        tmpV = Vector2.position(tmpX, tmpY);
        return tmpV;

    }

    public static Vector2 Mult(Vector2... vecs) {
        double tmpX = 0;
        double tmpY = 0;
        Vector2 tmpV;
        for (int i = 0; i < vecs.length; i++) {
            tmpX *= vecs[i].x;
            tmpY *= vecs[i].y;
        }
        tmpV = Vector2.position(tmpX, tmpY);
        return tmpV;

    }

    public static Vector2 Dev(Vector2 vec2, double x) {
        Vector2 tmp = vec2;
        tmp.x = tmp.x/x;
        tmp.y = tmp.y/x;
        return tmp;
    }

    public static Vector2 Dev(Vector2 vec2, Vector2 dvec2) {
        Vector2 tmp = vec2;
        tmp.x = tmp.x/dvec2.x;
        tmp.y = tmp.y/dvec2.y;
        return tmp;
    }

    public static Vector2 Sub(Vector2... vecs) {
        double tmpX = 0;
        double tmpY = 0;
        Vector2 tmpV;
        for (int i = 0; i < vecs.length; i++) {
            tmpX -= vecs[i].x;
            tmpY -= vecs[i].y;
        }
        tmpV = Vector2.position(tmpX, tmpY);
        return tmpV;

    }

    public static Vector2 Add(Vector2... vecs) {
        double tmpX = 0;
        double tmpY = 0;
        Vector2 tmpV;
        for (int i = 0; i < vecs.length; i++) {
            tmpX += vecs[i].x;
            tmpY += vecs[i].y;
        }
        tmpV = Vector2.position(tmpX, tmpY);
        return tmpV;

    }

    public static Vector2 ZERO() {
        return Vector2.position(0, 0);
    }
}
