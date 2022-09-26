package PowerTurret;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Scanner;

import org.junit.jupiter.api.Test;

import Power_Turret.Math.Vector2;
import Power_Turret.TargetSystem.Targeting;

class TurretTest {

    @Test
    public void TargetingCorectPositionOne() {
        // WPI_TalonFX t = new WPI_TalonFX(1);
        Targeting targeting = new Targeting(5 / 0.1, Vector2.position(10, 10), 0.1);
        Vector2 currentPosition = Vector2.position(0, 0);
        // structer of the test file
        // \n{(x,y)(x,y)}(awser)
        ArrayList<Vector2> points = new ArrayList<Vector2>();
        ArrayList<Vector2> corectTargetingPosition = new ArrayList<Vector2>();
        Scanner sc = null; 
        try {
            FileReader f = new FileReader("./TestPoints.test");
            sc = new Scanner(f);
        
       System.out.print("testing\n");
       while (sc.hasNextLine()) {
           String data = sc.nextLine();
           String colector = "";
           boolean colecting = false;
           boolean colectingAnser = false;
           for (int i = 0; i < data.length(); i++) {
               if (data.charAt(i) == '{') {
                   colectingAnser = false;
                   continue;
               }
               if (data.charAt(i) == '}') {
                   colectingAnser = true;
                   continue;
               }
               if (data.charAt(i) == '(') {
                   colector = "";
                   colecting = true;
                   continue;
               } else if (data.charAt(i) == ')') {
                   colecting = false;
                   if (colectingAnser) {
                       corectTargetingPosition.add(Vector2.position(Double.parseDouble(colector.split(",")[0]),
                               Double.parseDouble(colector.split(",")[1])));
                   } else {
                       points.add(Vector2.position(Double.parseDouble(colector.split(",")[0]),
                               Double.parseDouble(colector.split(",")[1])));
                   }
                   continue;
               }
               if (colecting) {
                   colector += data.charAt(i);
               }
           }

       }
        for (int i = 0; i < points.size(); i++) {

            currentPosition = points.get(i);
            targeting.Update(currentPosition);
            if (i % 2 == 0) {
                assertEquals(targeting.GetFuturePosition().x + 1, corectTargetingPosition.get(i).x,
                        "TargetingSystem faled");
                assertEquals(targeting.GetFuturePosition().y, corectTargetingPosition.get(i).y,
                        "TargetingSystem faled");
                System.out.print("going\r");
            }
        }
        } catch (FileNotFoundException ex) {
        }
        // turret.UpdateOdomitry(Vector2.position(4, 4));
        // turret.UpdateTargetSystem();
        // turret.UpdateOdomitry(Vector2.position(3, 3));

    }

}
