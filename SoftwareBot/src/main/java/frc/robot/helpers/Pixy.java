package frc.robot.helpers;

import java.util.ArrayList;

import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;

public class Pixy {
    private static Pixy2 pixy;

    private final static byte LED_OFF = (byte) 0;
    private final static byte LED_ON = (byte) 1;
    private final static int LED_MAX_RBG = 255; // full white

    public final static int TEAM_RED = 1;
    public final static int TEAM_BLUE = 2;
    private int teamColor;

    private int maxBlocks = 25;

    public Pixy(int teamColor) {
        this.teamColor = teamColor;
    }

    // create pid instance, turn on led
    public void initialize() {
        pixy = Pixy2.createInstance(new SPILink());
        pixy.init();
        pixy.setLamp(LED_ON, LED_ON);
        pixy.setLED(LED_MAX_RBG, LED_MAX_RBG, LED_MAX_RBG);
        }
    
    public Block getLargestBlock() {
        int blockCount = pixy.getCCC().getBlocks(false, teamColor, maxBlocks);
        
        if (blockCount <= 0) {
            return null;
        }
        
        ArrayList<Block> blocks = pixy.getCCC().getBlockCache();

        Block largestBlock = null;
        for (Block block : blocks) {
            if (largestBlock == null) {
                largestBlock = block;
            } else if (getArea(block) > getArea(block)) {
                largestBlock = block;
            }
        }
        return largestBlock;
    }

    public double getArea(Block block) {
        double area = block.getWidth() * block.getHeight();
        return area;
    }
}
