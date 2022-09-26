package Power_Turret.TurretBuilder;

import Power_Turret.Turrets.Turret;
import Power_Turret.Utils.Configurations;
import Power_Turret.Utils.ControlerInterface;
import Power_Turret.Utils.ControlerType;
import Power_Turret.Utils.CtreControler;
import Power_Turret.Utils.RevControler;
import Power_Turret.Math.Vector2;
import Power_Turret.TargetSystem.Targeting;
import edu.wpi.first.wpilibj.DigitalInput;

public class TurretBuilder {
    private boolean m_targeting = false;
    private boolean m_odomitry = false;
    private DigitalInput m_hallEffectSenser = null;
    private ControlerInterface m_controler = null;

    private Targeting m_targetSystem = null;

    /**
     * <b> TurretBuilder </b>
     * <p>
     * The <b> TurretBuilder </b> alows one to build a complete turret with minimal
     * Configurations
     * when finished configurating the turret just use the build() to return a
     * turret object.
     * </p>
     * 
     * @parma <b> ControlerType controlerType </b> set the controler type
     * @parma <b> int CANId </b> CAN ID for the motorController
     **/
    public TurretBuilder(ControlerType controlerType, int CANId) {
        
        switch (controlerType) {
            case SparkMax:
                m_controler = new RevControler();
                break;
            case Falcon:
                m_controler = new CtreControler();
                break;
        }
        
        m_controler.setControler(CANId);
    }

    public TurretBuilder() {
    }

    /**
     * <b> Targeting </b>
     * <p>
     * targeting enables the targeting system for the Turret.
     * </p>
     *
     * @parma <b> Vector2 targetPos </b> is the position of the target.
     **/
    public TurretBuilder Targeting(Vector2 targetPos) {
        this.m_targeting = true;
        this.m_targetSystem = new Targeting(targetPos, 0.1);
        return this;
    }

    /**
     * <b> TurretVersion </b>
     * <p>
     * the turret vesion gives the turret all of the information from the
     * {@link Configurations} object.
     * </p>
     *
     * <p>
     * Mach the turret with is Version number or the conversion factors will be
     * wrong
     * </p>
     *
     * @parma <b> COnfigurations configuration </b> configuration holds all the info
     *        for the turret.
     **/
    public TurretBuilder TurretVersion(Configurations configuration) {
        m_controler.setConfiguration(configuration);
        return this;
    }

    /**
     * <b> Build </b>
     * <p>
     * compleates the building prosses
     * </p>
     * 
     * @see Turret
     * @return <b> New Turret </b> returns a new turret based on the configuration
     *         of the builder.
     **/
    public Turret Build() {
        Turret tmp = new Turret(m_targeting,m_targetSystem,m_odomitry,m_hallEffectSenser,m_controler);
        return tmp;
    }

}
