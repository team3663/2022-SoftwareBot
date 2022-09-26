// Turrets 
package Power_Turret.Turrets;

import Power_Turret.Math.*;
import Power_Turret.TargetSystem.Targeting;
import Power_Turret.Utils.ControlerInterface;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * <b> Turret </b>
 * <p>
 * Turret object for Power_turret.
 * </p>
 **/
public class Turret {
    private Radions m_targetAngle;
    private Radions m_currentAngle;
    // Tracking
    private boolean m_targeting;
    private boolean m_odomitry;
    private Vector2 m_currentPosition;
    private Vector2 m_targetPosition = Vector2.ZERO();
    private Vector2 m_targetingSystemPosition = Vector2.ZERO();

    // Aimming solution
    private TargetingSolution m_targetingSolution = TargetingSolution.Idle;

    // Hardware
    private ControlerInterface m_control;
    private DigitalInput m_hallEffectSenser = null;

    private Radions m_homePosition = Radions.ZERO();
    private boolean m_homed = false;

    // Systems
    private Targeting m_targetingSystem = null;

    public Turret(boolean targeting, Targeting targetSystem, boolean odomitry,
            DigitalInput hallEfectSenser, ControlerInterface controler) {
        m_targeting = targeting;
        m_targetingSystem = targetSystem;
        m_odomitry = odomitry;
        m_hallEffectSenser = hallEfectSenser;
        m_control = controler;
        SettingHardware();
    }

    public Turret(Targeting targetSystem, ControlerInterface controler) {
        m_targeting = true;
        m_odomitry = true;
        m_targetingSystem = targetSystem;
        m_control = controler;
    }

    public Turret() {
    }

    private void SettingHardware() {
        m_homed = false;
    }

    /**
     * <p>
     * The Hardware Check Spesifys if the hardware is a <b>TalonFX</b> system or a
     * <p>
     * <b>CANSparkMax</b> System
     * </p>
     * </p>
     *
     * @return <b>False</b> CANSparkMax <b>True</b> TalenFX
     */
    public void Home() {
        m_control.setRPM(5);
        // if (m_hallEffectSenser.get() == true) {
        if (true) {
            m_control.setRPM(5);
            m_control.setHomePosition(m_homePosition);
        }
    }

    /**
     * <b> GetHomed </b>
     * <p>
     * sees if the turret is homed.
     * </p>
     * 
     * @return m_homed true = homed
     **/
    public boolean GetHomed() {
        return m_homed;
    }

    /**
     * <b> GetCurrentRPM </b>
     * <p>
     * Gets the current angle in radions.
     * </p>
     *
     * @return <b> Radions </b> the angle of the turret in radions
     **/
    public Radions GetCurrentAngle() {
        return m_currentAngle;
    }

    /**
     * <b> GetTargetangle </b>
     * <p>
     * gets the targeted angle.
     * </p>
     *
     * @return <b> Radions </b> targeting angle.
     **/
    public Radions GetTargetAngle() {
        return m_targetAngle;
    }

    /**
     * <b> GetTargetPosition </b>
     * <p>
     * Gets the targets position
     * </p>
     *
     * @return <b> Vector2 </b> the current target position as a Vector2
     **/
    public Vector2 GetTargetPosition() {
        return m_targetPosition;
    }

    /**
     * <b> GetPower </b>
     * <p>
     * gets the current power of the controler
     * </p>
     *
     * @return <b> double </b> Power from controler
     **/
    public double GetPower() {
        return m_control.getPower();
    }

    /**
     * <b> GetCurrentRPM </b>
     * <p>
     * gets the current RPM of the turret.
     * </p>
     *
     * @return <b> Double </b> the current rpm of the turret
     **/
    public double GetCurrentRPM() {
        return m_control.getRPM();
    }

    /**
     * <b> SetTargetingSolution </b>
     * <p>
     * By setting the targeting solution it will allow one to swich
     * TargetingSolutions.
     * </p>
     * <p>
     * <b> Idle </b> Idle runs the turret at a constant power <b>(0.05) </b>.
     * </p>
     * <p>
     * <b> Rotation </b> Rotation allows the turret to be controled by spesifying an
     * Radion object.
     * </p>
     * <p>
     * <b> Position </b> Position allows the turret to be controled by spesifying a
     * Vector2.
     * </p>
     * <p>
     * <b> TargetingSystem </b> TargetingSystem alows the turret to be controled by
     * a TargetingSystem object.
     * </p>
     * 
     * @parma <b> Targetingsolution </b> idle, Radions, Position, targetSystem
     **/
    public void SetTargetingSolution(TargetingSolution targetingSolution) {
        m_targetingSolution = targetingSolution;
    }

    /**
     * <b> SetTargetPosition </b>
     * <p>
     * Sets the target position for the turret.
     * or where every you want the turret to aim at.
     * </p>
     *
     * @parma <b> pos </b> Vector2 of the targetPosition
     **/
    public void SetTargetPosition(Vector2 pos) {
        m_targetPosition = pos;
    }

    /**
     * <b> SetTargetPosition </b>
     * <p>
     * Seting the target position by inputing double x and double y.
     * </p>
     *
     * @parma <b> double x </b> the x cordnet of the target
     * @parma <b> double y </b> the y cordnet of the target
     **/
    public void SetTargetPosition(double x, double y) {
        m_targetPosition = Vector2.position(x, y);
    }

    /**
     * <b> SetTargetAngle </b>
     * <p>
     * Setting the an anble so the turret can just lock to that angle.
     * </p>
     *
     * @parma <b> Radions angle </b> the target angle in radions.
     **/
    public void SetTargetAngle(Radions angle) {
        m_targetAngle = angle;
    }

    /**
     * <b> UpdateOdomitry </b>
     * <p>
     * Updating The odomitry allows The <b>- targeting System </b>
     * to work and allows the turret to am at a point on the feeled.
     * </p>
     *
     * @param vec2 Current position
     */
    public void UpdateOdomitry(Vector2 vec2) {
        m_currentPosition = vec2;
    }

    /**
     * <b> UpdateTargetSystem </b>
     * <p>
     * Updating the targeting system will alow you to ame at the target and prodict
     * where
     * its going to be. This dosen't return anything it just updates the targeting
     * systems internely.
     * </p>
     *
     * @param targetVec2 The position of the target.
     */
    public void UpdateTargetSystem() {
        m_targetingSystem.Update(m_currentPosition);
        // m_targetingSystemPosition = m_targetingSystem.Get();
    }

    public Vector2 GetTargetingSystemPosition() {
        return m_targetingSystemPosition;
    }

    /**
     * <b> Update </b>
     * <p>
     * Updating all of the data that has been set and runs the
     * proper Targeting Solution.
     * </p>
     *
     * @see #SetTargetingSolution(TargetingSolution) to set the TargetingSolution.
     *
     *      <p>
     *      <b> Idle </b> just spins at a conston rate of 5% power.
     *      </p>
     *      <p>
     *      <b> Rotation </b> allows the turret to rotate to a spesified angle.
     *      </p>
     *      <p>
     *      <b> Position </b> allows the turret to keep the turret pointed toward a
     *      position that is spesified
     *      </p>
     *      <p>
     *      <b> TargetingSystem </b> allows the turret to use Algorithms to offset
     *      the m_targetPosition so that it's on point every shot
     *      </p>
     */
    public void Update() {
        switch (m_targetingSolution) {
            case TargetingSystem:
                m_control.setTargetPosition(m_targetingSystemPosition);
                break;
            case Rotation:
                m_control.setTargetAngle(m_targetAngle);
                break;
            case Position:
                m_control.setTargetPosition(m_targetPosition);
                break;
            case Idle:
                m_control.setPower(0.05);
                break;
        }
    }

}
