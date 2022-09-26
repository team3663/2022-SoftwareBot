package Power_Turret.TargetSystem;

import Power_Turret.Math.Vector2;

public class Targeting {
    private Vector2 m_prevPosition;
    private Vector2 m_currPosition;
    private Vector2 m_currVel;
    private Vector2 m_futurePosition;
    private double m_projectileVel;
    private double m_timeStamp;
    private boolean m_useProjectileVel = false;
    private Vector2 m_aimVel;
    private Vector2 m_targetPos;

    /*
     * Creating Targeting system with initol target position, target size, time
     * stape.
     *
     * Enables error for size of the target.
     *
     * TODO acounts for error do to the size of the target.
     * 
     * @var targetPos - position of target
     * 
     * @var TargetSize - size of target
     * 
     * @var timeStamp - update rate of targeting system
     */
    public Targeting(Vector2 targetPos, Vector2 targetSize, double timeStamp) {
        m_targetPos = targetPos;
        m_timeStamp = timeStamp;
    }

    /*
     * Creating Targeting system with initol target position, target size, time
     * stape.
     * Enable targeting system by using using projectile velocity
     *
     * @var ProjectileVel - projectile's velosity in meeters per second
     *
     * @var targetPos - position of target
     *
     * @var timeStamp - update rate of targeting system
     */
    public Targeting(double ProjectileVel, Vector2 targetPos, double timeStamp) {
        m_projectileVel = ProjectileVel;
        m_useProjectileVel = true;
        m_targetPos = targetPos;
        m_timeStamp = timeStamp;
    }

    /*
     * Creating Targeting system with initol target position, target size, time
     * stape.
     * 
     * Defalt Confugration
     *
     * @var targetPos - position of target
     *
     * @var timeStamp - update rate of targeting system
     * */
    public Targeting(Vector2 targetPos, double timeStamp) {
        m_timeStamp = timeStamp;
        m_targetPos = targetPos;
        m_useProjectileVel = false;
    }

    /* <b> Update</b> 
     * <p>
     *  Update updates the current position to calculate the speed of the Robot
     *  then creates a fucher position to help calculate where the turret need 
     *  to aim to hit the targe exsact.
     * </p>
     *
     * */
    public void Update(Vector2 currentPosition) {
        if (!m_useProjectileVel) {
            if (m_prevPosition == null) {
                m_prevPosition = currentPosition;
            } else {
                m_prevPosition = m_currPosition;
            }
            m_currPosition = currentPosition;

            m_futurePosition = Vector2.Add(
                    Vector2.Mult(m_currPosition, Vector2.position(2, 2)),
                    Vector2.Mult(m_prevPosition, Vector2.position(-1, -1)));

            m_aimVel = Vector2.Add(
                    Vector2.Dev(Vector2.Sub(m_currPosition, m_prevPosition), m_timeStamp),
                    m_targetPos);

            m_currVel = Vector2.Dev(Vector2.Sub(m_prevPosition, m_currPosition), m_timeStamp);

            return;
        }
        return;
    }

    /** <b> Get </b> 
     * 
     * @return The velocity to offset the aim.
     **/
    public Vector2 Get() {
        return m_aimVel;
    }

     /** <b> GetFuturePosition </b> 
      *
      * @return Future position of the robot.
      **/
    public Vector2 GetFuturePosition() {
        return m_futurePosition;
    }

    /** <b> ProdictedPosition </b> 
     * <p>
     * Prodicted Position alows you to aim with perfect persision and with great
     * speed.
     * </p>
     * @return The prodicted position based on you position.
     **/
    public Vector2 ProdictedPosition() {
        Vector2 q = Vector2.Sub(m_currPosition, m_targetPos);

        double a = Vector2.Dot(m_currPosition, m_currPosition) - Math.pow(m_projectileVel, 2);
        double b = 2 * Vector2.Dot(m_currPosition, q);
        double c = Vector2.Dot(q, q);

        double disc = Math.pow(b, 2) - 4 * a * c;
        double d = Math.sqrt(disc);

        double t1 = (-b + d) / (2 * a);
        double t2 = (-b - d) / (2 * a);

        var time = Math.max(t1, t2);

        Vector2 prodictedPosition = Vector2.Add(m_currPosition, m_currVel.Mult(time));
        return prodictedPosition;
    }
}
