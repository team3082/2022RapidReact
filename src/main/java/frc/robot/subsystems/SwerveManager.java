package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import frc.robot.robotmath.Vector2D;

public class SwerveManager {

    private static SwerveMod[] m_swerveMods;

    public static void init() {
        m_swerveMods = new SwerveMod[] {
                new SwerveMod(2, 1, -1, -1, 253.389, 0),
                new SwerveMod(4, 3, -1,  1, 203.115, 0),
                new SwerveMod(6, 5,  1,  1, 227.461, 0),
                new SwerveMod(8, 7,  1, -1, 129.990, 0),
        };
    }
    
    // Zero the encoder output of each of the steering motors
    public static void zeroSteeringEncoders() {
        for (SwerveMod mod : m_swerveMods) {
            mod.resetSteerSensor();
        }
    }

    public static void rotateAndDrive(double rotSpeed, Vector2D move) {


        double heading = Pigeon.getRotation();
        heading = heading / 180.0 * Math.PI;

        // Array containing the unclamped movement vectors of each module
        Vector2D[] vectors = new Vector2D[m_swerveMods.length];

        // Multiply the movement vector by a rotation matrix to compensate for the pigeon's heading
        Vector2D relMove = move.rotate(heading);

        // The greatest magnitude of any module's distance from the center of rotation
        double maxModPosMagnitude = 0;
        for (int i = 0; i < m_swerveMods.length; i++) {
            maxModPosMagnitude = Math.max(maxModPosMagnitude,
                    m_swerveMods[i].m_pos.mag());
        }

        // The greatest speed of any of the modules. If any one module's speed is
        // greater than 1.0, all of the speeds are scaled down.
        double maxSpeed = 1.0;

        // Calculate unclamped movement vectors
        for (int i = 0; i < m_swerveMods.length; i++) {
            // The vector representing the direction the module should move to achieve the
            // desired rotation. Calculated by taking the derivative of the module's
            // position on the circle around the center of rotation, normalizing the
            // resulting vector according to maxModPosMagnitude (such that the magnitude of
            // the largest vector is 1), and scaling it by a factor of rotSpeed.
            
            Vector2D rotate = new Vector2D(
                (-1 * m_swerveMods[i].m_pos.y / maxModPosMagnitude) * rotSpeed, 
                (     m_swerveMods[i].m_pos.x / maxModPosMagnitude) * rotSpeed);

            // The final movement vector, calculated by summing movement and rotation
            // vectors
            Vector2D rotMove = relMove.add(rotate);

            vectors[i] = rotMove;
            maxSpeed = Math.max(maxSpeed, rotMove.mag());
        }

        for (int i = 0; i < m_swerveMods.length; i++) {
            // Convert the movement vectors to a directions and magnitudes, clamping the
            // magnitudes based on 'max'. 
            double direction = vectors[i].atan();
            double power = vectors[i].mag() / maxSpeed;

            // Drive the swerve modules
            if(power != 0)
                m_swerveMods[i].rotateToRad(direction);
            m_swerveMods[i].drive(power);
        }


    }

    public static double getEncoderPos(int id){
        return m_swerveMods[id].m_drive.getSelectedSensorPosition();
    }    

    public static double getDrivePosition(int id) {
        return m_swerveMods[id].getDrivePosition();
    }

    public static double getDriveVelocity(int id) {
        return m_swerveMods[id].getDriveVelocity();
    }

    public static double getSteerAngle(int id) {
        return m_swerveMods[id].getSteerAngle();
    }

    public static void pointWheels(double radians) {
        for(int i = 0; i < m_swerveMods.length; i++)
            m_swerveMods[i].rotateToRad(radians);
    }

    public static void calibrationTest()
    {
        for(int i = 0; i < m_swerveMods.length; i++) {
            m_swerveMods[i].m_steer.set(TalonFXControlMode.MotionMagic, 0);
            m_swerveMods[i].m_drive.set(TalonFXControlMode.PercentOutput, 0.1);;
        }

    }
}
