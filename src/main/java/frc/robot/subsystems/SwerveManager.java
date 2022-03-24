package frc.robot.subsystems;

import frc.robot.robotmath.Vector2D;

public class SwerveManager {

    private static SwerveMod[] m_swerveMods;
    private static final double ticksPerRotationSteer = 2048 * 12.8;
    private static final double ticksPerRotationDrive = 2048 * 8.14;

    public static void init() {
        m_swerveMods = new SwerveMod[] {
                new SwerveMod(2, 1, -1, -1, 253.389, 0),
                new SwerveMod(4, 3, -1,  1, 202.412, 0),
                new SwerveMod(6, 5,  1,  1, 249.434, 0),
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
        //double relMoveX = moveX *  Math.cos(heading) + moveY * Math.sin(heading);
        //double relMoveY = moveX * -Math.sin(heading) + moveY * Math.cos(heading);

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
            
            Vector2D rotate = new Vector2D((-1 * m_swerveMods[i].m_pos.y / maxModPosMagnitude) * rotSpeed, (m_swerveMods[i].m_pos.x / maxModPosMagnitude) * rotSpeed);

            // The final movement vector, calculated by summing movement and rotation
            // vectors
            Vector2D rotMove = relMove.add(rotate);;

            vectors[i] = rotMove;
            maxSpeed = Math.max(maxSpeed, rotMove.mag());
        }

        for (int i = 0; i < m_swerveMods.length; i++) {
            // Convert the movement vectors to a directions and magnitudes, clamping the
            // magnitudes based on 'max'. An angle of 0 corresponds with a rightward movement
            // in vector space but causes a forward movement for the motors, so PI/2 must be 
            // subtracted from the vector's direction to compensate.
            double direction = vectors[i].atan() - Math.PI * 0.5;
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

    public static double getManualDistance(int id) {
        return m_swerveMods[id].m_drive.getSelectedSensorPosition()/ticksPerRotationDrive*(3*Math.PI);
    }

    public static double getVelocityDistance(int id) {
        return m_swerveMods[id].m_drive.getSelectedSensorVelocity()*10/ticksPerRotationDrive*(4*Math.PI);
    }

    public static double getAngle(int id) {
        return m_swerveMods[id].m_steer.getSelectedSensorPosition()/ticksPerRotationSteer*2*Math.PI;
    }

    public static void pointWheels(double radians) {
        for(int i = 0; i < m_swerveMods.length; i++)
            m_swerveMods[i].rotateToRad(radians);
    }
}
