package frc.robot.subsystems;

public class SwerveManager {

    private static SwerveMod[] m_swerveMods;

    public static void init() {
        m_swerveMods = new SwerveMod[] {
                new SwerveMod(2, 1, -1, -1, 253.213),
                new SwerveMod(4, 3, -1,  1, 202.500),
                new SwerveMod(6, 5,  1,  1, 253.125),
                new SwerveMod(8, 7,  1, -1, 127.178)
        };
    }
    
    // Zero the encoder output of each of the steering motors
    public static void zeroSteeringEncoders() {
        for (SwerveMod mod : m_swerveMods) {
            mod.resetSteerSensor();
        }
    }

    public static void rotateAndDrive(double rotSpeed, double moveX, double moveY, double heading) {
        // Array containing the unclamped movement vectors of each module
        double[][] vectors = new double[m_swerveMods.length][2];
        // The greatest speed of any of the modules. If any one module's speed is
        // greater than 1.0, all of the speeds are scaled down.
        double maxSpeed = 1.0;

        // Multiply the movement vector by a rotation matrix to compensate for the pigeon's heading
        double relMoveX = moveX *  Math.cos(heading) + moveY * Math.sin(heading);
        double relMoveY = moveX * -Math.sin(heading) + moveY * Math.cos(heading);

        // The greatest magnitude of any module's distance from the center of rotation
        double maxModPosMagnitude = 0;
        for (int i = 0; i < m_swerveMods.length; i++) {
            maxModPosMagnitude = Math.max(maxModPosMagnitude,
                    Math.hypot(m_swerveMods[i].m_yPos, m_swerveMods[i].m_xPos));
        }

        // Calculate unclamped movement vectors
        for (int i = 0; i < m_swerveMods.length; i++) {
            // The vector representing the direction the module should move to achieve the
            // desired rotation. Calculated by taking the derivative of the module's
            // position on the circle around the center of rotation, normalizing the
            // resulting vector according to maxModPosMagnitude (such that the magnitude of
            // the largest vector is 1), and scaling it by a factor of rotSpeed.
            double rotateX = (-1 * m_swerveMods[i].m_yPos / maxModPosMagnitude) * rotSpeed;
            double rotateY = (m_swerveMods[i].m_xPos / maxModPosMagnitude) * rotSpeed;

            // The final movement vector, calculated by summing movement and rotation
            // vectors
            double x = relMoveX + rotateX;
            double y = relMoveY + rotateY;

            vectors[i] = new double[] { x, y };
            maxSpeed = Math.max(maxSpeed, Math.hypot(x, y));
        }

        for (int i = 0; i < m_swerveMods.length; i++) {
            // Convert the movement vectors to a directions and magnitudes, clamping the
            // magnitudes based on 'max'. An angle of 0 corresponds with a rightward movement
            // in vector space but causes a forward movement for the motors, so PI/2 must be 
            // subtracted from the vector's direction to compensate.
            double direction = Math.atan2(vectors[i][1], vectors[i][0]) - Math.PI * 0.5;
            double power = Math.hypot(vectors[i][0], vectors[i][1]) / maxSpeed;

            // Drive the swerve modules
            if (Math.abs(power) > 0.1)
                m_swerveMods[i].rotateToRad(direction);
            m_swerveMods[i].drive(power);
        }



        for(int i =0; i < m_swerveMods.length; i++)
        {
            //System.out.println(i + ": " + m_swerveMods[i].m_absEncoder.getAbsolutePosition() + " __ " + m_swerveMods[i].m_steer.getSelectedSensorPosition());
        }
    }

}
