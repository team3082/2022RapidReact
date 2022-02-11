package frc.robot.subsystems;

public class SwerveManager {

    private static SwerveMod[] m_swerveMods;

    public static void init() {
        m_swerveMods = new SwerveMod[] {
                new SwerveMod(0, 0, -8.75, 14.625),
                new SwerveMod(0, 0, 8.75, 14.625),
                new SwerveMod(0, 0, 8.75, -14.625),
                new SwerveMod(0, 0, -8.75, -14.625)
        };
    }

    public static void rotateAndDrive(double rotSpeed, double moveX, double moveY) {

        // Array containing the unclamped movement vectors of each module
        double[][] vectors = new double[m_swerveMods.length][2];
        // The greatest speed of any of the modules. If any one module's speed is
        // greater than 1.0, all of the speeds are scaled down.
        double max = 1.0;

        // Calculate unclamped movement vectors
        for (int i = 0; i < m_swerveMods.length; i++) {
            // The magnitude of the module's distance from the center of rotation
            double modPosMagnitude = Math.hypot(m_swerveMods[i].m_yPos, m_swerveMods[i].m_xPos);
            // The vector representing the direction the module should move to achieve the
            // desired rotation. Calculated by taking the derivative of the module's
            // position on the circle around the center of rotation, normalizing the
            // resulting vector, and scaling it by a factor of rotSpeed.
            double rotateX = (-1 * m_swerveMods[i].m_yPos / modPosMagnitude) * rotSpeed;
            double rotateY = (m_swerveMods[i].m_xPos / modPosMagnitude) * rotSpeed;

            // The final movement vector, calculated by summing movement and rotation
            // vectors
            double x = moveX + rotateX;
            double y = moveY + rotateY;

            vectors[i] = new double[] { x, y };
            max = Math.max(max, Math.hypot(x, y));
        }

        for (int i = 0; i < m_swerveMods.length; i++) {
            // Convert the movement vectors to a directions and magnitudes, clamping the
            // magnitudes based on 'max'
            double direction = Math.atan2(vectors[i][1], vectors[i][0]);
            double power = Math.hypot(vectors[i][0], vectors[i][1]) / max;

            // Drive the swerve modules
            m_swerveMods[i].rotateToRad(direction);
            m_swerveMods[i].drive(power);
        }

    }

}
