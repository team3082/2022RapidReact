package frc.robot.subsystems;

public class SwerveManager {
    
    private static SwerveMod[] m_swerveMods;

    public static void init(){
        m_swerveMods = new SwerveMod[]{new SwerveMod(0,0,-1,2), 
                                    new SwerveMod(0,0,1,2),
                                    new SwerveMod(0,0,1,-2),
                                    new SwerveMod(0,0,-1,-2)};
    }

    public static void rotateAndDrive(double rotSpeed, double moveX, double moveY){

        double[][] vectors = new double[m_swerveMods.length][2];
        double max = 1.0;

        for(int i=0; i<m_swerveMods.length;i++){
            double x = moveX + -1*m_swerveMods[i].m_yPos*rotSpeed;
            double y = moveY + m_swerveMods[i].m_xPos*rotSpeed;

            vectors[i] = new double[]{x, y};
            max = Math.max(max, Math.hypot(x, y));
        }

        for(int i=0; i<m_swerveMods.length;i++){
            double direction = Math.atan2(vectors[i][1], vectors[i][0]);
            double power = Math.hypot(vectors[i][0], vectors[i][1])/max;
            
            m_swerveMods[i].rotateToRad(direction);
            m_swerveMods[i].drive(power);
        }

    }
    
}