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

        for(int i=0; i<m_swerveMods.length;i++){
            double x = moveX + -1*m_swerveMods[i].m_yPos*rotSpeed;
            double y = moveY + m_swerveMods[i].m_xPos*rotSpeed;

            double direction = Math.atan2(y, x);
            double power = Math.hypot(x, y);
            
            m_swerveMods[i].rotateToRad(direction);
            m_swerveMods[i].drive(power);
        }

    }


    
}
