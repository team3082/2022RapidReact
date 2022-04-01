package frc.robot.robotmath;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

public class RTime {

    private static double m_starttime;
    private static double m_time;
    private static double m_deltatime;

    // Hardware clock in seconds
    private static double FPGATime() {
        return Timer.getFPGATimestamp();
    }

    // Zero the timers
    public static void init() {
        m_starttime = FPGATime();
        m_time = 0;
    }

    // Update the timers
    public static void update() {
        double curtime = FPGATime() - m_starttime;
        m_deltatime = curtime - m_time; 
        m_time = curtime;
    }

    // Current time since init in seconds
    public static double getTime() {
        return m_time;
    }

    // Time since the last time update was called in seconds
    public static double getDeltaTime() {
        // TODO: Check if true time or period has better results
        //return m_deltatime;
        return Robot.kDefaultPeriod;
    }


}