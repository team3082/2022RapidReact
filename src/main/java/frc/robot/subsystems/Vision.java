package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {

    private static NetworkTable table;

    public static void init(){
        table = NetworkTableInstance.getDefault().getTable("vis");
    }

    public static double getAngleOffset(){
        //return Math.asin(table.getEntry("offsetX"));
        return 0;
    }

}
