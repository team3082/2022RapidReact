package frc.robot.auto;

import frc.robot.robotmath.RTime;
import frc.robot.robotmath.Vector2D;
import frc.robot.subsystems.AutoAlign;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveManager;
import frc.robot.subsystems.SwervePosition;

public class BasicAuto {
    
    // INSTruction
    enum INST 
    {
        MOVE(),
        MOVETOCOORD(),
        MOVETOANDLOOKAT(),
        LOOKAT(),
        AUTOALIGN(),
        ROTATE(),
        INTAKE(),
        REVFORDIST(),
        SHOOT(),
        WAIT(),
        AUTOREV(),
    }

    public static class AutoFrame
    {

        static public AutoFrame Rotate(double angle)
        {
            AutoFrame frame = new AutoFrame();
            frame.instruction = INST.ROTATE;
            frame.targetangle = angle;
            return frame;
        }
        static public AutoFrame MoveToAndLookAt(double x, double y)
        {
            AutoFrame frame = new AutoFrame();
            frame.instruction = INST.MOVETOANDLOOKAT;
            frame.move = new Vector2D(x,y);
            return frame;
        }
        static public AutoFrame LookAt(double x, double y)
        {
            AutoFrame frame = new AutoFrame();
            frame.instruction = INST.LOOKAT;
            frame.move = new Vector2D(x,y);
            return frame;
        }        
        static public AutoFrame Move(double movex, double movey, double distance)
        {
            AutoFrame frame = new AutoFrame();
            frame.instruction = INST.MOVE;
            frame.move.x = movex;
            frame.move.y = movey;
            frame.movedistance = distance; // do unit conversion from meters to ticks here
            return frame;
        }
        static public AutoFrame MoveTo(double x, double y)
        {
            AutoFrame frame = new AutoFrame();
            frame.instruction = INST.MOVETOCOORD;
            frame.move = new Vector2D(x,y);
            return frame;
        }
        static public AutoFrame Intake(boolean intakeOn)
        {
            AutoFrame frame = new AutoFrame();
            frame.instruction = INST.INTAKE;
            frame.intakeOn = intakeOn;
            return frame;
        }
        static public AutoFrame RevForDist(double x, double y)
        {
            AutoFrame frame = new AutoFrame();
            frame.instruction = INST.REVFORDIST;
            frame.dist = Math.hypot(x, y)/12.0;
            return frame;
        }
        static public AutoFrame Shoot(){
            AutoFrame frame = new AutoFrame();
            frame.instruction = INST.SHOOT;
            frame.stopTime = Double.MAX_VALUE;
            return frame;
        }
        static public AutoFrame AutoAlign(){
            AutoFrame frame = new AutoFrame();
            frame.instruction = INST.AUTOALIGN;
            frame.AAHubSeen = false;
            return frame;
        }
        static public AutoFrame Wait(double seconds){
            AutoFrame frame = new AutoFrame();
            frame.instruction = INST.WAIT;
            frame.stopTime = seconds;
            return frame;
        }
        static public AutoFrame AutoRev(double x, double y)
        {
            AutoFrame frame = new AutoFrame();
            frame.instruction = INST.AUTOREV;
            frame.dist = Math.hypot(x, y)/12.0;
            return frame;
        }

        private AutoFrame() {} 

        INST instruction;

        double stopTime;
        
        double movedistance;
        Vector2D move;
        boolean intakeOn;
        double dist;

        double targetangle;

        boolean AAHubSeen;
    }
    
    private static boolean isDone = false;
    private static double[] start = new double[4];

    private static AutoFrame[] instructions;
    private static int index;

    private static boolean m_intakeOn = false;

    public static void init(){
        index = 0;
        isDone = false;
        m_intakeOn = false;
        //threeBall();
        backupAndShoot();
        //twoBallLeft();
        //basic();
        beginInstruction();
    }

    public static void basic(){
        instructions = new AutoFrame[]
        {            
            AutoFrame.AutoAlign(),
            AutoFrame.Shoot(),
        };
    }  

    public static void backupAndShoot(){
        Pigeon.setYaw(0);
        SwervePosition.setPosition(new Vector2D(0,-92));
        instructions = new AutoFrame[]{
            AutoFrame.RevForDist(0,-136),
            AutoFrame.MoveTo(0,-136),
            AutoFrame.Intake(true),
            AutoFrame.LookAt(0, 0),
            AutoFrame.AutoRev(0, -100),
            AutoFrame.Shoot(),
            AutoFrame.Intake(false)
        };
    }

    public static void twoBallLeft(){
        Pigeon.setYaw(140);
        SwervePosition.setPosition(new Vector2D(-44,-84));
        instructions = new AutoFrame[]{
            AutoFrame.MoveTo(-32, -72),
            AutoFrame.Wait(1),
            AutoFrame.Intake(true),
            AutoFrame.RevForDist(-85,-130),
            AutoFrame.MoveToAndLookAt(-85,-130),//Ball at -85,-130
            AutoFrame.Wait(1),
            AutoFrame.Intake(false),
            AutoFrame.LookAt(0, 0),
            AutoFrame.AutoRev(-85,-130),
            AutoFrame.Shoot(),
            AutoFrame.Wait(1),
            AutoFrame.AutoRev(-85,-130),
            AutoFrame.Shoot()
        };
    }

    public static void threeBall() {
        //NOT FINISHED!!!!
        Pigeon.setYaw(-90);
        SwervePosition.setPosition(new Vector2D(86, -39));
        instructions = new AutoFrame[]
        {
            AutoFrame.MoveTo(82, -39),
            AutoFrame.Wait(1),
            AutoFrame.Intake(true),
            //Ball at (150,-27)
            AutoFrame.MoveToAndLookAt(140, -39),
            AutoFrame.Wait(1),
            AutoFrame.Intake(false),
            AutoFrame.MoveTo(136, -140),
            //3rd ball at (87, -125)
            AutoFrame.RevForDist(120, -175),
            AutoFrame.MoveTo(120, -175),
            AutoFrame.LookAt(0, 0),
            AutoFrame.AutoRev(120, -175),
            AutoFrame.Shoot(),
            AutoFrame.Wait(1),
            AutoFrame.AutoRev(105, -158),
            AutoFrame.Shoot(),
            AutoFrame.Intake(true),
            AutoFrame.RevForDist(87, -125),
            AutoFrame.MoveTo(87, -125),
            AutoFrame.Intake(false),
            AutoFrame.LookAt(0, 0), 
            AutoFrame.AutoRev(87, -125),
            AutoFrame.Shoot(),
            //Human player at (118, -282)
            // AutoFrame.Intake(true),
            // AutoFrame.RevForDist(90, -275),
            // AutoFrame.MoveToAndLookAt(90, -275),
            // AutoFrame.LookAt(0,0),
            // AutoFrame.Shoot()

        };
    }

    public static void update(){
        if (isDone)
            return;
        double pow = 0;
        pow = Pigeon.correctTurnWithPID();
        Vector2D movement = new Vector2D(0,0);
        Intake.setEnabled(m_intakeOn);

        switch(instructions[index].instruction){
            case ROTATE:
                
            //if(pow == 0){
                if(Pigeon.atSetpoint())
                    nextInstruction();
                //} else {
                    //SwerveManager.rotateAndDrive(pow, new Vector2D());
                //}
                break;
            case MOVE:
                double avgDist = 0;
                for(int i = 0; i < 4; i++){
                    avgDist += Math.abs(start[i]-SwerveManager.getEncoderPos(i));
                }
                avgDist /= 4;
                if(avgDist >= instructions[index].movedistance){
                    nextInstruction();
                } else {
                }
                break;
            case MOVETOCOORD:
                //if(Math.hypot(instructions[index].move.x - SwervePosition.xPosition, instructions[index].move.y - SwervePosition.yPosition) < 12){
                if (instructions[index].move.sub(SwervePosition.getPosition()).mag() < 1.2) {
                    nextInstruction();
                }
                else {
                    Vector2D direction = instructions[index].move.sub(SwervePosition.getPosition()).norm();
                    movement = direction.mul(0.3);
                }
                break;
            case MOVETOANDLOOKAT:
                if (instructions[index].move.sub(SwervePosition.getPosition()).mag() < 1.2) {
                    nextInstruction();
                }
                else {
                    Vector2D direction = instructions[index].move.sub(SwervePosition.getPosition()).norm();
                    movement = direction.mul(0.4);
                    
                    Pigeon.setTargetAngle(direction.atanDeg());
                    //pow = -Pigeon.correctTurnWithPID();
                }
                break;
            case LOOKAT:
                Vector2D direction = instructions[index].move.sub(SwervePosition.getPosition()).norm();
                movement = new Vector2D(0,0);

                Pigeon.setTargetAngle(direction.atanDeg());
                //pow = -Pigeon.correctTurnWithPID();
                
                if(Pigeon.atSetpoint())
                    nextInstruction();
                break;
            case SHOOT:
                if(RTime.now() >= instructions[index].stopTime){
                    Shooter.stop();
                    nextInstruction();
                    return;
                }
                Shooter.fire();
                if(instructions[index].stopTime == Double.MAX_VALUE && Shooter.atSetpoint()){
                    instructions[index].stopTime = RTime.now() + 2;   
                    instructions[index].stopTime = RTime.now() + 1;   
                    System.out.println("stop time set");
                }
                break;
            case AUTOALIGN:
                if (AutoAlign.m_hubSeen && !instructions[index].AAHubSeen) {
                    instructions[index].AAHubSeen = true;
                    AutoAlign.setAngle();
                }
                if (instructions[index].AAHubSeen && Pigeon.atSetpoint())
                    nextInstruction();
                
                break;
            case WAIT:
                if (instructions[index].stopTime < RTime.now())
                    nextInstruction();
                break;
            default:
                break;
        }

        SwerveManager.rotateAndDrive(pow, movement);
    }

    private static void nextInstruction(){
        index++;
        System.out.println("next");
        if (instructions.length <= index){
            isDone = true;
            SwerveManager.rotateAndDrive(0, new Vector2D());
            return;
        }

        beginInstruction();
    }

    private static void beginInstruction() {
        
        switch(instructions[index].instruction){
            case ROTATE:
                Pigeon.setTargetAngle(instructions[index].targetangle);
                break;
            case MOVE:
                for(int i = 0; i < 4; i++){
                    start[i] = SwerveManager.getEncoderPos(i); 
                }
                break;
            case INTAKE:
                m_intakeOn = instructions[index].intakeOn;
                nextInstruction();
                break;
            case REVFORDIST:
                Shooter.setRPMForDist(instructions[index].dist);
                nextInstruction();
                break;
            case WAIT:
                instructions[index].stopTime += RTime.now();
                break;
            case AUTOREV:
                if (AutoAlign.m_hubSeen) 
                    Shooter.setRPMForDist(AutoAlign.m_distAvg);
                else
                    Shooter.setRPMForDist(instructions[index].dist);
                nextInstruction();
                break;
            default:
                break;
            
        }
    } 
}
