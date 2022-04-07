package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.CANMap;
import frc.robot.robotmath.Vector2D;

public class Climber {

    private static final boolean kDisabled = false;

    private static TalonFX hookMotor;
    private static TalonSRX screwMotor;

    private static boolean climbing;

    static NetworkTable nt;

    static double pigVel;
    static double _pigPos;

    static enum ClimbStep {
        RETRACT_FULL,
        EXTEND_TO_SECONDARY_HOOK,
        EXTEND_FULL,
        POSITION_HOOK_TO_GRAB,
        RETRACT_OVER_BAR,
        SWING_OUT_BASE,
        RETRACT_FULL_2,
        EXTEND_TO_SECONDARY_HOOK_2,
        EXTEND_FULL_2,
        POSITION_HOOK_TO_GRAB_2,
        RETRACT_FINAL,
        COMPLETE,
    }

    //HOOK: +: Down, -: Up
    //Tilt: +: In, -: Out
        

    static void resetVars() {
        m_hookclimbstate = 0;
    }

    public static void init(){
        if(disabled()) return;

        nt = NetworkTableInstance.getDefault().getTable("climb");
        hookMotor = new TalonFX(CANMap.CLIMBER_HOOK);
        hookMotor.configFactoryDefault();
        hookMotor.setInverted(false);
        hookMotor.setNeutralMode(NeutralMode.Brake);
        hookMotor.config_kF(0, 0, 30);
        hookMotor.config_kP(0, 1, 30);
        hookMotor.config_kI(0, 0, 30);
        hookMotor.config_kD(0, 0.05, 30);
        
        screwMotor = new TalonSRX(CANMap.CLIMBER_SCREW);
//        screwMotor.configFactoryDefault();
        screwMotor.setInverted(true);
        screwMotor.setNeutralMode(NeutralMode.Brake);


        Climber.zero();

        resetVars();
    }


    
    public static double hookPos;
    public static double screwPos;
    
    public static void update(){
        if(disabled()) return;


        hookPos = hookMotor.getSelectedSensorPosition();
        screwPos = screwMotor.getSelectedSensorPosition();
        t++;
        if(t%5==0){
            double pigPos = Math.abs(Pigeon.getPitch());
            pigVel = Math.abs(Math.max(pigPos,_pigPos)-Math.min(pigPos,_pigPos));
            nt.getEntry("vel").setDouble(pigVel);
            _pigPos = pigPos;
        }
        nt.getEntry("ang").setDouble(_pigPos);
        nt.getEntry("1encoder").setDouble(hookPos);
        nt.getEntry("2encoder").setDouble(screwPos);
    }
    
    
    public static boolean isClimbing(){
        if(disabled()) return false;

        return climbing;
    }
    
    public static boolean isLimitHit(){
        if(disabled()) return false;
        return screwMotor.getSensorCollection().isRevLimitSwitchClosed() && screwMotor.getSensorCollection().isFwdLimitSwitchClosed();
    }
    
    public static boolean isLimitOpen(){
    	if(disabled()) return false;
        return !screwMotor.getSensorCollection().isRevLimitSwitchClosed() && !screwMotor.getSensorCollection().isFwdLimitSwitchClosed();
    }
    

    public static void startClimb(){
        if(disabled()) return;
        
        //Remove so we can resume
        //step = ClimbStep.RETRACT_FULL;
		t = 0;
        high = false;
        climbing = true;
        SwerveManager.rotateAndDrive(0, new Vector2D(0, 0));
        Intake.setEnabled(false);
        setHook(0);
        setScrew(0);
    }

    static boolean high = false;

    public static void startHighBarClimb(){
        if(disabled()) return;
        high = true;
        SwerveManager.rotateAndDrive(0, new Vector2D(0, 0));
        Intake.setEnabled(false);
        //Remove so we can resume
        //step = ClimbStep.RETRACT_FULL;
		t = 0;
        climbing = true;

        setHook(0);
        setScrew(0);
    }
    
    public static void stopClimb(){
        if(disabled()) return;
        climbing = false;
        setHook(0);
        setScrew(0);
    }
    
    //4 controlling motors:
    //TODO: fix limit here as well

    public static void setHook(double pow){
        if(disabled()) return;

        double pos = hookMotor.getSelectedSensorPosition();

        // DO NOT DRIVE INTO THE BASE OR TOP!!
//        if( (pow > 0 && pos >= -70000.0) || (pow < 0 && pos <= -930000.0))
 //           hookMotor.set(ControlMode.PercentOutput, 0);
  //      else
            hookMotor.set(ControlMode.PercentOutput, pow);
    }
    
    public static void zero(){
        if(disabled()) return;
        hookMotor.setSelectedSensorPosition(0);
        screwMotor.setSelectedSensorPosition(0);
    }
    
    public static void setScrew(double pow){
        if(disabled()) return;
        screwMotor.set(ControlMode.PercentOutput, pow);
    }
    
    static ClimbStep step = ClimbStep.RETRACT_FULL;
    static double encoderDest;
    static int t = 0;
    
    //Auto climbing
    //Called in OI

    //TODO: fix limit switches

    public static void gotoStart(){
        hookMotor.set(ControlMode.Position, -990000);
    }

    static int m_hookclimbstate = 0;
    static void climbPastHookComplete() {
        m_hookclimbstate = 0;
    }

    static boolean climbPastHook(double bigS) {

        final double slowspeed = 0.35;
        final double fastspeed = bigS;
        
        System.out.println("!"+m_hookclimbstate);
        if(m_hookclimbstate == 2)
            return true;


        if(m_hookclimbstate == 1) {
            // Limit switch was released
            if(isLimitHit()) {

                // Stop the motor
                setHook(0.0);

                // Limit switch was pressed again, we just moved past the hook
                m_hookclimbstate = 2;
                return true;
            } else {
                setHook(slowspeed);
            }
        } else {
            // We haven't released our limit switch yet. Wait until the hook tilts and the switch is released.
            if(isLimitOpen()) {
                m_hookclimbstate = 1;
                setHook(slowspeed);
            } else {
                setHook(fastspeed);
            }
        }
        return false;
    }

    public static void printStep() {
        String name = "??";

        switch (step){
            case RETRACT_FULL:               name = "RETRACT_FULL";             break; 
            case EXTEND_TO_SECONDARY_HOOK:   name = "EXTEND_TO_SECONDARY_HOOK"; break;
            case EXTEND_FULL:                name = "EXTEND_FULL";              break;
            case POSITION_HOOK_TO_GRAB:      name = "POSITION_HOOK_TO_GRAB";    break;
            case RETRACT_OVER_BAR:           name = "RETRACT_OVER_BAR";         break;
            case SWING_OUT_BASE:             name = "SWING_OUT_BASE";           break;
            case RETRACT_FULL_2:             name = "RETRACT_FULL_2";           break;
            case EXTEND_TO_SECONDARY_HOOK_2: name = "EXTEND_TO_SECONDARY_HOOK_2"; break;
            case EXTEND_FULL_2:              name = "EXTEND_FULL_2"; break;
            case POSITION_HOOK_TO_GRAB_2:    name = "POSITION_HOOK_TO_GRAB_2"; break;
            case RETRACT_FINAL:              name = "RETRACT_FINAL"; break;
            case COMPLETE:                   name = "COMPLETE"; break;
        }

        System.out.println(name);

    }
    
    public static void climb(){
        if(disabled()) return;
        printStep();

        //Hook should start at up position
        switch (step){
            //Drive main hook down until it hits the limit
            //While driving down, also start to extend the screw
            case RETRACT_FULL:

                boolean screwHit = screwPos > 2800.0;//5400;
                boolean hookHit = climbPastHook(t<150?0.5:1);

                if(screwHit){
                    setScrew(0);
                } else if(t>150){
                    setScrew(-0.4);
                }

                if(screwHit && hookHit) { 
                    climbPastHookComplete();

                    encoderDest = hookPos-850000;//950000;//Full Extension
                    step = ClimbStep.EXTEND_TO_SECONDARY_HOOK;
                } 

                /*
                setHook(isLimitHit()?1:0.25);
                if(screwPos > 5400){
                    setScrew(0);
                    if(isLimitHit()){
                        encoderDest = hookPos-950000;//Full Extension
                        step = ClimbStep.EXTEND_TO_SECONDARY_HOOK;
                    }
                } else if(t>75){
                    setScrew(-0.4);
                }
                */
                
                break;
            //Extend hook at a slow speed until the bar attaches to the secondary hooks
            case EXTEND_TO_SECONDARY_HOOK:
                if(hookPos <= encoderDest+740000/*800000*/){
                    setHook(0);
                    step = ClimbStep.EXTEND_FULL;
                } else {
                    setHook(-0.35);
                }
                break;
            //EXTEND HOOK
            case EXTEND_FULL:
                if(hookPos <= encoderDest){
                    setHook(0);
                    encoderDest+=124000/*135000*/; //Hook on bar
                    step = ClimbStep.POSITION_HOOK_TO_GRAB;
                } else {
                    setHook(-1);
                }
                break;
            //PUT HOOK IN POSITION TO GRAB
            case POSITION_HOOK_TO_GRAB:
                if(screwPos <= 600/*2600*/){
                    setScrew(0);
                    if(!high)
                        step = ClimbStep.RETRACT_OVER_BAR;
                    else {
                        step = ClimbStep.RETRACT_FINAL;
                        encoderDest+=300000;
                    }
                } else {
                    setScrew(1);
                }
                break;
            //PUT HOOK OVER BAR
            case RETRACT_OVER_BAR:
                if(hookPos >= encoderDest){
                    setHook(0);
                    step = ClimbStep.SWING_OUT_BASE;
                } else {
                    setHook(1);
                }
                break;
            //SWING OUT BOTTOM OF ROBOT TO ALIGN CENTER OF MASS
            case SWING_OUT_BASE:
                if(screwPos >= 20200){//20303){
                    setScrew(0);
                    step = ClimbStep.RETRACT_FULL_2;
                    m_hookclimbstate = 0;
                } else {
                    setScrew(-1);
                }
                break;
            //Stage 2- retract hook fully
            case RETRACT_FULL_2:

                boolean screwHit_2 = screwPos < 2800; //5400;
                boolean hookHit_2 = climbPastHook(0.75);

                if(screwHit_2){
                    setScrew(0);
                } else {
                    setScrew(1);
                }

                if(screwHit_2 && hookHit_2) { 
                    climbPastHookComplete();

                    encoderDest = hookPos-850000;//950000;//Full Extension
                    step = ClimbStep.EXTEND_TO_SECONDARY_HOOK_2;
                } 

                /*
                setHook(hookPos>-450000?0.25:1);
                if(screwPos <= 5400){
                    setScrew(0);
                    if(isLimitHit()){
                        encoderDest = hookPos-950000;//Full Extension
                        step = ClimbStep.EXTEND_TO_SECONDARY_HOOK_2;
                    }
                } else {
                    setScrew(1);
                }
                */
                break;
            //EXTEND HOOK
            case EXTEND_TO_SECONDARY_HOOK_2:
                if(hookPos <= encoderDest+740000/*800000*/){
                    setHook(0);
                    step = ClimbStep.EXTEND_FULL_2;
                } else {
                    setHook(-0.25);
                }
                break;
            case EXTEND_FULL_2:
                if(hookPos <= encoderDest){
                    setHook(0);
                    encoderDest+=300000;//124000/*300000*/;//Hook past bar
                    step = ClimbStep.POSITION_HOOK_TO_GRAB_2;
                } else {
                    setHook(-1);
                }
                break;
            //PUT HOOK IN POSITION TO GRAB
            case POSITION_HOOK_TO_GRAB_2:
                if(screwPos < 600/*2600*/){
                    setScrew(0);
                    step = ClimbStep.RETRACT_FINAL;
                } else {
                    setScrew(1);
                }
                break;
            //PULL ONTO LAST BAR
            case RETRACT_FINAL:
                if(hookPos >= encoderDest){
                    setHook(0);
                    step = ClimbStep.COMPLETE;
                } else {
                    setHook(0.5);
                }
                break;
            case COMPLETE:
                setHook(0);
                setScrew(0);
                return;
        }
    }


    

    private static int m_disablePrintCount = 0;
    private static boolean disabled() {
        if(kDisabled) {

            if(m_disablePrintCount++ < 20)
                System.out.println("ERROR: CLIMBER DISABLED!!!!!!!!");
            return true;
        }
        return false;
    }
}

