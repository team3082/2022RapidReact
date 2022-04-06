package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Climber {

    private static TalonFX hookMotor;
    private static TalonSRX screwMotor;

    private static boolean climbing;

    static NetworkTable nt;

    static double pigVel;
    static double _pigPos;

    //HOOK: +: Down, -: Up
    //Tilt: +: In, -: Out
        
    public static void init(){
        nt = NetworkTableInstance.getDefault().getTable("climb");
        hookMotor = new TalonFX(0);
        hookMotor.configFactoryDefault();
        hookMotor.setInverted(false);
        hookMotor.setNeutralMode(NeutralMode.Brake);
        hookMotor.config_kF(0, 0, 30);
        hookMotor.config_kP(0, 0.1, 30);
        hookMotor.config_kI(0, 0, 30);
        hookMotor.config_kD(0, 0.05, 30);
        
        screwMotor = new TalonSRX(12);
        screwMotor.configFactoryDefault();
        screwMotor.setInverted(true);
        screwMotor.setNeutralMode(NeutralMode.Brake);
        
        Climber.zero();
    }
    
    public static double hookPos;
    public static double screwPos;
    
    public static void update(){
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
        return climbing;
    }
    
    public static boolean isLimitHit(){
        return screwMotor.getSensorCollection().isFwdLimitSwitchClosed() || screwMotor.getSensorCollection().isRevLimitSwitchClosed();
    }
    
    public static void startClimb(){
        step = -1;
        t = 0;
        climbing = true;
        setHook(0);
        setScrew(0);
    }
    
    public static void stopClimb(){
        climbing = false;
        setHook(0);
        setScrew(0);
    }
    
    //4 controlling motors:
    public static void setHook(double pow){
        if(isLimitHit())
        hookMotor.set(ControlMode.PercentOutput, Math.min(pow,0));
        else
            hookMotor.set(ControlMode.PercentOutput, pow);
        }
        public static void zero(){
            hookMotor.setSelectedSensorPosition(0);
            screwMotor.setSelectedSensorPosition(0);
        }
        
        public static void setScrew(double pow){
            screwMotor.set(ControlMode.PercentOutput, pow);
    }
    
    static int step;
    static double encoderDest;
    static int t = 0;
    
    //Auto climbing
    //Called in OI
    public static void climb(){
        //Hook should start at up position
        switch (step){
            //Drive main hook down until it hits the limit
            //While driving down, also start to extend the screw
            case -1:
                setHook(hookPos>-450000?0.25:1);
                if(screwPos > 5400){
                    setScrew(0);
                    if(isLimitHit()){
                        encoderDest = hookPos-950000;//Full Extension
                        step++;
                    }
                } else if(t>75){
                    setScrew(-0.4);
                }
                break;
            case 0:
                if(hookPos <= encoderDest+800000){
                    setHook(0);
                    step++;
                } else {
                    setHook(-0.25);
                }
                break;
            //EXTEND HOOK
            case 1:
                if(hookPos <= encoderDest){
                    setHook(0);
                    encoderDest+=135000;//Hook on bar
                    step++;
                } else {
                    setHook(-1);
                }
                break;
            //PUT HOOK IN POSITION TO GRAB
            case 2:
                if(screwPos <= 2600){
                    setScrew(0);
                    step++;
                } else {
                    setScrew(0.5);
                }
                break;
            //PUT HOOK OVER BAR
            case 3:
                if(hookPos >= encoderDest){
                    setHook(0);
                    step++;
                } else {
                    setHook(1);
                }
                break;
            //SWING OUT BOTTOM OF ROBOT TO ALIGN CENTER OF MASS
            case 4:
                if(screwPos >= 27552){
                    setScrew(0);
                    step++;
                } else {
                    setScrew(-1);
                }
                break;
            case 5:
                setHook(hookPos>-450000?0.25:1);
                if(screwPos <= 5400){
                    setScrew(0);
                    if(isLimitHit()){
                        encoderDest = hookPos-950000;//Full Extension
                        step++;
                    }
                } else {
                    setScrew(1);
                }
                break;
            //EXTEND HOOK
            case 6:
                if(hookPos <= encoderDest+800000){
                    setHook(0);
                    step++;
                } else {
                    setHook(-0.25);
                }
                break;
            case 7:
                if(hookPos <= encoderDest){
                    setHook(0);
                    encoderDest+=300000;//Hook past bar
                    step++;
                } else {
                    setHook(-1);
                }
                break;
            //PUT HOOK IN POSITION TO GRAB
            case 8:
                if(screwPos <= 2600){
                    setScrew(0);
                    step++;
                } else {
                    setScrew(0.5);
                }
                break;
            //PULL ONT LAST BAR
            case 9:
                if(hookPos >= encoderDest){
                    setHook(0);
                    step++;
                } else {
                    setHook(0.5);
                }
                break;
            default:
                setHook(0);
                setScrew(0);
                return;
        }
    }
}

