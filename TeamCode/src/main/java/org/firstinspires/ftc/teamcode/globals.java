package org.firstinspires.ftc.teamcode;

public class globals {
    public static boolean hardwareInit = false;
    public static boolean manualArm = false;
    public static boolean wristPickup = false;
    public static boolean armUpdated = false;
    public enum Location{
        LEFT,
        RIGHT
    }
    public static Location location = Location.LEFT;
    public static void setLocation(Location loc){
        location = loc;
    }
    public enum ClawState{
        OPEN,
        CLOSED
    }
    public enum ledState{
        TEST1,
        TEST2
    }
    public static ClawState clawState = ClawState.CLOSED;
    public static void openClaw(){
            clawState = ClawState.OPEN;
        }
        public static void closeClaw(){
            clawState = ClawState.CLOSED;
        }
    public static void toggleClaw(){
        if(clawState == ClawState.CLOSED){
            clawState = ClawState.OPEN;
        } else if(clawState == ClawState.OPEN){
            clawState = ClawState.CLOSED;
        }
    }
    public enum armVal{
        HIGH_BASKET,
        LOW_BASKET,
        STOW,
        PICKUPHIGH,
        PICKUPLOW,
        SAMPLE1PICKUP,
        SAMPLE2PICKUP,
        SAMPLE3PICKUP,
        HIGH_STOW
    }
    public enum armPos{
        HIGH_BASKET,
        LOW_BASKET,
        PICKUP,
        STOW,
        HIGH_STOW
    }

    }


