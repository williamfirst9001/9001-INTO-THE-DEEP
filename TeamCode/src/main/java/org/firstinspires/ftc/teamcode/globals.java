package org.firstinspires.ftc.teamcode;

public class globals {
    public static boolean hardwareInit = false;
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
    public static ClawState clawState = ClawState.CLOSED;
    public static void toggleClaw(){
        if(clawState == ClawState.CLOSED){
            clawState = ClawState.OPEN;
        } else if(clawState == ClawState.OPEN){
            clawState = ClawState.CLOSED;
        }
    }
}
