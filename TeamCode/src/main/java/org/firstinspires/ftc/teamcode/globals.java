package org.firstinspires.ftc.teamcode;

public class globals {
    public static boolean hardwareInit = false;
    public enum Team{
        BLUE,
        RED,
        NOTINIT
    }
    public static Team team = Team.NOTINIT;
    public void setTeam(Team t){
        team = t;
    }
}
