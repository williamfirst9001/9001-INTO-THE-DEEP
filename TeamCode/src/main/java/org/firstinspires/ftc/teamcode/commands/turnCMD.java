package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.driveBase;

public class turnCMD extends CommandBase {
    private driveBase m_drive;
    private double turn;
    private double startAngle;

    public turnCMD(driveBase drive,double angle){
        m_drive = drive;
        addRequirements(m_drive);
        turn = angle;
        startAngle = m_drive.getPos().getHeading();
    }
    @Override
    public void initialize(){

    }
    @Override
    public void execute(){
        m_drive.turn(turn);
        m_drive.update();
    }
    public boolean isFinished(){
        Pose2d velo = m_drive.getVelo();
        if(velo.getHeading()<.1){
            return true;
        }
        return false;
    }
}
