package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.globals;
import org.firstinspires.ftc.teamcode.subsystems.driveBase;

public class parkCMD extends CommandBase {
    private driveBase m_drive;

    private Pose2d drivePos;

    public parkCMD(driveBase drive){
        m_drive = drive;
        addRequirements(m_drive);


    }
    @Override
    public void initialize(){


    }
    @Override
    public void execute(){
        if(globals.location == globals.Location.LEFT) {
            drivePos = new Pose2d(40, -62, Math.toRadians(90));
            m_drive.goToPos(new Pose2d(40, -62, Math.toRadians(90)));
        } else{
            drivePos = new Pose2d(60, -62, Math.toRadians(90));
            m_drive.goToPos(new Pose2d(60, -62, Math.toRadians(90)));
        }




        m_drive.update();
    }
    public boolean isFinished(){
        Pose2d pos = m_drive.getPos();
        if(Math.abs(pos.getX()-drivePos.getX())<.25
        && Math.abs(pos.getY()-drivePos.getY())<.25
        && Math.abs(Math.toDegrees(pos.getHeading())-Math.toDegrees(drivePos.getHeading()))<2){
            return true;
        }
        return false;
    }
}
