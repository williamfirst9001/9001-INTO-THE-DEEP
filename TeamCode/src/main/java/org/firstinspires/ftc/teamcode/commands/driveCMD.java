package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.driveBase;

public class driveCMD extends CommandBase {
    private driveBase m_drive;
    private Pose2d drivePos;

    public driveCMD(driveBase drive, Pose2d pos){
        m_drive = drive;
        addRequirements(m_drive);
        drivePos = pos;
    }
    @Override
    public void initialize(){

    }
    @Override
    public void execute(){
        m_drive.goToPos(drivePos);
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
