package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.driveBase;

public class driveCMD extends CommandBase {
    private driveBase m_drive;
    private Pose2d drivePos;
    private Pose2d endPos;

    public driveCMD(driveBase drive, Pose2d goal){
        m_drive = drive;
        addRequirements(m_drive);
        drivePos = m_drive.getPos();
        endPos = goal;
    }
    @Override
    public void initialize(){
        m_drive.goToPos(endPos);
    }
    @Override
    public void execute(){
        m_drive.update();
    }
    public boolean isFinished(){
        drivePos = m_drive.getPos();
        if(Math.abs(endPos.getX()-drivePos.getX())<.25
        && Math.abs(endPos.getY()-drivePos.getY())<.25
        && Math.abs(Math.toDegrees(endPos.getHeading())-Math.toDegrees(drivePos.getHeading()))<2){
            return true;
        }
        return false;
    }
}
