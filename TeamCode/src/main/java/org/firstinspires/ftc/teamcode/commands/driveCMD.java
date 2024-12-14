package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.driveBase;
import org.firstinspires.ftc.teamcode.subsystems.elevator;

public class driveCMD extends CommandBase {
    private driveBase m_drive;
    private Pose2d drivePos;
    private Pose2d endPos;
    private elevator m_arm = null;

    public driveCMD(driveBase drive, Pose2d goal){
        m_drive = drive;
        addRequirements(m_drive);
        drivePos = m_drive.getPos();
        endPos = goal;
        m_arm = null;
    }
    public driveCMD(driveBase drive, elevator arm, Pose2d goal){
        m_drive = drive;
        m_arm = arm;
        addRequirements(m_drive,m_arm);
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
        if(m_arm!=null){
            m_arm.update();
        }
    }
    @Override
    public boolean isFinished(){
        Pose2d pos = m_drive.getPos();
        return Math.abs(pos.getX() - endPos.getX()) < 1 &&
                Math.abs(pos.getY() - endPos.getY()) < 1 &&
                Math.abs(Math.toDegrees(pos.getHeading()) - Math.toDegrees(endPos.getHeading())) < 10;
    }
}
