package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.driveBase;

public class turnCMD extends CommandBase {
    private driveBase m_drive;
    private double turn;

    public turnCMD(driveBase drive,double angle){
        m_drive = drive;
        addRequirements(m_drive);
        turn = angle;
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
        if(Math.abs(Math.toDegrees(turn)-Math.toDegrees(m_drive.getPos().getHeading()))<2){
            return true;
        }
        return false;
    }
}
