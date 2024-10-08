package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.globals;
import org.firstinspires.ftc.teamcode.subsystems.claw;

public class clawCMD extends CommandBase {
    private claw m_claw;

    public clawCMD(claw Claw){
        m_claw = Claw;
        addRequirements(m_claw);
    }
    @Override
    public void initialize(){
        globals.toggleClaw();
    }
    @Override
    public void execute(){

    }
    @Override
    public boolean isFinished(){
        return true;
    }
}
