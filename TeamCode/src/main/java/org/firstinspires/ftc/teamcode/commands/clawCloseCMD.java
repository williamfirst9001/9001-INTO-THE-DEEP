package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.globals;
import org.firstinspires.ftc.teamcode.subsystems.claw;

public class clawCloseCMD extends CommandBase {
    private claw m_claw;
   private NanoClock clock = NanoClock.system();
   private double CMDStart;
   private double runTime;
   private globals.ClawState m_clawState;

    public clawCloseCMD(claw Claw){
        m_claw = Claw;
        addRequirements(m_claw);
        CMDStart = clock.seconds();
    }
    @Override
    public void initialize(){
            m_claw.close();
    }
    @Override
    public void execute(){
    runTime = clock.seconds()-CMDStart;
    }
    @Override
    public boolean isFinished(){
        return runTime>.75;
    }

}
