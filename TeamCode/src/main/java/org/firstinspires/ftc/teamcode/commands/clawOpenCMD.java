package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.globals;
import org.firstinspires.ftc.teamcode.subsystems.claw;

public class clawOpenCMD extends CommandBase {
    private claw m_claw;
   private NanoClock clock = NanoClock.system();
   private double CMDStart;
   private double runTime;
   private globals.ClawState m_clawState;

    public clawOpenCMD(claw Claw){
        m_claw = Claw;
        addRequirements(m_claw);
        CMDStart = clock.seconds();
    }
    @Override
    public void initialize(){
            m_claw.open();
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
