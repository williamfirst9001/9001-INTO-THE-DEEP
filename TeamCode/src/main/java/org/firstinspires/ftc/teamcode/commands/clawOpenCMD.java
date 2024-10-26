package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.globals;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

public class clawOpenCMD extends CommandBase {
    private Claw m_claw;
   private NanoClock clock = NanoClock.system();
   private double CMDStart = 0;
   private double runTime =0;
   private globals.ClawState m_clawState;

    public clawOpenCMD(Claw Claw){
        m_claw = Claw;
        addRequirements(m_claw);
        CMDStart = clock.seconds();
    }
    @Override
    public void initialize(){
            m_claw.open();
        CMDStart = clock.seconds();
    }
    @Override
    public void execute(){
    runTime = clock.seconds()-CMDStart;
    }
    @Override
    public boolean isFinished(){
        return runTime>1.5;
    }

}
