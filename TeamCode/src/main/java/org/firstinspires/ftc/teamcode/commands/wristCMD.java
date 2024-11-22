package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.globals;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

import java.util.List;

public class wristCMD extends CommandBase {
    private Wrist m_wrist;
    private double wristPoint;
    private NanoClock clock = NanoClock.system();
    private double start;



    public wristCMD(Wrist wrist, globals.armVal type) {
        m_wrist = wrist;
        addRequirements(m_wrist);
        wristPoint = constants.points.map.get(type).get(2);
        start = clock.seconds();
    }


    @Override
    public void initialize() {
        m_wrist.move(wristPoint);


    }
    @Override
    public void execute(){
        m_wrist.move(wristPoint);
    }
    @Override
    public boolean isFinished(){
        return true;
    }



}
