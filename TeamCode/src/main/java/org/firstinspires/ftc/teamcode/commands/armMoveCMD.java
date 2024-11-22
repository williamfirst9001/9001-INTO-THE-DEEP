package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.globals;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.elevator;

import java.util.List;

public class armMoveCMD extends CommandBase {
    private elevator m_arm;
    private static double armPoint,pivotPoint,wristPoint;
    private boolean end;
    private Wrist m_wrist = null;
    private NanoClock clock = NanoClock.system();
    private boolean tune = false;
    private List<Double> vals;
    globals.armVal m_type;



    public armMoveCMD(elevator arm, Wrist wrist, globals.armVal type){
        m_arm = arm;
        m_wrist = wrist;
        addRequirements(m_arm,m_wrist);
        m_type = type;
        tune = false;
    }




    @Override
    public void initialize() {
        m_wrist.setStartTime(clock.seconds());
        m_arm.setState(elevator.armState.AUTO);

            m_arm.setSetPoint(constants.points.map.get(m_type));
            m_wrist.move(constants.points.map.get(m_type));



        m_arm.setPivotGains(constants.pivotConstants.P,constants.pivotConstants.I,constants.pivotConstants.D);


    }
    @Override
    public void execute(){
        m_arm.update();

            if(m_arm.pivotDone()) {
                if (m_wrist != null) {
                    m_wrist.setSetPoint(wristPoint);
                }
            }



    }
    @Override
    public boolean isFinished(){
        return m_arm.isDone() && clock.seconds()- m_wrist.getStartTime()>.5;
    }




}
