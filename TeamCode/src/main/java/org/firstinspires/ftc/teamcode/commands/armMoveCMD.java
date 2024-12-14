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
        m_arm.setArmVal(type);

    }
    public armMoveCMD(elevator arm, Wrist wrist, globals.armVal type, globals.forceArm forcearm){
        m_arm = arm;
        m_wrist = wrist;
        addRequirements(m_arm,m_wrist);
        m_type = type;
        tune = false;
        m_arm.setArmVal(type);

    }




    @Override
    public void initialize() {
        m_wrist.setStartTime(clock.seconds());
        if(!tune) {
            m_arm.setSetPoint(constants.points.map.get(m_type));
            m_wrist.move(constants.points.map.get(m_type));
        } else{
            m_arm.setSetPoint(vals);
            m_wrist.move(vals);
        }
        m_arm.setArmVal(m_type);
        globals.armUpdated = true;




        m_arm.setPivotGains(constants.pivotConstants.P,constants.pivotConstants.I,constants.pivotConstants.D);


    }
    @Override
    public void execute(){
        if(m_wrist!= null){
            m_wrist.setSetPoint(wristPoint);
        }
        m_arm.update();


    }
    @Override
    public boolean isFinished(){
        return m_arm.isDone();
    }






}
