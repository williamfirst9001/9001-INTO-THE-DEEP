package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.elevator;

public class autoArmMoveCMD extends CommandBase {
    private elevator m_arm;
    private Wrist m_wrist = null;
    private double armPoint;
    private double pivotPoint;
    private double wristPoint;
    public autoArmMoveCMD(elevator arm, double armP, double pivotP) {
        m_arm = arm;
        addRequirements(m_arm);
        armPoint = armP;
        pivotPoint = pivotP;
    }
    public autoArmMoveCMD(elevator arm, Wrist wrist, double armP, double pivotP, double wristP) {
        m_arm = arm;
        m_wrist = wrist;
        addRequirements(m_arm,m_wrist);
        armPoint = armP;
        pivotPoint = pivotP;
        wristPoint = wristP;
    }



    @Override
    public void initialize() {
        m_arm.setPivotGains(constants.pivotConstants.P,constants.pivotConstants.I,constants.pivotConstants.D);

    }
    @Override
    public void execute(){
        if(m_wrist!= null){
            m_wrist.move(wristPoint);
        }
        m_arm.setSetPoint(armPoint,pivotPoint);
    }

    @Override
    public boolean isFinished(){
        return m_arm.isDone();
    }
}
