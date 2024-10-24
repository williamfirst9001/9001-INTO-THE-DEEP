package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.elevator;

public class armMoveCMD extends CommandBase {
    private elevator m_arm;
    private double armPoint,pivotPoint,wristPoint;
    private boolean end;
    private Wrist m_wrist = null;

    public armMoveCMD(elevator arm,double armP,double pivotP) {
        m_arm = arm;
        addRequirements(m_arm);
        armPoint = armP;
        pivotPoint = pivotP;
    }
    public armMoveCMD(elevator arm,Wrist wrist,double armP,double pivotP, double wristP) {
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
        m_arm.goToPivotPoint(pivotPoint);
        if(m_arm.pivotDone()){
            m_arm.goToSetpoint(armPoint,pivotPoint);
        }
    }




}
