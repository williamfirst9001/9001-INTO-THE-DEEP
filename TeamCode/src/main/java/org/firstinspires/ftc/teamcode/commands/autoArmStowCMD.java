package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.elevator;

public class autoArmStowCMD extends CommandBase {
    private elevator m_arm;
    private Wrist m_wrist = null;
    private double armPoint;
    private double pivotPoint;
    private double wristPoint;
    private boolean alreadyDone;
    public autoArmStowCMD(elevator arm, double armP, double pivotP) {
        m_arm = arm;
        addRequirements(m_arm);
        armPoint = armP;
        pivotPoint = pivotP;
    }
    public autoArmStowCMD(elevator arm, Wrist wrist, double armP, double pivotP, double wristP,boolean alreadyDone) {
        m_arm = arm;
        m_wrist = wrist;
        addRequirements(m_arm,m_wrist);
        armPoint = armP;
        pivotPoint = pivotP;
        wristPoint = wristP;
        this.alreadyDone  = alreadyDone;
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
        if(!alreadyDone) {
            m_arm.goToSetpoint(0);
            if (m_arm.armDone())
                m_arm.goToSetpoint(0, 100);
        }
        else{
            m_arm.goToSetpoint(0,100);
        }
    }


}
