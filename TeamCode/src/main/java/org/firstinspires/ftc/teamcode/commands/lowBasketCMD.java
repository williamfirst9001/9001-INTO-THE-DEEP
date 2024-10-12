package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.subsystems.elevator;

public class lowBasketCMD extends CommandBase {
    private elevator m_arm;

    public lowBasketCMD(elevator arm) {
        m_arm = arm;
        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        if(m_arm.getPivotPos()>800&&m_arm.getPivotPower()>0){
            m_arm.setPivotGains(constants.pivotConstants.top.P,constants.pivotConstants.top.I,constants.pivotConstants.top.D);
        }
        if(m_arm.getPivotPos()<800 && m_arm.getPivotPower()>0){
            m_arm.setPivotGains(constants.pivotConstants.up.P,constants.pivotConstants.up.I,constants.pivotConstants.up.D);
        }
        if(m_arm.getPivotPower()<-.25){
            m_arm.setPivotGains(constants.pivotConstants.down.P,constants.pivotConstants.down.I,constants.pivotConstants.down.D);
        }
    }
    @Override
    public void execute(){
        m_arm.goToSetpoint(constants.elevatorSetpoints.armSetpoints.lowBasket,constants.elevatorSetpoints.pivotSetpoints.basket);
        if(m_arm.getPivotPos()>800&&m_arm.getPivotPower()>0){
            m_arm.setPivotGains(constants.pivotConstants.top.P,constants.pivotConstants.top.I,constants.pivotConstants.top.D);
        }
        if(m_arm.getPivotPos()<800 && m_arm.getPivotPower()>0){
            m_arm.setPivotGains(constants.pivotConstants.up.P,constants.pivotConstants.up.I,constants.pivotConstants.up.D);
        }
        if(m_arm.getPivotPower()<-.25){
            m_arm.setPivotGains(constants.pivotConstants.down.P,constants.pivotConstants.down.I,constants.pivotConstants.down.D);
        }
    }
    @Override
    public boolean isFinished(){
        return m_arm.isDone();
    }
}
