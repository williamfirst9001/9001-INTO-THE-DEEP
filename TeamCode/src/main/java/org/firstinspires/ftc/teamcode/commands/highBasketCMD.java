package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.subsystems.elevator;

public class highBasketCMD extends CommandBase {
    private elevator m_arm;

    public highBasketCMD(elevator arm) {
        m_arm = arm;
        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        m_arm.goToSetpoint(constants.elevatorSetpoints.armSetpoints.highBasket,constants.elevatorSetpoints.pivotSetpoints.basket);

        m_arm.setPivotGains(constants.pivotConstants.P,constants.pivotConstants.I,constants.pivotConstants.D);


    }
    @Override
    public void execute(){

            m_arm.goToSetpoint(constants.elevatorSetpoints.armSetpoints.highBasket, constants.elevatorSetpoints.pivotSetpoints.basket);



    }
    /**
    @Override
    public boolean isFinished(){
        return m_arm.isDone();
    }
    **/
}
