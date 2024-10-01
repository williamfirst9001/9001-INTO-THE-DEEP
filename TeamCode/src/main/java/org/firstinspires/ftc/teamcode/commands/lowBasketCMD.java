package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.robotContainer;
import org.firstinspires.ftc.teamcode.subsystems.elevator;

public class lowBasketCMD extends CommandBase {
    private elevator m_arm;

    public lowBasketCMD(elevator arm) {
        m_arm = arm;
        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        m_arm.setElevatorGains(constants.armConstants.baskets.kV,constants.armConstants.baskets.kA,constants.armConstants.baskets.kStatic);
    }
    @Override
    public void execute(){
        m_arm.goToSetpoint(constants.elevatorSetpoints.armSetpoints.lowBasket,constants.elevatorSetpoints.pivotSetpoints.basket);
    }
    @Override
    public boolean isFinished(){
        return m_arm.isDone();
    }
}
