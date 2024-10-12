package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.robotHardware;

import org.firstinspires.ftc.teamcode.storage;

import static org.firstinspires.ftc.teamcode.constants.armConstants.middle.*;

public class elevator extends SubsystemBase {

    private PIDController elevatorPID = new PIDController(constants.armConstants.middle.P, constants.armConstants.middle.I, constants.armConstants.middle.D);

    private PIDController pivotPID = new PIDController(constants.pivotConstants.up.P, constants.pivotConstants.up.I, constants.pivotConstants.up.D);



    private robotHardware robot = robotHardware.getInstance();
    public elevator() {

        elevatorPID.setTolerance(150);
        pivotPID.setTolerance(2);

    }

    public boolean isDone(){
        return Math.abs(robot.pivotMotor.getCurrentPosition()-pivotPID.getSetPoint())<25;
    }
    public void goToSetpoint(double armPoint,double pivotPoint) {
        elevatorPID.setSetPoint(armPoint);
        pivotPID.setSetPoint(pivotPoint);

        robot.elevatorMotor.setPower(elevatorPID.calculate(robot.elevatorMotor.getCurrentPosition()));
        robot.pivotMotor.setPower(pivotPID.calculate(robot.pivotMotor.getCurrentPosition()));

    }
    public int getPivotPos(){
        return robot.pivotMotor.getCurrentPosition();
    }
    public double getPivotPower(){
        return robot.pivotMotor.getPower();
    }
    public void setElevatorGains(double P, double I,double D){
        elevatorPID.setPID(P,I,D);
    }
    public void setPivotGains(double P, double I,double D){
        pivotPID.setPID(P,I,D);
    }

    }