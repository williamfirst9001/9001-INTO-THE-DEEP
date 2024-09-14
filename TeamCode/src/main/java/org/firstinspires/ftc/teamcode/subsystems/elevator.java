package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants;

public class elevator {
    private DcMotor elevatorMotor, pivotMotor;

    private PIDController elevatorPID = new PIDController(constants.armPID.middle.kP, constants.armPID.middle.kI, constants.armPID.middle.kD);

    private PIDController pivotPID = new PIDController(constants.pivotPID.kP, constants.pivotPID.kI, constants.pivotPID.kD);
    public elevator(HardwareMap hardwareMap) {

            elevatorMotor = hardwareMap.get(DcMotor.class, "elevator");
            pivotMotor = hardwareMap.get(DcMotor.class, "pivot");
            elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            elevatorPID.setTolerance(constants.armPID.tolerance);
            pivotPID.setTolerance(constants.pivotPID.tolerance);

    }
        public double pivotToAngle() {
            return pivotMotor.getCurrentPosition() / constants.armGearRatio.countsPerDegree;
        }

        public void update(int pivotPoint,int armPoint) {
            double pivotAngle = pivotMotor.getCurrentPosition() / constants.armGearRatio.countsPerDegree;

            

            //set the target position for the dc motor encoder
            elevatorMotor.setTargetPosition(armPoint);
            //sets the setpoint for PIDF controller
            elevatorPID.setSetPoint(armPoint);
            //gets motor powers from PIDF controller
            elevatorMotor.setPower(elevatorPID.calculate(elevatorMotor.getCurrentPosition()));
            //(int)Math.round(elevatorPID.calculate())

            double pivotPos = pivotMotor.getCurrentPosition();

            pivotPID.setSetPoint(pivotPoint);
            pivotMotor.setPower(pivotPID.calculate(pivotMotor.getCurrentPosition()));


        }


    }

