package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.teamcode.constants.autoGetPoints.*;
import static org.firstinspires.ftc.teamcode.PID.pivotPID.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PID.pivotPID;
@TeleOp(name="pivotTuning", group="Linear OpMode")
public class pivotTuning extends LinearOpMode {

    private DcMotorEx pivot;
    private PIDController PID = new PIDController(uP,uI,uD);
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode(){
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");

        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.x){
                PID.setSetPoint(800);
            }
            if(gamepad1.y){
                PID.setSetPoint(0);
            }
            if(PID.getSetPoint()==0){
                PID.setPID(dP,dI,dD);
            }
            if(PID.getSetPoint()==800){
                PID.setPID(uP,uI,uD);
            }

            pivot.setPower(PID.calculate(pivot.getCurrentPosition()));

            telemetry.addData("targetPos", PID.getSetPoint());
            telemetry.addData("pos",pivot.getCurrentPosition());
            telemetry.addData("motorPower",PID.calculate(pivot.getCurrentPosition()));

            telemetry.update();




        }
    }
}
