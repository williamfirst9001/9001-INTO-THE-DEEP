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
import org.firstinspires.ftc.teamcode.constants;

@TeleOp(name="pivotTuning", group="Linear OpMode")
public class pivotTuning extends LinearOpMode {
    private DcMotorEx elevatorMotor;
    private DcMotorEx pivot;
    private PIDController PID = new PIDController(P,I,D);
    private PIDController elevatorPID = new PIDController(constants.armConstants.middle.P, constants.armConstants.middle.I, constants.armConstants.middle.D);
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode(){
        elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevator");
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");

        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elevatorMotor.setDirection(DcMotor.Direction.REVERSE);
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while(opModeIsActive()){
                elevatorPID.setSetPoint(2100);
                PID.setSetPoint(1200);
                PID.setPID(P,I,D);




            pivot.setPower(PID.calculate(pivot.getCurrentPosition()));
            elevatorMotor.setPower(elevatorPID.calculate(elevatorMotor.getCurrentPosition()));

            telemetry.addData("targetPos", PID.getSetPoint());
            telemetry.addData("pos",pivot.getCurrentPosition());
            telemetry.addData("motorPower",PID.calculate(pivot.getCurrentPosition()));

            telemetry.update();




        }
    }
}
