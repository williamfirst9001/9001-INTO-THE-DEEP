package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.teamcode.constants.autoGetPoints.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.roadRunner.startup.util.Encoder;
@TeleOp(name="deadwheel test", group="Linear OpMode")
public class deadwheelTest extends LinearOpMode {
    private Encoder rightEncoder,leftEncoder,frontEncoder;
    public void runOpMode(){
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontRight"));
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontLeft"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backRight"));
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("right",rightEncoder.getCurrentPosition());
            telemetry.addData("left",leftEncoder.getCurrentPosition());
            telemetry.addData("front",frontEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
