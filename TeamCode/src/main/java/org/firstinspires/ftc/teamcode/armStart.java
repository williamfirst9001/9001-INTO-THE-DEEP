package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.elevator;

public class armStart {
    static robotHardware robot = robotHardware.getInstance();
    static elevator arm = new elevator();
    static Wrist wrist = new Wrist();
    private static boolean zeroed = false;
    private static boolean run = true;
    public static void start(){
        if(!zeroed && run){

        }
        if(robot.armSwitch.isPressed() && !zeroed && run){
            robot.eMotors.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.eMotors.resetEncoder();
            zeroed = true;
        }
        if(zeroed&& run){
            robot.eMotors.setPower(-.1);
        }


    }
    public static void reset(){
        zeroed = false;
        run = true;
    }
    public static void stop(){
        robot.eMotors.setPower(0);
        run = false;
        robot.eMotors.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.eMotors.resetEncoder();
    }
}
