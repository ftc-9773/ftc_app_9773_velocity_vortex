package org.firstinspires.ftc.teamcode;

/**
 * Created by Kids on 9/23/2016.
 */

import org.firstinspires.ftc.teamcode.util.JsonReadingClass;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TestOp", group = "TeleOp")

/*Luke edited Pranav's sample TeleOp on 9/23/2016*/

public class TestOp extends LinearOpMode{
    DcMotor motor1 = null;
    DcMotor motor2 = null;

    public void runOpMode() throws InterruptedException{
        /*motor1 = hardwareMap.dcMotor.get("fMotorL");
        motor2 = hardwareMap.dcMotor.get("fMotorR");*/
        JsonReadingClass reader = new JsonReadingClass("/sdcard/FIRST/lineFollow.json");

        waitForStart();
        while (opModeIsActive()){
            reader.vevoId();

            /*if(gamepad1.a){
                if(gamepad1.y){
                    motor1.setPower(0);
                }
                else {
                    motor1.setPower(1);
                }
            } else if(gamepad1.y && !gamepad1.a){
                motor1.setPower(-1);
            } else if(gamepad1.x){
                if(gamepad1.b){
                    motor2.setPower(0);
                }
                else {
                    motor2.setPower(-1);
                }
            } else if(gamepad1.b && !gamepad1.x){
                motor2.setPower(1);
            }*/

            idle();
        }
    }
}