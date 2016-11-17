package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.json.JSONException;
import org.json.JSONObject;


public class CapBallLift implements  Attachment {
    FTCRobot robot;
    LinearOpMode curOpMode;
    DcMotor liftMotor;
    Servo liftServo;


    public CapBallLift(FTCRobot robot, LinearOpMode curOpMode, JSONObject rootObj) {
        String key;
        JSONObject liftObj = null;
        JSONObject motorsObj = null, liftMotorObj = null, liftServoObj=null;

        this.robot = robot;
        this.curOpMode = curOpMode;
        try {
            key = JsonReader.getRealKeyIgnoreCase(rootObj, "CapBallLift");
            liftObj = rootObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(liftObj, "motors");
            motorsObj = liftObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(motorsObj, "liftMotor");
            liftMotorObj = motorsObj.getJSONObject(key);
            liftMotor = curOpMode.hardwareMap.dcMotor.get("liftMotor");
            if (liftMotorObj.getBoolean("needReverse")) {
                DbgLog.msg("Reversing the lift servo");
                liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double maxSpeed = liftMotorObj.getDouble("maxSpeed");
            liftMotor.setMaxSpeed((int)(liftMotor.getMaxSpeed() * maxSpeed));

            key = JsonReader.getRealKeyIgnoreCase(motorsObj, "liftServo");
            liftServoObj = motorsObj.getJSONObject(key);
            liftServo = curOpMode.hardwareMap.servo.get("liftServo");
            liftServo.scaleRange(liftServoObj.getDouble("scaleRangeMin"),
                    liftServoObj.getDouble("scaleRangeMax"));
        } catch (JSONException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void getAndApplyDScmd() {
        float power;


        if(curOpMode.gamepad1.right_trigger > 0){
            power = curOpMode.gamepad1.right_trigger;
        }
        else if(curOpMode.gamepad1.left_trigger > 0){
            power = -curOpMode.gamepad1.left_trigger;
        }
        else{
            power = 0;
        }
        liftMotor.setPower(power);
        if (curOpMode.gamepad1.left_bumper){
            liftServo.setPosition(0.0);
        }
        else if (curOpMode.gamepad1.right_bumper){
            liftServo.setPosition(1.0);
        }
    }
}
