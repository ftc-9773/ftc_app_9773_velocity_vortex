package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.json.JSONException;
import org.json.JSONObject;


/*
 * Copyright (c) 2016 Robocracy 9773
 */

public class CapBallLift implements  Attachment {
    FTCRobot robot;
    LinearOpMode curOpMode;
    DcMotor liftMotor;
    CRServo liftServoCR = null;
    Servo liftServo = null;
    boolean lockLift = false;


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
                DbgLog.msg("ftc9773: Reversing the lift servo");
                liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double maxSpeed = liftMotorObj.getDouble("maxSpeed");
            liftMotor.setMaxSpeed((int)(liftMotor.getMaxSpeed() * maxSpeed));

            key = JsonReader.getRealKeyIgnoreCase(motorsObj, "liftServo");
            liftServoObj = motorsObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(liftServoObj, "motorType");
            String motorType = liftServoObj.getString(key);
            if (motorType.equalsIgnoreCase("CRservo")) {
                liftServoCR = curOpMode.hardwareMap.crservo.get("liftServo");
            } else {
                liftServo = curOpMode.hardwareMap.servo.get("liftServo");
                liftServo.scaleRange(liftServoObj.getDouble("scaleRangeMin"),
                        liftServoObj.getDouble("scaleRangeMax"));
                if (liftServoObj.getBoolean("needReverse")) {
                    DbgLog.msg("ftc9773: Reversing the lift servo");
                    liftServo.setDirection(Servo.Direction.REVERSE);
                }
                liftServo.setPosition(1);
            }
            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        } catch (JSONException e) {
            e.printStackTrace();
        }
    }

    public void autoPlacement(){
        //Unfolding
        unfoldFork();
        curOpMode.sleep(500);
        idleFork();
        //raising
        applyPower(1);
        curOpMode.sleep(900);
        applyPower(0);
        //lowering
        applyPower(-1);
        curOpMode.sleep(900);
        applyPower(0);
    }

    public void applyPower(double power){
        if (!lockLift){
            liftMotor.setPower(power);
        }
        else if (lockLift){
            if(liftMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            liftMotor.setPower(1);
        }
    }
    public void lockLiftMotor(){
        lockLift = true;
        liftMotor.setTargetPosition(liftMotor.getCurrentPosition());
    }
    public void unlockLiftMotor(){
        lockLift = false;
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void unfoldFork(){
        liftServoCR.setPower(-1);
    }
    public void foldFork(){
        liftServoCR.setPower(1);
    }
    public void idleFork(){
        liftServoCR.setPower(0);
    }

    @Override
    public void getAndApplyDScmd() {
        float power;

        power = -curOpMode.gamepad2.right_stick_y;

        applyPower(power);

        if(curOpMode.gamepad2.right_bumper){
            lockLiftMotor();
        }
        if (curOpMode.gamepad2.left_bumper){
            unlockLiftMotor();
        }


        if (liftServoCR != null) {
            if (liftServoCR!= null && curOpMode.gamepad2.a) {
                autoPlacement();
            } else if (liftServoCR !=null && curOpMode.gamepad2.y) {
                foldFork();
            } else {
                idleFork();
            }
        }
    }
}
