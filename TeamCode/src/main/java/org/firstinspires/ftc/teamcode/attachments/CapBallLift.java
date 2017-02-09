package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.DriverStation;
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

    DriverStation.DSGamePad liftStick, forkFoldKey, forkUnfoldKey, liftLockKey, liftUnlockKey;


    public CapBallLift(FTCRobot robot, LinearOpMode curOpMode, JSONObject rootObj) {
        String key;
        JSONObject liftObj = null;
        JSONObject motorsObj = null, liftMotorObj = null, liftServoObj=null, dsCmdsObj=null;

        this.robot = robot;
        this.curOpMode = curOpMode;
        try {
            liftObj = JsonReader.getJsonObject(rootObj, "CapBallLift");
            motorsObj = JsonReader.getJsonObject(liftObj, "motors");
            liftMotorObj = JsonReader.getJsonObject(motorsObj, "liftMotor");
            liftMotor = curOpMode.hardwareMap.dcMotor.get("liftMotor");
            if (liftMotorObj.getBoolean("needReverse")) {
                DbgLog.msg("ftc9773: Reversing the lift servo");
                liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double maxSpeed = liftMotorObj.getDouble("maxSpeed");
            liftMotor.setMaxSpeed((int)(liftMotor.getMaxSpeed() * maxSpeed));

            liftServoObj = JsonReader.getJsonObject(motorsObj, "liftServo");
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

            // Initilize the Driver Station commands
            dsCmdsObj = JsonReader.getJsonObject(liftObj, "DScommands");
            liftStick = robot.drvrStation.StringToGamepadID(dsCmdsObj.getString("raise_lower"));
            forkFoldKey = robot.drvrStation.StringToGamepadID(dsCmdsObj.getString("fold_fork"));
            forkUnfoldKey = robot.drvrStation.StringToGamepadID(dsCmdsObj.getString("unfold_fork"));
            liftLockKey = robot.drvrStation.StringToGamepadID(dsCmdsObj.getString("lock_lift"));
            liftUnlockKey = robot.drvrStation.StringToGamepadID(dsCmdsObj.getString("unlock_lift"));
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

//        power = -curOpMode.gamepad2.right_stick_y;
        power = -1 * robot.drvrStation.getFloat(liftStick);
        applyPower(power);

//        if(curOpMode.gamepad2.right_bumper){
        if (robot.drvrStation.getBoolean(liftLockKey)) {
            lockLiftMotor();
        }
//        if (curOpMode.gamepad2.left_bumper){
        if (robot.drvrStation.getBoolean(liftUnlockKey)) {
            unlockLiftMotor();
        }


        if (liftServoCR != null) {
//            if (liftServoCR!= null && curOpMode.gamepad2.a) {
            if (liftServoCR!= null && robot.drvrStation.getBoolean(forkUnfoldKey)) {
                autoPlacement();
//            } else if (liftServoCR !=null && curOpMode.gamepad2.y) {
            } else if (liftServoCR !=null && robot.drvrStation.getBoolean(forkFoldKey)) {
                    foldFork();
            } else {
                idleFork();
            }
        }
    }
}
