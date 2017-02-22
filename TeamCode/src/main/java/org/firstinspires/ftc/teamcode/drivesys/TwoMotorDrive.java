package org.firstinspires.ftc.teamcode.drivesys;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.navigation.NavigationChecks;

/*
 * Copyright (c) 2016 Robocracy 9773
 */

public class TwoMotorDrive extends DriveSystem{
    DcMotor motorL = null;
    DcMotor motorR = null;
    double frictionCoefficient;
    int maxSpeedCPS; // encoder counts per second
    int motorLMaxSpeed, motorRMaxSpeed;
    Wheel wheel;
    int motorCPR;  // Cycles Per Revolution.  == 1120 for Neverest40
    boolean driveSysIsReversed = false;
    double distBetweenWheels;
    boolean LisZero, RisZero;
    ElapsedTime Ltimer, Rtimer;

    public class ElapsedEncoderCounts implements DriveSystem.ElapsedEncoderCounts {
        double encoderCountL;
        double encoderCountR;

        public ElapsedEncoderCounts() {
            encoderCountL = encoderCountR = 0;
        }

        public void reset() {
            encoderCountL = motorL.getCurrentPosition();
            encoderCountR = motorR.getCurrentPosition();
        }

        public double getDistanceTravelledInInches() {
            double avgEncoderCounts = 0.0;
            double distanceTravelled = 0.0;

            avgEncoderCounts = (Math.abs(motorL.getCurrentPosition() - encoderCountL) +
                    Math.abs(motorR.getCurrentPosition() - encoderCountR)) / 2;

            distanceTravelled = (avgEncoderCounts / motorCPR) * wheel.getCircumference();
            return (distanceTravelled);
        }

        public double getDegreesTurned() {
            double distanceTravelledInInches, degreesTurned;
            double leftDegreesTurned;

            distanceTravelledInInches = this.getDistanceTravelledInInches();
            degreesTurned = 360 * distanceTravelledInInches / (Math.PI * distBetweenWheels);
            leftDegreesTurned = motorL.getCurrentPosition() - encoderCountL;
            if (leftDegreesTurned < 0) {
                degreesTurned *= -1; // Negate the number to indicate counterclockwise spin
            }
            return (degreesTurned);
        }

        @Override
        public void printCurrentEncoderCounts() {
            DbgLog.msg("ftc9773: printCurrent...(): encoder counts: L=%d, R=%d",
                    motorL.getCurrentPosition(), motorR.getCurrentPosition());
        }
    }

    public TwoMotorDrive(DcMotor motorL, DcMotor motorR, int maxSpeedCPS,
                         double frictionCoefficient, Wheel wheel, int motorCPR){
        this.motorL = motorL;
        this.motorR = motorR;
        motorR.setDirection(DcMotorSimple.Direction.REVERSE);
        this.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frictionCoefficient = frictionCoefficient;
        this.maxSpeedCPS = maxSpeedCPS;
        DbgLog.msg("ftc9773: max speed CPS = %d", maxSpeedCPS);
        motorL.setMaxSpeed(maxSpeedCPS);
        motorR.setMaxSpeed(maxSpeedCPS);
        this.wheel = wheel;
        this.motorCPR = motorCPR;
        this.LisZero = this.RisZero =  true;
        this.Ltimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.Rtimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        Ltimer.reset();
        Rtimer.reset();
    }

    @Override
    public void drive(float speed, float direction){
        double left = (speed - direction) * frictionCoefficient;
        double right = (speed + direction) * frictionCoefficient;

        motorL.setPower(left);
        motorR.setPower(right);
    }

    @Override
    public void setZeroPowerMode(DcMotor.ZeroPowerBehavior zp_behavior) {
        motorL.setZeroPowerBehavior(zp_behavior);
        motorR.setZeroPowerBehavior(zp_behavior);
    }

    @Override
    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return (motorL.getZeroPowerBehavior());
    }

    @Override
    public void turnOrSpin(double leftSpeed, double rightSpeed) {
        motorL.setPower(leftSpeed);
        motorR.setPower(rightSpeed);
    }

    @Override
    public void stop() {
        motorL.setPower(0.0);
        motorR.setPower(0.0);
    }

    @Override
    public void turnDegrees(double degrees, float speed, NavigationChecks navExc) {
        double distInInches = (Math.abs(degrees) / 360) * Math.PI * this.distBetweenWheels;
        double countsPerInch = motorCPR / wheel.getCircumference();
        double targetCounts = countsPerInch * distInInches;
        double L1targetCounts, L2targetCounts, R1targetCounts, R2targetCounts;

        if (degrees < 0) {
            // Spin counterclockwise => left motors backward, right motors forward
            motorL.setTargetPosition(getNonZeroCurrentPos(motorL) - (int) targetCounts);
            motorR.setTargetPosition(getNonZeroCurrentPos(motorR) + (int) targetCounts);
        } else {
            // Spin clockwise => left motors forward, right motors backward
            motorL.setTargetPosition(getNonZeroCurrentPos(motorL) + (int) targetCounts);
            motorR.setTargetPosition(getNonZeroCurrentPos(motorR) - (int) targetCounts);
        }
        DbgLog.msg("ftc9773: motorL1 current position = %d", getNonZeroCurrentPos(motorL));

        setDriveSysMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.drive((float) (speed * frictionCoefficient), 0.0f);

        while (motorL.isBusy()  && motorR.isBusy() && curOpMode.opModeIsActive()) {
//                && !navExc.stopNavigation() && curOpMode.opModeIsActive()) {
            curOpMode.idle();
        }

        this.stop();

        DbgLog.msg("ftc9773: motorL1 current position = %d", getNonZeroCurrentPos(motorL));
        setDriveSysMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return;
    }

    //    public void driveToDistance(float speed, float direction, double distance){
//        double startingPositionL = motorL.getCurrentPosition();
//        double startingPositionR = motorR.getCurrentPosition();
//
//        double targetPosition =(distance / wheel.getCircumference()) * motorCPR;
//
//        while(((motorL.getCurrentPosition()-startingPositionL)<targetPosition) &&
//                ((motorR.getCurrentPosition()-startingPositionR)<targetPosition)){
//            drive(speed, direction);
//        }
//        motorR.setPower(0);
//        motorL.setPower(0);
//    }
    @Override
    public void driveToDistance(float speed, double distance){
        double countsPerInch = motorCPR / wheel.getCircumference();
        double targetCounts = countsPerInch * distance;

        motorL.setTargetPosition(motorL.getCurrentPosition() + (int) targetCounts);
        motorR.setTargetPosition(motorR.getCurrentPosition() + (int) targetCounts);

        setDriveSysMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.drive((float) (speed * frictionCoefficient), 0.0f);

        while (motorL.isBusy() && motorR.isBusy()  && curOpMode.opModeIsActive()) {
            curOpMode.idle();
        }

        this.stop();

        setDriveSysMode(DcMotor.RunMode.RUN_USING_ENCODER);

        return;
    }

    @Override
    public void setMaxSpeed(float maxSpeed) {
        motorL.setMaxSpeed((int)(motorLMaxSpeed * maxSpeed));
        motorR.setMaxSpeed((int)(motorRMaxSpeed * maxSpeed));
    }

    @Override
    public void resumeMaxSpeed() {
        motorL.setMaxSpeed(motorLMaxSpeed);
        motorR.setMaxSpeed(motorRMaxSpeed);
    }

    @Override
    public void reverse() {
        if (driveSysIsReversed) {
            motorL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorR.setDirection(DcMotorSimple.Direction.REVERSE);
            driveSysIsReversed = false;
        }
        else {
            motorL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorR.setDirection(DcMotorSimple.Direction.FORWARD);
            driveSysIsReversed = true;
        }
    }

    @Override
    public ElapsedEncoderCounts getNewElapsedCountsObj() {
        ElapsedEncoderCounts encoderCountsObj = new ElapsedEncoderCounts();
        return encoderCountsObj;
    }

    @Override
    public void printCurrentPosition() {
        DbgLog.msg("ftc9773: L:%d, R:%d", motorL.getCurrentPosition(), motorR.getCurrentPosition());
    }

    @Override
    public void initForPlay() {
        Ltimer.reset();
        Rtimer.reset();
        return;
    }

    @Override
    public String getDriveSysInstrData() {
        String instrData= String.format("%f,%f,%f,%f", motorL.getPower(), motorR.getPower(),
                motorL.getCurrentPosition(), motorR.getCurrentPosition());
        return instrData;
    }

    private void setDriveSysMode(DcMotor.RunMode runMode) {
        motorL.setMode(runMode);
        motorR.setMode(runMode);
    }

    public int getNonZeroCurrentPos(DcMotor motor){
        int curPos = motor.getCurrentPosition();
        boolean skipWhileLoop = false;
//        DbgLog.msg("ftc9773: Motor = %s, curPos = %d, isZeroPos = %b", motor.toString(), curPos, motor==motorL1 ? LisZero : motor==motorR1 ? RisZero : motor==motorL2 ? L2IsZero : R2IsZero);
        if(motor==motorL && LisZero) {
//            DbgLog.msg("ftc9773: Motor = L1, curPos = %d, isZeroPos = %b", curPos, LisZero);
            if(Ltimer.milliseconds()>1000) LisZero = false;
            skipWhileLoop = true;}
        if(motor==motorR && RisZero) {
//            DbgLog.msg("ftc9773: Motor = R1, curPos = %d, isZeroPos = %b",  curPos, RisZero);
            if(Rtimer.milliseconds()>1000) RisZero = false;
            skipWhileLoop = true;
        }
        if (skipWhileLoop) return curPos;

        while(curPos==0){
            curOpMode.sleep(10);
            curPos = motor.getCurrentPosition();
        }
//        DbgLog.msg("ftc9773: Motor = %s, curPos = %d, isZeroPos = %b", motor.toString(), curPos, motor==motorL1 ? LisZero : motor==motorR1 ? RisZero : motor==motorL2 ? L2IsZero : R2IsZero);
        return curPos;
    }


}
