package org.firstinspires.ftc.teamcode.drivesys;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
            DbgLog.msg("printCurrent...(): encoder counts: L=%d, R=%d",
                    motorL.getCurrentPosition(), motorR.getCurrentPosition());
        }
    }

    public TwoMotorDrive(DcMotor motorL, DcMotor motorR, int maxSpeedCPS,
                         double frictionCoefficient, Wheel wheel, int motorCPR){
        this.motorL = motorL;
        this.motorR = motorR;
        this.frictionCoefficient = frictionCoefficient;
        this.maxSpeedCPS = maxSpeedCPS;
        DbgLog.msg("max speed CPS = %d", maxSpeedCPS);
        motorL.setMaxSpeed(maxSpeedCPS);
        motorR.setMaxSpeed(maxSpeedCPS);
        this.wheel = wheel;
        this.motorCPR = motorCPR;
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
        DbgLog.msg("L:%d, R:%d", motorL.getCurrentPosition(), motorR.getCurrentPosition());
    }
}
