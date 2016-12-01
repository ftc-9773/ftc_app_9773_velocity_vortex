package org.firstinspires.ftc.teamcode.drivesys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class TwoMotorDrive extends DriveSystem{
    DcMotor motorL = null;
    DcMotor motorR = null;
    double frictionCoefficient;
    double maxSpeed;
    double minSpeed;
    int motorLMaxSpeed, motorRMaxSpeed;
    Wheel wheel;
    int motorCPR;  // Cycles Per Revolution.  == 1120 for Neverest40
    boolean driveSysIsReversed = false;

    public TwoMotorDrive(DcMotor motorL, DcMotor motorR, double maxSpeed, double minSpeed,
                         double frictionCoefficient, Wheel wheel, int motorCPR){
        this.motorL = motorL;
        this.motorR = motorR;
        this.frictionCoefficient = frictionCoefficient;
        this.maxSpeed = maxSpeed;
        this.minSpeed = minSpeed;
        this.wheel = wheel;
        this.motorCPR = motorCPR;
        this.motorLMaxSpeed = motorL.getMaxSpeed();
        this.motorRMaxSpeed = motorR.getMaxSpeed();
    }

    @Override
    public void drive(float speed, float direction){
        double left = (speed - direction) * frictionCoefficient;
        double right = (speed + direction) * frictionCoefficient;

        motorL.setPower(left);
        motorR.setPower(right);
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
}
