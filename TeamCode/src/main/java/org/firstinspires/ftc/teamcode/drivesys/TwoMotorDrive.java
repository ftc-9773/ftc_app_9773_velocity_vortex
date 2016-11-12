package org.firstinspires.ftc.teamcode.drivesys;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TwoMotorDrive extends DriveSystem{
    DcMotor motorL = null;
    DcMotor motorR = null;
    double frictionCoefficient;
    double maxSpeed;
    double minSpeed;
    Wheel wheel;
    int motorCPR;  // Cycles Per Revolution.  == 1120 for Neverest40

    public TwoMotorDrive(DcMotor motorL, DcMotor motorR, double maxSpeed, double minSpeed,
                         double frictionCoefficient, Wheel wheel, int motorCPR){
        this.motorL = motorL;
        this.motorR = motorR;
        this.frictionCoefficient = frictionCoefficient;
        this.maxSpeed = maxSpeed;
        this.minSpeed = minSpeed;
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
}
