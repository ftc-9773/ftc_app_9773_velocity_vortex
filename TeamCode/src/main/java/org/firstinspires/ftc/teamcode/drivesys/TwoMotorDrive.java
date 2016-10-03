package org.firstinspires.ftc.teamcode.drivesys;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TwoMotorDrive {
    DcMotor motorL = null;
    DcMotor motorR = null;
    double frictionCoefficientR;
    double frictionCoefficientL;
    double maxSpeed;
    double minSpeed;
    Wheel wheel;
    double[] wheelValues;

    public TwoMotorDrive(DcMotor motorL, DcMotor motorR, double maxSpeed, double minSpeed, double frictionCoefficientR, double frictionCoefficientL, Wheel wheel){
        this.motorL = motorL;
        this.motorR = motorR;
        this.frictionCoefficientR = frictionCoefficientR;
        this.frictionCoefficientL = frictionCoefficientL;
        this.maxSpeed = maxSpeed;
        this.minSpeed = minSpeed;
        this.wheel = wheel;
        this.wheelValues = wheel.getValues();
    }

    public void drive(float speed, float direction){
        double left = (-speed + direction) * frictionCoefficientL;
        double right = (speed + direction) * frictionCoefficientR;

        motorL.setPower(left);
        motorR.setPower(right);
    }

    public void driveToDistance(float speed, float direction, double distance){
        double startingPositionL = motorL.getCurrentPosition();
        double startingPositionR = motorR.getCurrentPosition();

        double targetPosition =(distance / wheelValues[1]) * 1120;

        while(((motorL.getCurrentPosition()-startingPositionL)<targetPosition) && ((motorR.getCurrentPosition()-startingPositionR)<targetPosition)){
            drive(speed, direction);
        }
        motorR.setPower(0);
        motorL.setPower(0);
    }
}
