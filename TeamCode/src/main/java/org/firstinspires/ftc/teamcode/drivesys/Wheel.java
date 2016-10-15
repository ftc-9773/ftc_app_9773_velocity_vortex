package org.firstinspires.ftc.teamcode.drivesys;

public class Wheel {
    public enum Type{MECANUM, OMNI, RUBBER_TREADED}

    Type type;
    double diameter;
    double circumference;

    public Wheel(Type type, double diameter)
    {
        this.type = type;
        this.diameter = diameter;
        this.circumference = diameter * Math.PI;
    }

    public double getCircumference() { return (this.circumference); }
    public double getRadius() { return (this.diameter); }


}
