package org.firstinspires.ftc.teamcode.drivesys;

public class Wheel {
    public enum Type{MECANUM, OMNI, RUBBER_TREADED}

    Type type;
    public Wheel(Type type){
        this.type = type;
    }

    public double[] getValues(){
        double[] values = new double[2];
        switch (type){
            case MECANUM:
                values[0] = 4;
                values[1] = 4 * Math.PI;
                break;
            case OMNI:
                values[0] = 4;
                values[1] = 4 * Math.PI;
                break;
            case RUBBER_TREADED:
                values[0] = 4;
                values[1] = 4 * Math.PI;
                break;
            default:
                break;
        }
        return values;
    }

}
