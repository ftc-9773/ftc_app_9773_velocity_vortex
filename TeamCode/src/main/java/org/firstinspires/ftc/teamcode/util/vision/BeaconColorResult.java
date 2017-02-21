package org.firstinspires.ftc.teamcode.util.vision;

import org.opencv.core.Scalar;

/**
 * Created by pranavburugula on 12/31/2016.
 */

/*
 * Copyright (c) 2016 Robocracy 9773
 */

public class BeaconColorResult {
    public BeaconColorResult(BeaconColor leftColor, BeaconColor rightColor) {
        this.leftColor = leftColor;
        this.rightColor = rightColor;
    }

    public BeaconColor getLeftColor() {
        return leftColor;
    }

    public BeaconColor getRightColor() {
        return rightColor;
    }

    public enum BeaconColor{
        RED (ImageUtil.RED),
        GREEN (ImageUtil.GREEN),
        BLUE (ImageUtil.BLUE),
        UNKNOWN (ImageUtil.BLACK);

        public final Scalar color;

        BeaconColor(Scalar color){
            this.color = color;
        }
    }

    private final BeaconColor leftColor, rightColor;

    public String toString(){
        return leftColor+","+rightColor;
    }
}
