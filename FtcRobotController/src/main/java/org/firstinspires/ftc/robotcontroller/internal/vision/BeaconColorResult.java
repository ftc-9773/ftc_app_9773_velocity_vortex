/*
 * Copyright (c) 2016 Robocracy 9773.
 */

package org.firstinspires.ftc.robotcontroller.internal.vision;

import org.opencv.core.Scalar;

/**
 * Created by pb8xe_000 on 12/30/2016.
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
