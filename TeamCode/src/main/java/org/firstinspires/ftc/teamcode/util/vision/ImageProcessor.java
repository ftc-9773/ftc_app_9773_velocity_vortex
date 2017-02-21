package org.firstinspires.ftc.teamcode.util.vision;

import org.opencv.core.Mat;

/**
 * Created by pranavburugula on 12/31/2016.
 */

/*
 * Copyright (c) 2016 Robocracy 9773
 */

public interface ImageProcessor<ResultType> {
    ImageProcessorResult<ResultType> process(long startTime, Mat rgbaFrame, boolean saveImages);
}
