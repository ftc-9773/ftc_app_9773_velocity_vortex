package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by ftcrobocracy on 1/1/17.
 */

public class LoopStatistics {
    double minTime, maxTime, avgTime, totalTime;
    int count;
    ElapsedTime timer;

    public LoopStatistics() {
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();
        minTime = Integer.MAX_VALUE;
        maxTime = avgTime = totalTime = 0.0;
        count = 0;
    }

    public void startLoopInstrumentation() {
        timer.reset();
        minTime = Integer.MAX_VALUE;
        maxTime = avgTime = totalTime = 0.0;
        count = 0;
    }

    public void updateLoopInstrumentation() {
        double millis = timer.milliseconds();
        timer.reset();
        if (minTime > millis) {
            minTime = millis;
        }
        if (maxTime < millis) {
            maxTime = millis;
        }
        totalTime += millis;
        count++;
    }

    public void printLoopInstrumentation() {
        avgTime = totalTime / count;
        DbgLog.msg("ftc9773: totalTime=%f, minTime=%f, avgTime=%f, maxTime=%f, count=%d",
                totalTime, minTime, avgTime, maxTime, count);
    }
}
