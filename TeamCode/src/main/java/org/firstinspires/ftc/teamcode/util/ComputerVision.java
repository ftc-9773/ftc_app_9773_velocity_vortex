package org.firstinspires.ftc.teamcode.util;


import com.vuforia.CameraDevice;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.State;

import java.nio.ByteBuffer;

public class ComputerVision {
    @Override
    public void onVuforiaUpdate(State state){
        Image imageRGB565 = null;
        Frame frame = state.getFrame();

        for(int i=0;i<frame.getNumImages();++i){
            Image image = frame.getImage(i);
            if(image.getFormat() == PIXEL_FORMAT.RGB565){
                imageRGB565 = image;
                break;
            }
        }

        if(imageRGB565 != null){
            ByteBuffer pixels = imageRGB565.getPixels();
            byte[] pixelArray = new byte[pixels.remaining()];
            pixels.get(pixelArray, 0, pixelArray.length);
        }
    }
}
