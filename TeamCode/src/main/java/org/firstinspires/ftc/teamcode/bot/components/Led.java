package org.firstinspires.ftc.teamcode.bot.components;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Led {
    RevBlinkinLedDriver blinkin;
    int numPixels = 0;

    private boolean ledTurnOff = false;

    public Led(HardwareMap hardwaremap) {

        blinkin = hardwaremap.get(RevBlinkinLedDriver.class, "blinkin");
        ledsOff();
    }

    /**
     * the next three functions are increasing num of pixels to 1, 2, and 0. only reason they
     * are separate functions is for testing, otherwise they can be all in one if else thingy
     *
     * @param pressed just was using pressed for testing purposes, later on this param
     *                should be based on distance sensor stuff
     */
    public void numPixels1(boolean pressed) {
        if (pressed) {
            numPixels = 1;
        }
    }

    public void numPixels2(boolean pressed) {
        if (pressed) {
            numPixels = 2;
        }
    }

    public void numPixelsReset(boolean pressed) {
        if (pressed) {
            numPixels = 0;
        }
    }

    public void setNumPixels(int n){
        numPixels = n;
    }

    public void ledsOff(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    /**
     * constantly run the leds to default purple if number of pixels is less than two and turn white
     * once there are two pixels.
    */
    public void runLeds() {

        if (ledTurnOff) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            return;
        }
        if(numPixels == 0){
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        } else if (numPixels == 2) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        } else if (numPixels < 2) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        }
        else
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
    }

    public void switchLedOnState(boolean pressed) {
        if (pressed) {
            ledTurnOff = !ledTurnOff;
        }
    }
}
