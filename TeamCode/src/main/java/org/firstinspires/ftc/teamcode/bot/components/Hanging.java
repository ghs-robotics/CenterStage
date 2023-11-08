package org.firstinspires.ftc.teamcode.bot.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hanging {
    private CRServo hangServo1;
    private CRServo hangServo2;

    /**
     * puts hanging object in robot
     * @param hardwareMap is how code interacts with robot
     */
    public Hanging (HardwareMap hardwareMap) {
        hangServo1 = hardwareMap.get(CRServo.class, "hang1");
        hangServo2 = hardwareMap.get(CRServo.class, "hang2");
    }

    /**
     * Hangs the robot on the truss
     * @param lower brings robot down
     * @param raise brings robot up
     */
    public void hang (boolean lower, boolean raise) {
        if (lower) {
            hangServo1.setPower(-1);
            hangServo2.setPower(1);
        }
        else if (raise) {
            hangServo1.setPower(1);
            hangServo2.setPower(-1);
        }
        else {
            hangServo1.setPower(0);
            hangServo2.setPower(0);
        }
    }
}
