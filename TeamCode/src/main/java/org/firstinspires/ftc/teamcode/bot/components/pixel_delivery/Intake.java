package org.firstinspires.ftc.teamcode.bot.components.pixel_delivery;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.reflect.Array;
import java.util.ArrayList;

public class Intake {
    private final DcMotor conveyorBeltMotor;
    private final DcMotor intakeMotor;
    private final Servo intakeServo;
    private final DistanceSensor distance;
    private final double[] intakeServoPos = {0.01, 0.06, 0.12, 0.15, 0.17, 0.21};

    private int intakeLvl = 60;

    private int pixels;

    private double[] pixelDistances = {0, 0, 0, 0, 0, 0};

    public Intake(HardwareMap hardwareMap) {
        conveyorBeltMotor = hardwareMap.get(DcMotor.class, "belt");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakem");
        intakeServo = hardwareMap.get(Servo.class, "intakes");
        distance = hardwareMap.get(DistanceSensor.class, "distance");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeServo.setPosition(0.05);

        pixels = 0;
    }

    //-------------------------------------------------------------------------------------
    //                                   AutoRed Functions
    //-------------------------------------------------------------------------------------

    public void autoPixelOut() {
        intakeMotor.setPower(-0.5);
        conveyorBeltMotor.setPower(-1);
    }

    public void setIntakeHeight(int targetLevel) {
        int diff = targetLevel - getIntakeLvl();
        intakeLvl += diff;
        setIntakePosition();
    }

    //-------------------------------------------------------------------------------------
    //                                   Intake Functions
    //-------------------------------------------------------------------------------------

    public void pixelIn(double power) {
        intakeMotor.setPower(power / 2);
        conveyorBeltMotor.setPower(-power);
    }

    public void changeIntakeHeight(boolean decrease, boolean increase) {
        if (decrease) {
            intakeLvl -= 1;
        }
        if (increase) {
            intakeLvl += 1;
        }
        setIntakePosition();
    }

    public void addDataToDistanceArray () {
        for (int i = 0; i < pixelDistances.length; i++) {
            pixelDistances[i] = getDistanceFromPixel();
        }
    }

    public int countPixels() {
        if (pixelDistances[0] > pixelDistances[5] + 0.6) {
            pixels++;
        }
        return pixels;
    }

    public String pixelDistancesToString () {
        String dist = "";
        for (int i = 0; i < pixelDistances.length - 1; i++) {
            dist = dist + pixelDistances[i] + ", ";
        }
        dist = dist + pixelDistances[pixelDistances.length - 1];

        return dist;
    }

    //-------------------------------------------------------------------------------------
    //                                   Simple Functions
    //-------------------------------------------------------------------------------------

    private void setIntakePosition () {
        intakeServo.setPosition(intakeServoPos[Math.abs(intakeLvl % intakeServoPos.length)]);
    }

    public double getIntakePosition () {
        return intakeServo.getPosition();
    }

    private int getIntakeLvl () {
        return Math.abs(intakeLvl % intakeServoPos.length);
    }

    public int getPixelNumber () {
        return countPixels();
    }

    public double getDistanceFromPixel () {
        return distance.getDistance(DistanceUnit.CM);
    }

    public String getPixelDistances () {
        return pixelDistancesToString();
    }
}

