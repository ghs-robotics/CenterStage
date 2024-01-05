package org.firstinspires.ftc.teamcode.bot.components.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.bot.components.Gyro;

public class BallDrive implements Drivebase {
    private DcMotorEx leftDrive;
    private DcMotorEx rightDrive;
    private DcMotorEx backDrive;

    private double lp;
    private double rp;
    private double bp;

    private Gyro gyro;

    private boolean metaDriveOn;

    private double x;
    private double y;
    private double heading;

    private int lastL;
    private int lastR;
    private int lastB;

    public BallDrive(HardwareMap hardwareMap, Gyro gyro) {

        leftDrive = hardwareMap.get(DcMotorEx.class, "left");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right");
        backDrive = hardwareMap.get(DcMotorEx.class, "back");

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE); // y-axis encoder
        backDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD); // x-axis encoder

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lastL = 0;
        lastR = 0;
        lastB = 0;

        this.gyro = gyro;
    }

    public void update(){
        // todo: needs updating

        final double MM_PER_TICK = (35 * Math.PI) / (8192);
        final double Y_MULTI = 609.4 / 450;
        final double X_MULTI = 609.4 / 454;
        final double ROT_MULTI = 1; //90 / 111.2;
//        double lTracking = 134.109;
//        double rTracking = 153.275;
        double lTracking = 153.058;
        double rTracking = 153.058;
        double bTracking = 92.79683;

        int left = leftDrive.getCurrentPosition();
        int right = rightDrive.getCurrentPosition();
        int back = backDrive.getCurrentPosition();

//        int deltaL = - (left - lastL);
//        int deltaR = (right - lastR);
//        int deltaB = back - lastB;

//        heading = (((left) - (right)) / (lTracking + rTracking)) * ROT_MULTI;
        heading = Math.toDegrees(gyro.getHeading());
        double tempHeading = heading;

        if (heading == 0)
            tempHeading = 1;

        double y = 2 * (left / (tempHeading) + lTracking) * Math.sin((heading) / 2);
        double x = 2 * (back / (tempHeading) + bTracking) * Math.sin((heading) / 2);

        this.x = x * X_MULTI * MM_PER_TICK;
        this.y = y * Y_MULTI * MM_PER_TICK;

        lastL = left;
        lastR = right;
        lastB = back;

    }

    public boolean runToPosition(int tarX, int tarY){
        return runToPosition(tarX, tarY, 0);
    }

    public boolean runToPosition(int tarX, int tarY, double tarHeading){
        update();
        int maxError = 25; // should be in ticks
        double headingError = 12;
        double slowingRangeMultiplier = 5;

        double xPower = 0;
        double yPower = 0;
        double rotPower = 0;

        double xError = tarX - this.x;
        double yError = tarY - this.y;

        if (Math.abs(xError) > maxError)
            xPower = (xError) / (maxError * slowingRangeMultiplier);

        else if (Math.abs(yError) > maxError)
            yPower = (yError) / (maxError * slowingRangeMultiplier);

//        else if (Math.abs(tarHeading - this.heading) > headingError){
//            rotPower = (tarHeading - this.heading) / (headingError * slowingRangeMultiplier);
//        }

        calculateDrivePowers(xPower * .75, yPower * .75, rotPower, true);
        return xError <= maxError && yError <=maxError;
    }


    @Override
    public void calculateDrivePowers(double x, double y, double rot) {
        bp = x;
        lp = y - rot;
        rp = y + rot;

        setMotorPowers();
    }

    @Override
    public void calculateDrivePowers(double x, double y, double rot, boolean driveMode){
        double angle = gyro.getHeading();
        metaDriveOn = driveMode;

        double driveX = x;
        double driveY = y;

        if(driveMode) {
            driveX = x * Math.cos(angle) - y * Math.sin(angle) ;
            driveY = y * Math.cos(angle) + x * Math.sin(angle);
        }
        calculateDrivePowers(driveX, driveY, rot);
    }

    @Override
    public int[] getEncoderTicks() {
        // 0 gets y, 1 gets x
        int[] ticks = {leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition(), backDrive.getCurrentPosition()};
        return ticks;
    }

    @Override
    public void resetEncoders() {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public boolean getDriveMode() {
        return metaDriveOn;
    }

    @Override
    public DcMotorEx[] getEncoderMotors(){
        return new DcMotorEx[]{leftDrive, rightDrive, backDrive};

    }

    private void setMotorPowers(){
        leftDrive.setPower(lp);
        rightDrive.setPower(rp);
        backDrive.setPower(bp);
    }

    public void resetCoords(){
        x = 0;
        y = 0;
        heading = 0;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }
}
