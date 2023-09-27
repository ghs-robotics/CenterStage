package org.firstinspires.ftc.teamcode.bot.control;

public class PID {
    private double P = 0;
    private double I = 0;
    private double D = 0;

    private double maxIOutput = 0;
    private double maxError = 0;
    private double errorSum = 0;

    private double maxOutput = 0;
    private double minOutput = 0;

    private double target = 0;

    private double lastActual = 0;

    private boolean firstRun = true;
    private boolean reversed = false;

    private double outputRampRate = 0;
    private double lastOutput = 0;

    private double outputFilter = 0;

    private double targetRange = 0;


    public PID (double p, double i, double d) {
        P = p;
        I = i;
        D = d;
        checkSigns();
    }

    public void setP (double p) {
        P = p;
        checkSigns();
    }

    public void setI (double i) {
        if(i != 0) {
            errorSum = errorSum * I / i;
        }
        if(maxIOutput!=0) {
            maxError = maxIOutput / i;
        }
        I = i;
        checkSigns();
    }

    public void setD (double d) {
        D = d;
        checkSigns();
    }

    public void setPID (double p, double i, double d) {
        P = p;
        D = d;
        setI(i);
        checkSigns();
    }

    public void setMaxIOutput (double max) {
        maxIOutput = max;
        if (I != 0) {
            maxError = maxIOutput / I;
        }
    }

    public void setOutputLimits (double output) {
        setOutputLimits(-output, output);
    }

    public void setOutputLimits (double min, double max) {
        if (!(max < min)) {
            maxOutput = max;
            minOutput = min;
        }
        if (maxIOutput == 0 || maxIOutput > (max - min)) {
            setMaxIOutput(max - min);
        }
    }

    public void setDirection (boolean reversed) {
        this.reversed = reversed;
    }

    public void setTarget (double target) {
        this.target = target;
    }

    public double getOutput (double actual, double target) {
        double output;
        double Poutput;
        double Ioutput;
        double Doutput;

        this.target = target;

        if (targetRange != 0) {
            target = constrain(target, actual - targetRange, actual + targetRange);
        }

        double error = target - actual;

        Poutput = P * error;

        if (firstRun) {
            lastActual = actual;
            lastOutput = Poutput;
            firstRun = false;
        }

        Doutput = -D * (actual - lastActual);
        lastActual = actual;



        return 0;
    }

    private double constrain (double value, double min, double max) {
        if (value > max) {
            return max;
        }
        if (value < min) {
            return min;
        }
        return value;
    }

    private void checkSigns () {
        if (reversed) {
            if (P > 0) {P = -P;}
            if (I > 0) {I = -I;}
            if (D > 0) {D = -D;}
        } else {
            if (P < 0) {P = -P;}
            if (I < 0) {I = -I;}
            if (D < 0) {D = -D;}
        }
    }
}