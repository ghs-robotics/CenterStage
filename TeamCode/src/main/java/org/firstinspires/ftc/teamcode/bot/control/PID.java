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

        Ioutput = I * errorSum;
        if (maxIOutput != 0) {
            Ioutput = constrain(Ioutput, -maxIOutput, maxIOutput);
        }

        output = Poutput + Ioutput + Doutput;

        if (minOutput != maxOutput && !bounded(output, minOutput, maxOutput)) {
            errorSum = error;
        }
        else if (outputRampRate != 0 && bounded(output, minOutput, maxOutput)) {
            errorSum = error;
        }
        else if (maxIOutput != 0 && !bounded(output, lastOutput - outputRampRate, lastOutput + outputRampRate)) {
            errorSum = constrain(errorSum + error, -maxError, maxError);
        }
        else {
            errorSum += error;
        }
        if (outputRampRate != 0) {
            output = constrain(output, lastOutput - outputRampRate, lastOutput + outputRampRate);
        }
        if (minOutput != maxOutput) {
            output = lastOutput * outputFilter + output * (1 - outputFilter);
        }

        // for testing - prints outputs
         System.out.printf("Final output %5.2f [ %5.2f, %5.2f , %5.2f  ], eSum %.2f\n",output,Poutput, Ioutput, Doutput,errorSum );
         System.out.printf("%5.2f\t%5.2f\t%5.2f\t%5.2f\n",output,Poutput, Ioutput, Doutput );

        lastOutput = output;
        return output;
    }

    public double getOutput (double actual) {
        return getOutput(lastActual,target);
    }

    /**
     * resets the booleans firstRun and errorSum
     */
    public void reset () {
        firstRun = true;
        errorSum = 0;
    }

    public void setOutputRampRate (double rate) {
        outputRampRate = rate;
    }

    public void setTargetRange (double range) {
        targetRange = range;
    }

    public void setOutputFilter (double strength) {
        if (strength == 0 || bounded(strength, 0, 1)) {
            outputFilter = strength;
        }
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

    private boolean bounded (double value, double min, double max) {
        return (min < value) && (value < max);
    }

    /**
     * Checks the reversed boolean to ensure PID is making proper adjustments
     * If the reversed is true, the PID will decrease motor input
     * If reversed is false, the PID will increase motor input
     */
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