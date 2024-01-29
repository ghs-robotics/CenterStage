package org.firstinspires.ftc.teamcode.control.PIDControllers;

public class NavigationPID {
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

    private double error;

    private boolean run = false;

    public NavigationPID(double[] pid){
        this(pid[0], pid[1], pid[2]);
    }

    public NavigationPID(double p, double i, double d) {
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
        if (this.target == target)
            return;

        this.target = target;
        run = true;
        reset();
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

        error = target - actual;

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

        lastOutput = output;
        return output;
    }

    public double getOutput (double actual) {
        return getOutput(actual,target);
    }

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

    public void setError(double e){
        error = e;
    }

    public double getError() {
        return error;
    }

    public boolean isRun() {
        return run;
    }

    public void setRun(boolean run) {
        this.run = run;
    }
}
//    private class PID{
//        private double lastError;
//        private double lastIntegral;
//
//        private double error;
//        private double integral;
//        private double derivative;
//
//        private double kp = .6;
//        private double ki = 1.2;
//        private double kd = 0.05;
//        private double bias = 0;
//
//        private double target;
//
//        private ElapsedTime timer;
//
//        private double lastIterationTime;
//
//        private double maxIntegral;
//
//        PID(double p, double i, double d){
//            this();
//            kp = p;
//            ki = i;
//            kd = d;
//        }
//
//        PID(){
//            timer = new ElapsedTime();
//        }
//
//        double getOutput(double current){
//            double iterationTime = timer.milliseconds() - lastIterationTime;
//            lastIterationTime += iterationTime;
//
//            error = target - current;
//            integral = lastIntegral + error * iterationTime;
//            derivative = (error - lastError) / iterationTime;
//
//            constrain();
//
//            double out = (kp * error) + (ki * integral) + (kd * derivative) + bias;
//
//            lastError = error;
//            lastIntegral = integral;
//
//            return out;
//        }
//
//        double getError(){
//            return error;
//        }
//
//        void setMaxError(double max){
//            maxIntegral = max;
//        }
//
//        void constrain(){
//            if (Math.abs(integral) > maxIntegral)
//                integral = maxIntegral * (integral / Math.abs(integral));
//        }
//
//        void setTarget(double target){
//            if (this.target == target)
//                return;
//
//            this.target = target;
//            lastError = 0;
//            lastIntegral = 0;
//            error = 0;
//            integral = 0;
//            derivative = 0;
//            timer.reset();
//            lastIterationTime = 0;
//        }
//
//    }