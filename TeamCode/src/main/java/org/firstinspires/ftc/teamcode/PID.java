package org.firstinspires.ftc.teamcode;

/**
 * PID Controller: kp * (e + (integral(e) / ti) + (td * derivative(e))).
 * https://en.wikipedia.org/wiki/PID_controller#Ideal_versus_standard_PID_form
 */
public class PID {

    // Proportional factor to scale error to output.
    private double kp;
    // The number of seconds to eliminate all past errors.
    private double ti;
    // The number of seconds to predict the error in the future.
    private double td;
    // The min of the running integral.
    private double integralMin;
    // The max of the running integral.
    private double integralMax;
    // The min allowed PID output.
    private double outputMin;
    // The max allowed PID output.
    private double outputMax;

    // The last error value.
    private double previousError;
    // The discrete running integral (bounded by integralMax).
    private double runningIntegral;

    /**
     * Creates a PID Controller.
     *
     * @param kp          Proportional factor to scale error to output.
     * @param ti          The number of seconds to eliminate all past errors.
     * @param td          The number of seconds to predict the error in the future.
     * @param integralMin The min of the running integral.
     * @param integralMax The max of the running integral.
     * @param outputMin   The min of the PID output.
     * @param outputMax   The max of the PID output.
     */
    public PID(double kp, double ti, double td, double integralMin,
               double integralMax, double outputMin, double outputMax) {
        this.kp = kp;
        this.ti = ti;
        this.td = td;
        this.integralMin = integralMin;
        this.integralMax = integralMax;
        this.outputMin = outputMin;
        this.outputMax = outputMax;

        this.previousError = 0;
        this.runningIntegral = 0;
    }

    /**
     * Performs a PID update and returns the output control.
     *
     * @param desiredValue The desired state value (e.g. speed).
     * @param actualValue  The actual state value (e.g. speed).
     * @param dt           The amount of time (sec) elapsed since last update.
     * @return The output which impacts state value (e.g. motor throttle).
     */
    public double update(double desiredValue, double actualValue, double dt) {
        double e = desiredValue - actualValue;
        runningIntegral = clampValue(runningIntegral + e * dt, integralMin, integralMax);
        double d = (e - previousError) / dt;
        double output = clampValue(kp * (e + (runningIntegral / ti) + (td * d)),
                outputMin, outputMax);

        previousError = e;
        return output;
    }

    /**
     * Clamps a value to a given range.
     *
     * @param value The value to clamp.
     * @param min   The min clamp.
     * @param max   The max clamp.
     * @return The clamped value.
     */
    public static double clampValue(double value, double min, double max) {
        return Math.min(max, Math.max(min, value));
    }

    public void setPIDCoef(double P, double I, double D) {
        this.kp = P;
        this.ti = I;
        this.td = D;
    }

    public void setKp(double kp) {
        this.kp = kp;
    }
    public void setTi(double ti) {
        this.ti = ti;
    }
    public void setTd(double td) {
        this.td = td;
    }
    public void setIntegralMin(double integralMin) {
        this.integralMin = integralMin;
    }
    public void setIntegralMax(double integralMax) {
        this.integralMax = integralMax;
    }
    public void setOutputMin(double outputMin) {
        this.outputMin = outputMin;
    }
    public void setOutputMax(double outputMax) {
        this.outputMax = outputMax;
    }

    public double getKp() {
        return this.kp;
    }

    public double getTi() {
        return this.ti;
    }

    public double getTd() {
        return this.td;
    }

    public double getIntegralMin() {
        return this.integralMin;
    }

    public double getIntegralMax() {
        return this.integralMax;
    }

    public double getOutputMin() {
        return this.outputMin;
    }

    public double getOutputMax() {
        return this.outputMax;
    }

    public double getPreviousError() {
        return this.previousError;
    }

    public double getRunningIntegral() {
        return this.runningIntegral;
    }

}
