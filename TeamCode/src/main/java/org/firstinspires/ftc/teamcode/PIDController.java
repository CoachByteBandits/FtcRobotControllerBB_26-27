package org.firstinspires.ftc.teamcode;

public class PIDController {

    public double setPoint = 0;
    public double maxOutput = 1.0;
    public double maxIntegral = 0.5;

    private double Kp, Ki, Kd;
    private double currentError = 0;
    private double lastError = 0;
    private double lastInput = 0;
    private double integral = 0;
    private long lastTime = System.nanoTime();
    private boolean hasRun = false; // Prevents First-Run Derivative Spike

    public PIDController(double KP, double KI, double KD) {
        this.Kp = KP;
        this.Ki = KI;
        this.Kd = KD;
    }

    public void setSetPoint(double setPoint) {
        this.setPoint = setPoint;
    }

    public void setOutputRange(double maxOutput) {
        this.maxOutput = maxOutput;
    }

    public void reset() {
        currentError = 0;
        lastError = 0;
        lastInput = 0;
        integral = 0;
        lastTime = System.nanoTime();
        hasRun = false; // Reset first run flag
    }

    // Standard linear distance calculation
    public double getComputedOutput(double input) {
        double error = setPoint - input;
        return computePID(error, input);
    }

    // Angle-wrapped calculation for heading in Radians (-PI to PI)
    public double getComputedAngleOutput(double currentAngleRad) {
        double error = setPoint - currentAngleRad;

        // Angle Wrap: normalize error to [-PI, PI]
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;

        return computePID(error, currentAngleRad);
    }

    private double computePID(double error, double input) {
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9; // Convert nanoseconds to seconds
        if (dt <= 0) dt = 0.001;

        currentError = error;

        // Handle first frame cleanly without derivative spike
        if (!hasRun) {
            lastInput = input;
            lastError = error;
            lastTime = currentTime;
            hasRun = true;
        }

        // Anti-windup integral
        integral += currentError * dt;
        if (integral > maxIntegral) integral = maxIntegral;
        if (integral < -maxIntegral) integral = -maxIntegral;

        // Calculate derivative on input change (prevents derivative kick)
        double inputChange = (input - lastInput) / dt;
        double derivative = -inputChange;

        double output = (Kp * currentError) + (Ki * integral) + (Kd * derivative);

        // Clamp output
        output = Math.max(-maxOutput, Math.min(maxOutput, output));

        lastError = currentError;
        lastInput = input;
        lastTime = currentTime;

        return output;
    }

    public double getCurrentError() {
        return currentError;
    }
}