package org.firstinspires.ftc.teamcode;

public class PIDFController {
    private double Kp, Ki, Kd, Kf;
    private double integral, previousError;
    private double setpoint;

    public PIDFController(double Kp, double Ki, double Kd, double Kf) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        this.integral = 0;
        this.previousError = 0;
    }

    public double compute(double setpoint, double measuredValue, double deltaTime) {
        this.setpoint = setpoint;
        double error = setpoint - measuredValue;
        integral += error * deltaTime;
        double derivative = (error - previousError) / deltaTime;
        previousError = error;

        return Kp * error + Ki * integral + Kd * derivative + Kf * setpoint;
    }
    public double compute(double setpoint, double measuredValue) {
        this.setpoint = setpoint;
        double error = setpoint - measuredValue;
        integral += error; // Integrate error directly
        double derivative = error - previousError;
        previousError = error;

        return Kp * error + Ki * integral + Kd * derivative + Kf * setpoint;
    }
}

