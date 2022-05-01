package frc.lib.util;


import edu.wpi.first.math.controller.PIDController;

public class PIDConfig {

    private double Kp;
    private double Ki;
    private double Kd;
    private double Kf;

    public PIDController getController(double period){
        return new PIDController(Kp, Ki, Kd, period);
    }

    /**
     * Constructs a Generic PIDConfig (Wrapper class to include all of the PID's values)
     * @param Kp - Proportional value
     * @param Ki - Integral value
     * @param Kd - Derrivative value
     * @param Kf - Feedforward value
     */
    public PIDConfig(double Kp, double Ki, double Kd, double Kf) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
    }

    /**
     * Constructs a Generic PIDConfig (Wrapper class to include all of the PID's values)
     * @param Kp - Proportional value
     * @param Ki - Integral value
     * @param Kd - Derrivative value
     */
    public PIDConfig(double Kp, double Ki, double Kd) {
        this(Kp, Ki, Kd, 0);
    }

    public double getKp(){return this.Kp;}
    public double getKi(){return this.Ki;}
    public double getKd(){return this.Kd;}
    public double getKf(){return this.Kf;}

    public void setKp(double Kp){this.Kp = Kp;}
    public void setKi(double Ki){this.Ki = Ki;}
    public void setKd(double Kd){this.Kd = Kd;}
    public void setKf(double Kf){this.Kf = Kf;}
}

