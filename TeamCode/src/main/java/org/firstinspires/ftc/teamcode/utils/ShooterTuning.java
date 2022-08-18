package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.Constants.PROJECTILE_MOTION.*;

// Using derivative to calculate the necessary change in velocity and theta
// dh = sin(theta)*v/g

public class ShooterTuning {
    private double deltaValue;
    private double V;
    private double Theta;
    private double initV;
    private double initTheta;

    public ShooterTuning(double deltaHeight) {
        deltaValue = deltaHeight;
    }

    public void update (double v0, double theta0) {
        initV = v0;
        initTheta = theta0;
    }

    public double calculateVError (double thetaError) {
        double value = GRAVITATIONAL_ACCEL*deltaValue/(initV*Math.sin(initTheta))- initV * Math.cos(initTheta)*thetaError;
        double vError = value/Math.sin(initTheta);
        return vError;
    }

    public double calculateThetaError (double vError) {
        double value = GRAVITATIONAL_ACCEL * deltaValue/(initV*Math.sin(initTheta)) - vError*Math.sin(initTheta);
        double thetaError = value/(initV*Math.cos(initTheta));
        return thetaError;
    }
}
