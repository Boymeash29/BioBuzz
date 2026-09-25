package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Class to control turret aiming
 */
public class Turret {

    // declare hardware here
    private final DcMotorEx motor;

    private boolean enabled = false;
    private boolean tracking = false;
    private double lastTurnErrorDeg = 0.0;
    private Vision.CameraSource lastSource = null;

    public Turret(HardwareMap hw) {
        // initialize hardware here
        motor = hw.get(DcMotorEx.class, "turret");
    }

    public boolean isEnabled() { return enabled; }

    /**
     * Enable or disable turret tracking
     * @param e - enabled state
     */
    public void setEnabled(boolean e) {
        enabled = e;
        if (!e) tracking = false;
    }

    public boolean isTracking() { return tracking; }
    public Vision.CameraSource getLastSource() { return lastSource; }
    public double getLastTurnErrorDeg() { return lastTurnErrorDeg; }

    /**
     * Get the current turret angle in degrees
     */
    public double getTurretAngleDeg() {
        return 0.0; // TODO: read from motor encoder
    }

    public boolean isOnTarget() {
        return tracking && Math.abs(lastTurnErrorDeg) <= 1.5;
    }

    /**
     * Update turret aiming toward the scorable target
     * @param status - scoring target status from Vision
     */
    public void update(Vision.ScoringTargetStatus status) {
        // aiming logic here
    }

    /**
     * Manual driver override
     * @param power - power to apply
     */
    public void manualNudge(double power) {
        tracking = false;
        // set power here
    }

    /**
     * Safety stop function
     */
    public void stop() {
        motor.setPower(0);
    }
}