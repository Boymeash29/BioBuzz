package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.ArrayList;
import java.util.List;

/**
 * Class to control vision: turret cam and chassis cam, ball detection,
 * opposing robot detection, and the ball color sensor.
 */
public class Vision {

    // declare hardware here
    private final VisionPortal turretPortal;
    private final AprilTagProcessor turretAprilTag;

    private final VisionPortal chassisPortal;
    private final AprilTagProcessor chassisAprilTag;
    private final ColorBlobLocatorProcessor pollenLocator;
    private final ColorBlobLocatorProcessor nectarLocatorRed;
    private final ColorBlobLocatorProcessor nectarLocatorBlue;

    private final NormalizedColorSensor ballSensor;

    private Ball currentBall = Ball.NONE;

    public Vision(HardwareMap hw) {
        // initialize hardware here
        int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        turretAprilTag = new AprilTagProcessor.Builder().build();
        turretPortal = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, "Webcam Turret"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(turretAprilTag)
                .setLiveViewContainerId(viewIds[0])
                .build();

        chassisAprilTag = new AprilTagProcessor.Builder().build();

        ImageRegion fieldROI = ImageRegion.asUnityCenterCoordinates(-1.0, 0.4, 1.0, -1.0);
        pollenLocator = new ColorBlobLocatorProcessor.Builder().setTargetColorRange(ColorRange.YELLOW).setRoi(fieldROI).build();
        nectarLocatorRed = new ColorBlobLocatorProcessor.Builder().setTargetColorRange(ColorRange.RED).setRoi(fieldROI).build();
        nectarLocatorBlue = new ColorBlobLocatorProcessor.Builder().setTargetColorRange(ColorRange.BLUE).setRoi(fieldROI).build();

        chassisPortal = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, "Webcam Chassis"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(chassisAprilTag)
                .addProcessor(pollenLocator)
                .addProcessor(nectarLocatorRed)
                .addProcessor(nectarLocatorBlue)
                .setLiveViewContainerId(viewIds[1])
                .build();

        ballSensor = hw.get(NormalizedColorSensor.class, "ballColor");
    }

    public boolean isTurretCameraReady() {
        return turretPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    public boolean isChassisCameraReady() {
        return chassisPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    public void setTurretCamAprilTagEnabled(boolean enabled) {
        turretPortal.setProcessorEnabled(turretAprilTag, enabled);
    }

    public void setChassisCamAprilTagEnabled(boolean enabled) {
        chassisPortal.setProcessorEnabled(chassisAprilTag, enabled);
    }

    public void setPollenEnabled(boolean enabled) {
        chassisPortal.setProcessorEnabled(pollenLocator, enabled);
    }

    public void setRedNectarEnabled(boolean enabled) {
        chassisPortal.setProcessorEnabled(nectarLocatorRed, enabled);
    }

    public void setBlueNectarEnabled(boolean enabled) {
        chassisPortal.setProcessorEnabled(nectarLocatorBlue, enabled);
    }

    public void setTfodEnabled(boolean enabled) {
        // enable/disable robot classifier here
    }

    public enum CameraSource { TURRET_CAM, CHASSIS_CAM }
    public enum AllianceColor { RED, BLUE }

    /** Targeting solution for the scorable CELL cluster. */
    public static class TargetSolution {
        public final CameraSource source;
        public final String clusterName;
        public final double bearingDeg;
        public final double rangeIn;
        public final double elevationDeg;
        public final double yawDeg;
        public final double rollDeg;
        public final double percentClusterFound;
        public final long timestampMs;

        public TargetSolution(CameraSource source, String clusterName, double bearingDeg, double rangeIn,
                              double elevationDeg, double yawDeg, double rollDeg,
                              double percentClusterFound, long timestampMs) {
            this.source = source;
            this.clusterName = clusterName;
            this.bearingDeg = bearingDeg;
            this.rangeIn = rangeIn;
            this.elevationDeg = elevationDeg;
            this.yawDeg = yawDeg;
            this.rollDeg = rollDeg;
            this.percentClusterFound = percentClusterFound;
            this.timestampMs = timestampMs;
        }
    }

    /** High-level status of the scorable target, including staleness. */
    public static class ScoringTargetStatus {
        public final boolean found;
        public final boolean isCurrentlyVisible;
        public final TargetSolution solution;
        public final long ageMs;

        public ScoringTargetStatus(boolean found, boolean isCurrentlyVisible, TargetSolution solution, long ageMs) {
            this.found = found;
            this.isCurrentlyVisible = isCurrentlyVisible;
            this.solution = solution;
            this.ageMs = ageMs;
        }
    }

    /**
     * Finds the scorable target for the given alliance
     * @param alliance - alliance to search for
     */
    public ScoringTargetStatus getScorableTarget(AllianceColor alliance) {
        return new ScoringTargetStatus(false, false, null, -1); // TODO: scan clusters, pick best, track staleness
    }

    public List<ColorBlobLocatorProcessor.Blob> getPollenBlobs() {
        return new ArrayList<>(); // TODO: read + filter blobs
    }

    public List<ColorBlobLocatorProcessor.Blob> getRedNectarBlobs() {
        return new ArrayList<>(); // TODO: read + filter blobs
    }

    public List<ColorBlobLocatorProcessor.Blob> getBlueNectarBlobs() {
        return new ArrayList<>(); // TODO: read + filter blobs
    }

    public List<Recognition> getCertainRobotDetections() {
        return new ArrayList<>(); // TODO: read classifier results
    }

    public Vector2d getRobotAvoidanceVector() {
        return new Vector2d(0, 0); // TODO: combine detections into an avoidance vector
    }

    public static class Recognition {
        public final String label;
        public final float confidence;
        public final String region;

        public Recognition(String label, float confidence, String region) {
            this.label = label;
            this.confidence = confidence;
            this.region = region;
        }

        public String getLabel() { return label; }
        public float getConfidence() { return confidence; }
        public String getRegion() { return region; }
    }

    public static final double POLLEN_DIAMETER_IN = 2.8;
    public static final double NECTAR_DIAMETER_IN = 3.6;

    /** Type of ball currently passing the color sensor on the way to the turret. */
    public enum Ball {
        NONE, POLLEN, NECTAR_RED, NECTAR_BLUE, UNKNOWN;

        public boolean isLarge() {
            return this == NECTAR_RED || this == NECTAR_BLUE || this == UNKNOWN;
        }

        public double diameterIn() {
            if (this == NONE) return 0.0;
            return isLarge() ? NECTAR_DIAMETER_IN : POLLEN_DIAMETER_IN;
        }
    }

    /**
     * Reads the color sensor and updates which ball is currently passing
     * into the turret. Call once per loop.
     */
    public void updateBallSensor() {
        // color classification logic here
    }

    public Ball getCurrentBall() { return currentBall; }

    public double getCurrentBallDiameterIn() {
        return currentBall.diameterIn();
    }

    public void close() {
        turretPortal.close();
        chassisPortal.close();
    }
}