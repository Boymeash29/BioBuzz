package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.sequences.DrivingSequences;

public class Robot {
    public Drivetrain drivetrain;
    private final GoBildaPinpointDriver pinpoint;

    // Constructor for Autonomous (used by MainAuto)
    public Robot(HardwareMap hw, DrivingSequences driveSequences) {
        pinpoint = hw.get(GoBildaPinpointDriver.class, "pinpoint");

        Pose2d startPose = new Pose2d(0, 0, 0); // default if no start location defined
        if (driveSequences != null && driveSequences.getLocations().containsKey("start")) {
            Pose2d found = driveSequences.getLocations().get("start");
            if (found != null) startPose = found;
        }

        setPose(startPose);

        // This is where all the subsystems will be initialized
        drivetrain = new Drivetrain(hw, startPose);
    }

    public Robot(HardwareMap hw) {
        pinpoint = hw.get(GoBildaPinpointDriver.class, "pinpoint");

        // This is where all the subsystems will be initialized
        drivetrain = new Drivetrain(hw, new Pose2d(0, 0, 0));
    }

    public void setPose(Pose2d pose) {
        if (pinpoint != null) {
            pinpoint.setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(
                    DistanceUnit.INCH, pose.position.x, pose.position.y,
                    AngleUnit.RADIANS, pose.heading.toDouble()));
        }
    }

    public void stopAll() {
        drivetrain.stop();
    }
}