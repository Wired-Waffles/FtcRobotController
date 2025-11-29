package org.firstinspires.ftc.teamcode.mechanisms;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagRead {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal vision;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();
    private Telemetry telemetry;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        vision = builder.build();
    }
    public void update(){
        detectedTags = aprilTagProcessor.getDetections();

    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public void displayTelemetry(AprilTagDetection ID) {
        if (ID == null) return;
        if (ID.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", ID.id, ID.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (cm)", ID.ftcPose.x, ID.ftcPose.y, ID.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", ID.ftcPose.pitch, ID.ftcPose.roll, ID.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (cm, deg, deg)", ID.ftcPose.range, ID.ftcPose.bearing, ID.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", ID.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", ID.center.x, ID.center.y));
        }
    }

    public AprilTagDetection getTagByID(int ID){
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == ID) return detection;
        }
        return null;
    }
    public void kill() {
        if (vision != null) vision.close();
    }
}
