package org.firstinspires.ftc.teamcode.mechanism;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class GoalAprilTagTracker {

    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;
    public AprilTagDetection tag;

    public List<AprilTagDetection> detections;

    public void init(HardwareMap hwMap){
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hwMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(640, 480))
                .build();
    }

    public void update(){
        detections = tagProcessor.getDetections();
        tag = detections.isEmpty() ? null : detections.get(0);
    }

    public AprilTagDetection getTag() {
        return tag;
    }

    public List<AprilTagDetection> getDetections() {
        return tagProcessor.getDetections();
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
