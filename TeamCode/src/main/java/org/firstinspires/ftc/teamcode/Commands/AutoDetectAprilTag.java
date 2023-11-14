package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Lib.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Lib.GlobalData;
import org.firstinspires.ftc.teamcode.Lib.k;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class AutoDetectAprilTag extends CommandBase {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    int cameraMonitorViewId;
    double m_timeOut;
    CommandOpMode m_opMode;
    ElapsedTime m_elapsedTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    AprilTagDetection tagOfInterest = null;

    public AutoDetectAprilTag(CommandOpMode _opMode, double _timeOut) {
        m_timeOut = _timeOut;
        m_opMode = _opMode;
    }

    @Override
    public void initialize() {
        cameraMonitorViewId = m_opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", m_opMode.hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(m_opMode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(k.CAMERA.tagsize, k.CAMERA.fx, k.CAMERA.fy, k.CAMERA.cx, k.CAMERA.cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        // NOTE: this must be called *before* you call startStreaming(...)
        //camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.NATIVE_VIEW);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {

            }
        });
        m_elapsedTimer.reset();
    }

    @Override
    public void execute() {
//        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        if (currentDetections.size() != 0) {
            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == GlobalData.tagOfInterest) {
                    tagOfInterest = tag;
                    break;
                }
            }
        }
        m_opMode.sleep(20);

        if (tagOfInterest != null) { // We found a tag so set the GlobalVariables
            // Set the global variable numbers
            GlobalData.AprilTag_X = tagOfInterest.pose.x * 1000;
            GlobalData.AprilTag_Y = tagOfInterest.pose.y * 1000;
            GlobalData.AprilTag_Z = tagOfInterest.pose.z * 1000 * 1.25;
            //GlobalData.tagPoseZ = tagOfInterest.pose.z;
            GlobalData.AprilTagBearing = Math.toDegrees(Math.atan(GlobalData.AprilTag_X / GlobalData.AprilTag_Z));
            GlobalData.AprilTagRange = Math.sqrt(GlobalData.AprilTag_Z * GlobalData.AprilTag_Z + GlobalData.AprilTag_X * GlobalData.AprilTag_X);

        } else { // Did not find the tag so add some default data.
            GlobalData.AprilTag_X = 0;
            GlobalData.AprilTag_Y = 0;
            GlobalData.AprilTag_Z = 0;
            GlobalData.AprilTagBearing = 5;
            GlobalData.AprilTagRange = 150;
        }
    }
    @Override
    public boolean isFinished() {
        if (tagOfInterest != null && m_elapsedTimer.seconds() > m_timeOut) {
            return true;
        }
        return false;
    }
    private double calcWallTagToStackAngle(double _x, double _z){
        switch(GlobalData.TeamColor){
            case RED:
                break;
            case BLUE:
                break;
            default:
                break;
        }
        return 0;
    }
    private double calcWallTagToStackDistance(double _x, double _z){
        return 0;
    }
}
