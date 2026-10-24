package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import androidx.appcompat.app.AlertDialog;


@Autonomous(name = "Limelight Tag Align", group = "Vision")
public class LimeLight_AT_Detection extends LinearOpMode {
    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult tag
                    : result.getFiducialResults()) {

                if (tag.getFiducialId() != 1) {
                    continue;
                }

                double tx = tag.getTargetXDegrees();
                double ty = tag.getTargetYDegrees();
                double ta = tag.getTargetArea();

                Pose3D targetRobot = tag.getTargetPoseRobotSpace();

                telemetry.addData("tx (deg)", tx);
                telemetry.addData("ty (deg)", ty);
                telemetry.addData("ta (%)", ta);

                telemetry.addData("X (m)",
                        targetRobot.getPosition().x);
                telemetry.addData("Y (m)",
                        targetRobot.getPosition().y);
                telemetry.addData("Z (m)",
                        targetRobot.getPosition().z);

                telemetry.addData("Yaw (deg)",
                        targetRobot.getOrientation()
                                .getYaw(AngleUnit.DEGREES));

                telemetry.update();
            }
        }


    }
}
