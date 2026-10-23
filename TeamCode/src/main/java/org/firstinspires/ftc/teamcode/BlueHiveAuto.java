package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.LimelightColorDetection;

@Autonomous(name = "Blue Hive Auto")
public class BlueHiveAuto extends LimelightColorDetection {

    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initializeRobot();

        telemetry.addLine("Blue Hive Auto Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            LLResultTypes.FiducialResult tag = findBestTag();

            if (tag != null) {

                alignToTag(tag);

            } else {

                search();
            }

            telemetry.update();
        }

        stopDrive();
    }
}