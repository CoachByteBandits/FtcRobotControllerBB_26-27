
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Shooter Test")
public class AutoShooterTest extends LinearOpMode {

    private DcMotorEx shooter;
    private static final double TICKS_PER_REV = 28.0;

    @Override
    public void runOpMode() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        waitForStart();

        // shooter power: 50%
        shooter.setPower(0.5);

        while (opModeIsActive()) {
            double ticksPerSecond = shooter.getVelocity();
            double rpm = (ticksPerSecond / TICKS_PER_REV) * 60.0;

            // Telemetry
            telemetry.addData("Shooter Power", shooter.getPower());
            telemetry.addData("Shooter Velocity", "%.2f ticks/sec", ticksPerSecond);
            telemetry.addData("Shooter RPM", "%.2f RPM", rpm);

            telemetry.update();
        }

        // Stop shooter motor
        shooter.setPower(0);
    }
}
