package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LimelightColorTracker;

/**
 * Проверка детекции шара ДО запуска AutoTurret.
 *
 * Запусти, поводи шаром перед камерой:
 *  - hasRawTarget = true когда шар в кадре;
 *  - tx меняется влево/вправо, ty вверх/вниз;
 *  - ta растёт при приближении.
 *
 * Если детекции нет — донастрой HSV/exposure в web-UI Limelight (pipeline 0 = Color).
 * Если ловит мусор — подними MIN_AREA или сузь цвет.
 */
@TeleOp(name = "Limelight Color Tester", group = "AutoTurret")
public class LimelightColorTester extends LinearOpMode {

    @Override
    public void runOpMode() {
        LimelightColorTracker tracker = new LimelightColorTracker();
        tracker.init(hardwareMap);

        telemetry.addData("Status", "Pipeline 0 = Color. Поводи шаром перед камерой.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("hasRawTarget", tracker.hasRawTarget());
            telemetry.addData("tx", "%.2f", tracker.getTx());
            telemetry.addData("ty", "%.2f", tracker.getTy());
            telemetry.addData("ta (%)", "%.2f", tracker.getTa());
            telemetry.update();
        }

        tracker.stop();
    }
}
