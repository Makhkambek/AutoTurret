package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Калибровка осей pan/tilt ДО запуска AutoTurret.
 *
 * Каждая ось крутится ДВУМЯ моторами (left/right), сведёнными через общую
 * шестерню. Тестер управляет всеми 4 моторами НЕЗАВИСИМО друг от друга — это
 * нужно, чтобы покрутить каждую сторону по отдельности и по знаку тиков
 * определить, какому мотору нужен reverse, до того как включать общий PIDF.
 *
 *  - left stick X  → panMotorLeft   (сырая мощность)
 *  - left stick Y  → panMotorRight  (сырая мощность)
 *  - right stick X → tiltMotorLeft  (сырая мощность)
 *  - right stick Y → tiltMotorRight (сырая мощность)
 *  - энкодеры обнуляются на старте (home).
 *
 * Калибровка TICKS_PER_DEG: повернуть ось ровно на 90° по разметке (крутить
 * обоими моторами оси вместе), снять ticks с телеметрии → TICKS_PER_DEG =
 * ticks / 90. Вписать в AutoTurretOpMode.
 * Направление: если мотор крутит общую шестерню «не туда» — выставить
 * соответствующий *_REVERSE в AutoTurretOpMode.
 */
@TeleOp(name = "AxisMotor Tester", group = "AutoTurret")
public class AxisMotorTester extends LinearOpMode {

    static final double TEST_POWER = 0.3;

    @Override
    public void runOpMode() {
        DcMotorEx panLeft   = hardwareMap.get(DcMotorEx.class, "panMotorLeft");
        DcMotorEx panRight  = hardwareMap.get(DcMotorEx.class, "panMotorRight");
        DcMotorEx tiltLeft  = hardwareMap.get(DcMotorEx.class, "tiltMotorLeft");
        DcMotorEx tiltRight = hardwareMap.get(DcMotorEx.class, "tiltMotorRight");

        for (DcMotorEx m : new DcMotorEx[]{panLeft, panRight, tiltLeft, tiltRight}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addData("Status", "Home = центр. LX=panL LY=panR RX=tiltL RY=tiltR");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double panLeftPow   =  gamepad1.left_stick_x  * TEST_POWER;
            double panRightPow  =  gamepad1.left_stick_y  * TEST_POWER;
            double tiltLeftPow  =  gamepad1.right_stick_x * TEST_POWER;
            double tiltRightPow = -gamepad1.right_stick_y * TEST_POWER;

            panLeft.setPower(panLeftPow);
            panRight.setPower(panRightPow);
            tiltLeft.setPower(tiltLeftPow);
            tiltRight.setPower(tiltRightPow);

            telemetry.addLine("Крути КАЖДЫЙ мотор по отдельности, смотри знак ticks");
            telemetry.addLine("Затем крути ось ровно на 90° → TICKS_PER_DEG = ticks / 90");
            telemetry.addData("panLeft   ticks/power",  "%d / %.2f", panLeft.getCurrentPosition(),   panLeftPow);
            telemetry.addData("panRight  ticks/power",  "%d / %.2f", panRight.getCurrentPosition(),  panRightPow);
            telemetry.addData("tiltLeft  ticks/power",  "%d / %.2f", tiltLeft.getCurrentPosition(),  tiltLeftPow);
            telemetry.addData("tiltRight ticks/power",  "%d / %.2f", tiltRight.getCurrentPosition(), tiltRightPow);
            telemetry.update();
        }

        panLeft.setPower(0);
        panRight.setPower(0);
        tiltLeft.setPower(0);
        tiltRight.setPower(0);
    }
}
