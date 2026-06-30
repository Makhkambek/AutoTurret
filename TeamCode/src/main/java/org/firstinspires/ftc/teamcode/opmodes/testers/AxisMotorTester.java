package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Калибровка осей pan/tilt ДО запуска AutoTurret.
 *
 * Зачем: проверить направление моторов и измерить TICKS_PER_DEG.
 *  - left stick X  → pan (сырая мощность)
 *  - right stick Y → tilt (сырая мощность)
 *  - энкодеры обнуляются на старте (home).
 *
 * Калибровка TICKS_PER_DEG: повернуть ось ровно на 90° по разметке, снять ticks
 * с телеметрии → TICKS_PER_DEG = ticks / 90. Вписать в AutoTurretOpMode.
 * Направление: если ось едет «не туда» — выставить *_REVERSE в AutoTurretOpMode.
 */
@TeleOp(name = "AxisMotor Tester", group = "AutoTurret")
public class AxisMotorTester extends LinearOpMode {

    static final double TEST_POWER = 0.3;

    @Override
    public void runOpMode() {
        DcMotorEx panMotor  = hardwareMap.get(DcMotorEx.class, "panMotor");
        DcMotorEx tiltMotor = hardwareMap.get(DcMotorEx.class, "tiltMotor");

        for (DcMotorEx m : new DcMotorEx[]{panMotor, tiltMotor}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addData("Status", "Home = центр. left X = pan, right Y = tilt");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double panPow  =  gamepad1.left_stick_x  * TEST_POWER;
            double tiltPow = -gamepad1.right_stick_y * TEST_POWER;
            panMotor.setPower(panPow);
            tiltMotor.setPower(tiltPow);

            int panTicks  = panMotor.getCurrentPosition();
            int tiltTicks = tiltMotor.getCurrentPosition();

            telemetry.addLine("Поверни ось на 90° → TICKS_PER_DEG = ticks / 90");
            telemetry.addData("pan  ticks",  panTicks);
            telemetry.addData("tilt ticks",  tiltTicks);
            telemetry.addData("pan  power",  "%.2f", panPow);
            telemetry.addData("tilt power",  "%.2f", tiltPow);
            telemetry.update();
        }

        panMotor.setPower(0);
        tiltMotor.setPower(0);
    }
}
