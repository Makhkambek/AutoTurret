package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.logic.TargetFilter;
import org.firstinspires.ftc.teamcode.logic.TurretFSM;
import org.firstinspires.ftc.teamcode.subsystems.AxisMotor;
import org.firstinspires.ftc.teamcode.subsystems.LimelightColorTracker;

/**
 * AutoTurret MVP — главный OpMode.
 *
 * Поток: Limelight (color) → TargetFilter (persistence+grace) → TurretFSM →
 * AxisMotor pan/tilt (angle-PIDF). Лазер всегда включён (отдельно, не управляем).
 * «Выстрел» при LOCKED = событие в телеметрии (shotCount).
 *
 * Визуальный сервоинг: tx/ty — угловая ошибка в °, поэтому доворачиваем ось на
 * долю ошибки ({@code VISUAL_GAIN}), PIDF держит. При потере цели ось остаётся
 * на последнем угле (maintainTarget), затем FSM уходит в SEARCH.
 *
 * Геймпад: A = старт наведения, B = стоп (IDLE).
 */
@TeleOp(name = "AutoTurret", group = "AutoTurret")
public class AutoTurretOpMode extends LinearOpMode {

    // ── Калибровка (подогнать под реальную сборку!) ───────────────────────────
    static final double TICKS_PER_DEG_PAN  = 5.97;
    static final double TICKS_PER_DEG_TILT = 5.97;
    static final double PAN_MIN  = -135, PAN_MAX  = 135;   // 270° охват
    static final double TILT_MIN =  -15, TILT_MAX =  85;
    // Каждая ось — 2 мотора (left/right) через общую шестерню. Флипнуть отдельно
    // ту сторону, что крутит не туда (см. AxisMotor Tester).
    static final boolean PAN_LEFT_REVERSE   = false;
    static final boolean PAN_RIGHT_REVERSE  = false;
    static final boolean TILT_LEFT_REVERSE  = false;
    static final boolean TILT_RIGHT_REVERSE = false;
    static final double TILT_GRAVITY_FF = 0.08; // удержание наклона от провисания

    // ── Поведение ──────────────────────────────────────────────────────────────
    static final int    FRAMES_TO_CONFIRM = 3;
    static final long   GRACE_MS          = 300;
    static final double DEADBAND_DEG       = 1.0;
    static final int    FRAMES_TO_LOCK     = 3;
    static final double VISUAL_GAIN        = 0.6;   // доля ошибки за loop (плавность)
    static final double SCAN_STEP_DEG      = 0.6;   // скорость скана в SEARCH

    private AxisMotor pan, tilt;
    private LimelightColorTracker tracker;
    private TargetFilter filter;
    private TurretFSM fsm;

    private int scanDir = 1;

    @Override
    public void runOpMode() {
        // Старт ИЗ HOME (центр) — энкодеры обнуляются здесь.
        pan  = new AxisMotor(hardwareMap, "panMotorLeft", "panMotorRight", TICKS_PER_DEG_PAN,
                             PAN_MIN, PAN_MAX, PAN_LEFT_REVERSE, PAN_RIGHT_REVERSE, true);
        tilt = new AxisMotor(hardwareMap, "tiltMotorLeft", "tiltMotorRight", TICKS_PER_DEG_TILT,
                             TILT_MIN, TILT_MAX, TILT_LEFT_REVERSE, TILT_RIGHT_REVERSE, true);
        tilt.kG = TILT_GRAVITY_FF;

        tracker = new LimelightColorTracker();
        tracker.init(hardwareMap);

        filter = new TargetFilter(FRAMES_TO_CONFIRM, GRACE_MS);
        fsm    = new TurretFSM(DEADBAND_DEG, FRAMES_TO_LOCK);

        telemetry.addData("Status", "Initialized — A=start, B=stop");
        telemetry.addData("Home", "турель должна стоять в центре!");
        telemetry.update();

        waitForStart();
        fsm.start();

        while (opModeIsActive()) {
            // 1. Зрение → фильтр → стабильная видимость
            boolean rawVisible = tracker.hasRawTarget();
            filter.update(rawVisible, System.currentTimeMillis());
            boolean visible = filter.isVisible();

            double tx = tracker.getTx();
            double ty = tracker.getTy();

            // 2. FSM
            TurretFSM.State state = fsm.update(visible, tx, ty);

            // 3. Геймпад
            if (gamepad1.a) fsm.start();
            if (gamepad1.b) fsm.stop();

            // 4. Привод осей по состоянию
            switch (state) {
                case IDLE:
                    pan.stop();
                    tilt.maintainTarget(); // держит наклон (kG)
                    break;

                case SEARCH:
                    scan();
                    tilt.maintainTarget();
                    break;

                case TRACK:
                case LOCKED:
                    // tx/ty — угловая ошибка: доворачиваем на долю, PIDF держит.
                    pan.nudgeTargetBy(tx * VISUAL_GAIN);
                    tilt.nudgeTargetBy(ty * VISUAL_GAIN);
                    pan.maintainTarget();
                    tilt.maintainTarget();
                    break;
            }

            // 5. «Выстрел» — разово при захвате
            if (fsm.shouldFire()) {
                telemetry.log().add(String.format(
                    "SHOT #%d  t=%dms  tx=%.1f ty=%.1f ta=%.2f",
                    fsm.getShotCount(), System.currentTimeMillis(), tx, ty, tracker.getTa()));
            }

            // 6. Телеметрия
            telemetry.addData("State", state);
            telemetry.addData("visible (raw/stable)", "%b / %b", rawVisible, visible);
            telemetry.addData("tx/ty/ta", "%.1f / %.1f / %.2f", tx, ty, tracker.getTa());
            telemetry.addData("pan deg (cur→tgt)", "%.1f → %.1f", pan.getCurrentAngle(), pan.getTargetAngle());
            telemetry.addData("tilt deg (cur→tgt)", "%.1f → %.1f", tilt.getCurrentAngle(), tilt.getTargetAngle());
            telemetry.addData("shots", fsm.getShotCount());
            telemetry.update();
        }

        pan.stop();
        tilt.stop();
        tracker.stop();
    }

    /** Скан pan между лимитами по энкодеру. */
    private void scan() {
        double a = pan.getCurrentAngle();
        if (a >= PAN_MAX) scanDir = -1;
        if (a <= PAN_MIN) scanDir =  1;
        pan.setTargetAngle(pan.getTargetAngle() + scanDir * SCAN_STEP_DEG);
        pan.maintainTarget();
    }
}
