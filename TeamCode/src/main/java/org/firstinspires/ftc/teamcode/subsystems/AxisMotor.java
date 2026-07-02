package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Одна ось наведения (pan или tilt) на паре goBILDA Yellow Jacket моторов,
 * сведённых через общую шестерню (leader + follower — оба мотора крутят одну
 * и ту же выходную ось, механически жёстко связаны через зубчатую передачу).
 *
 * Обобщённая версия проверенного {@code TurretMotor} из FTC-кода: угловой PIDF
 * с энкодерной обратной связью, статический friction-feedforward ({@code kF}),
 * gravity-feedforward ({@code kG}, для наклона) и soft-лимиты по углу.
 *
 * Угол читается только с энкодера leader — follower считается жёстко связанным
 * через шестерню и просто зеркалит мощность leader'а (без своего PIDF/лимита).
 *
 * Управление через {@code targetAngle} + PIDF, а не прямой {@code setPower}
 * (кроме калибровочных методов). Визуальный сервоинг в OpMode: ошибка камеры —
 * это угол в градусах, поэтому {@code setTargetAngle(getCurrentAngle() + tx)}
 * доворачивает ось ровно на ошибку, а PIDF плавно держит.
 *
 * Старт всегда из механического home — там обнуляется энкодер (leader и follower).
 */
public class AxisMotor {

    private final DcMotorEx leader;
    private final DcMotorEx follower;

    // PIDF — public для тюнинга через Dashboard / тестер.
    public double kP = 0.011;
    public double kI = 0.0;
    public double kD = 0.0007;
    public double kF = 0.09;   // static friction feedforward (signum)
    public double kG = 0.0;    // gravity feedforward (constant) — для tilt > 0

    private final double ticksPerDegree;
    private final double minAngle;
    private final double maxAngle;

    public double angleTolerance = 1.0; // ° — deadband удержания
    private static final double INTEGRAL_LIMIT = 1.0;
    private static final double OVERRIDE_POWER = 0.35;

    private double integral       = 0;
    private double lastError      = 0;
    private long   lastUpdateTime;
    private double targetAngle    = 0.0;

    /**
     * @param hw              hardware map
     * @param leaderName      имя leader-мотора (источник угла), напр. "panMotorLeft"
     * @param followerName    имя follower-мотора (зеркалит мощность leader'а), напр. "panMotorRight"
     * @param ticksPerDegree  тиков энкодера leader на градус выходной оси (калибровать!)
     * @param minAngle        нижний soft-лимит, °
     * @param maxAngle        верхний soft-лимит, °
     * @param leaderReverse   true → инвертировать направление leader-мотора
     * @param followerReverse true → инвертировать направление follower-мотора
     * @param resetEncoder    true → обнулить энкодер leader (старт из home)
     */
    public AxisMotor(HardwareMap hw, String leaderName, String followerName, double ticksPerDegree,
                     double minAngle, double maxAngle,
                     boolean leaderReverse, boolean followerReverse, boolean resetEncoder) {
        this.ticksPerDegree = ticksPerDegree;
        this.minAngle       = minAngle;
        this.maxAngle       = maxAngle;

        leader = hw.get(DcMotorEx.class, leaderName);
        leader.setDirection(leaderReverse ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        if (resetEncoder) {
            leader.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        leader.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower = hw.get(DcMotorEx.class, followerName);
        follower.setDirection(followerReverse ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        follower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        follower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lastUpdateTime = System.nanoTime();
        targetAngle    = resetEncoder ? 0.0 : getCurrentAngle();
    }

    // ── PIDF ─────────────────────────────────────────────────────────────────

    /** PIDF-выход для целевого/текущего угла, с friction- и gravity-feedforward. */
    public double calculatePIDF(double target, double current) {
        long now = System.nanoTime();
        double dt = (now - lastUpdateTime) / 1e9;
        lastUpdateTime = now;
        if (dt <= 0) return 0;

        double error = target - current;

        integral += error * dt;
        integral = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, integral));

        double derivative = (error - lastError) / dt;
        lastError = error;

        double pid = kP * error + kI * integral + kD * derivative;
        double ff  = (Math.abs(error) > angleTolerance) ? kF * Math.signum(error) : 0;
        return clampPower(pid + ff + kG);
    }

    /** Удерживать текущий targetAngle через PIDF. Вызывать раз в loop. */
    public void maintainTarget() {
        applyPower(calculatePIDF(targetAngle, getCurrentAngle()));
    }

    // ── Target control ───────────────────────────────────────────────────────

    public void setTargetAngle(double angle) {
        targetAngle = Math.max(minAngle, Math.min(maxAngle, angle));
    }

    public void returnToCenter() { setTargetAngle(0.0); }

    /** Доворот относительно текущего угла (визуальный сервоинг: delta = ошибка камеры). */
    public void nudgeTargetBy(double deltaDeg) {
        setTargetAngle(getCurrentAngle() + deltaDeg);
    }

    // ── Reads ────────────────────────────────────────────────────────────────

    public double  getCurrentAngle() { return leader.getCurrentPosition() / ticksPerDegree; }
    public double  getTargetAngle()  { return targetAngle; }
    public double  getMotorPower()   { return leader.getPower(); }
    public boolean atTarget()        { return Math.abs(targetAngle - getCurrentAngle()) < angleTolerance; }
    public double  getMinAngle()     { return minAngle; }
    public double  getMaxAngle()     { return maxAngle; }

    // ── Calibration / manual (для тестера) ─────────────────────────────────────

    /** Прямая мощность с учётом soft-лимитов, без PIDF. Сбрасывает интеграл. */
    public void manualOverride(double direction) {
        integral    = 0;
        lastError   = 0;
        targetAngle = getCurrentAngle();

        if (Math.abs(direction) > 0.1) {
            double power   = Math.signum(direction) * OVERRIDE_POWER;
            double current = getCurrentAngle();
            if ((power > 0 && current >= maxAngle) || (power < 0 && current <= minAngle)) {
                applyPower(0);
            } else {
                applyPower(power);
            }
        } else {
            applyPower(0);
        }
    }

    /** Сырая мощность без лимитов и энкодера — только калибровка направления. */
    public void manualRotateRaw(double power) {
        applyPower(power);
    }

    // ── Lifecycle ──────────────────────────────────────────────────────────────

    public void setPIDF(double p, double i, double d, double f) { kP = p; kI = i; kD = d; kF = f; }

    public void resetEncoder() {
        leader.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leader.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetAngle    = 0.0;
        integral       = 0;
        lastError      = 0;
        lastUpdateTime = System.nanoTime();
    }

    public void stop() {
        applyPower(0);
        integral       = 0;
        lastError      = 0;
        lastUpdateTime = System.nanoTime();
    }

    /** Единая точка выдачи мощности — зеркалит на leader и follower. */
    private void applyPower(double p) {
        leader.setPower(p);
        follower.setPower(p);
    }

    private double clampPower(double p) { return Math.max(-1.0, Math.min(1.0, p)); }
}
