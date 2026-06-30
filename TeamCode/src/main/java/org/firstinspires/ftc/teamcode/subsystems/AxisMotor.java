package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Одна ось наведения (pan или tilt) на goBILDA Yellow Jacket моторе с энкодером.
 *
 * Обобщённая версия проверенного {@code TurretMotor} из FTC-кода: угловой PIDF
 * с энкодерной обратной связью, статический friction-feedforward ({@code kF}),
 * gravity-feedforward ({@code kG}, для наклона) и soft-лимиты по углу.
 *
 * Управление через {@code targetAngle} + PIDF, а не прямой {@code setPower}
 * (кроме калибровочных методов). Визуальный сервоинг в OpMode: ошибка камеры —
 * это угол в градусах, поэтому {@code setTargetAngle(getCurrentAngle() + tx)}
 * доворачивает ось ровно на ошибку, а PIDF плавно держит.
 *
 * Старт всегда из механического home — там обнуляется энкодер.
 */
public class AxisMotor {

    private final DcMotorEx motor;

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
     * @param hw             hardware map
     * @param deviceName     имя мотора в конфигурации (напр. "panMotor")
     * @param ticksPerDegree тиков энкодера на градус выходной оси (калибровать!)
     * @param minAngle       нижний soft-лимит, °
     * @param maxAngle       верхний soft-лимит, °
     * @param reverse        true → инвертировать направление мотора
     * @param resetEncoder   true → обнулить энкодер (старт из home)
     */
    public AxisMotor(HardwareMap hw, String deviceName, double ticksPerDegree,
                     double minAngle, double maxAngle, boolean reverse, boolean resetEncoder) {
        this.ticksPerDegree = ticksPerDegree;
        this.minAngle       = minAngle;
        this.maxAngle       = maxAngle;

        motor = hw.get(DcMotorEx.class, deviceName);
        motor.setDirection(reverse ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        if (resetEncoder) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        motor.setPower(calculatePIDF(targetAngle, getCurrentAngle()));
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

    public double  getCurrentAngle() { return motor.getCurrentPosition() / ticksPerDegree; }
    public double  getTargetAngle()  { return targetAngle; }
    public double  getMotorPower()   { return motor.getPower(); }
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
                motor.setPower(0);
            } else {
                motor.setPower(power);
            }
        } else {
            motor.setPower(0);
        }
    }

    /** Сырая мощность без лимитов и энкодера — только калибровка направления. */
    public void manualRotateRaw(double power) {
        motor.setPower(power);
    }

    // ── Lifecycle ──────────────────────────────────────────────────────────────

    public void setPIDF(double p, double i, double d, double f) { kP = p; kI = i; kD = d; kF = f; }

    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetAngle    = 0.0;
        integral       = 0;
        lastError      = 0;
        lastUpdateTime = System.nanoTime();
    }

    public void stop() {
        motor.setPower(0);
        integral       = 0;
        lastError      = 0;
        lastUpdateTime = System.nanoTime();
    }

    private double clampPower(double p) { return Math.max(-1.0, Math.min(1.0, p)); }
}
