package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Low-level turret motor control: PIDF, encoder, manual movement.
 * No knowledge of field coordinates, vision, or ballistics.
 */
public class TurretMotor {

    private final DcMotorEx turretMotor;

    // PIDF coefficients — public for Dashboard / TurretTester tuning
    public double kP = 0.011;
    public double kI = 0.0;
    public double kD = 0.0007;
    public double kF = 0.09;

    private double integral    = 0;
    private double lastError   = 0;
    private long   lastUpdateTime;

    private double targetAngle = 0.0;

    // Physical constants
    public static double TICKS_PER_DEGREE = 3.15;
    public static final double MAX_ANGLE       =  120.0;
    public static final double MIN_ANGLE       = -180.0;
    public static final double ANGLE_TOLERANCE =    2.0;

    private static final double MANUAL_STEP    =  3.0;
    private static final double OVERRIDE_POWER =  0.4;
    private static final double INTEGRAL_LIMIT =  1.0;

    /**
     * @param hardwareMap   FTC hardware map
     * @param resetEncoder  true → STOP_AND_RESET_ENCODER (Auto / fresh start)
     *                      false → keep encoder value from previous OpMode (TeleOp after Auto)
     */
    public TurretMotor(HardwareMap hardwareMap, boolean resetEncoder) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setDirection(DcMotor.Direction.REVERSE); // right = positive angles
        if (resetEncoder) {
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lastUpdateTime = System.nanoTime();
        // TeleOp: start from wherever turret physically is (Auto left it there)
        targetAngle = resetEncoder ? 0.0 : getCurrentAngle();
    }

    // ── PIDF ─────────────────────────────────────────────────────────────────

    /** Calculate PIDF output for given target/current angles. */
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
        // Static friction feedforward: applied only when outside tolerance
        double ff  = (Math.abs(error) > ANGLE_TOLERANCE) ? kF * Math.signum(error) : 0;
        return Math.max(-1.0, Math.min(1.0, pid + ff));
    }

    /** Apply calculated power directly to the motor. */
    public void applyPower(double power) {
        turretMotor.setPower(power);
    }

    // ── Position reads ───────────────────────────────────────────────────────

    public double getCurrentAngle()    { return turretMotor.getCurrentPosition() / TICKS_PER_DEGREE; }
    public double getCurrentPosition() { return turretMotor.getCurrentPosition(); }
    public double getTargetPosition()  { return targetAngle * TICKS_PER_DEGREE; }
    public double getTargetAngle()     { return targetAngle; }
    public double getMotorPower()      { return turretMotor.getPower(); }
    public boolean atTarget()          { return Math.abs(targetAngle - getCurrentAngle()) < ANGLE_TOLERANCE; }
    public boolean isCentered()        { return Math.abs(getCurrentAngle()) < ANGLE_TOLERANCE; }

    // ── Target control ───────────────────────────────────────────────────────

    public void setTargetAngle(double angle) {
        targetAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angle));
    }

    public void returnToCenter() { setTargetAngle(0.0); }

    /** Hold current targetAngle using PIDF. */
    public void maintainTarget() {
        double power = calculatePIDF(targetAngle, getCurrentAngle());
        turretMotor.setPower(power);
    }

    // ── Manual control ───────────────────────────────────────────────────────

    /** Joystick-driven: incrementally moves targetAngle, PIDF holds it. */
    public void manualControl(double joystickInput) {
        if (Math.abs(joystickInput) > 0.1) {
            targetAngle += joystickInput * MANUAL_STEP;
            targetAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, targetAngle));
        }
        double power = calculatePIDF(targetAngle, getCurrentAngle());
        turretMotor.setPower(power);
    }

    /** Direct power bypass — no PIDF. Resets integral to avoid fight on return. */
    public void manualOverride(double direction) {
        integral    = 0;
        lastError   = 0;
        targetAngle = getCurrentAngle(); // sync so PID won't jerk when re-enabled

        if (Math.abs(direction) > 0.1) {
            double power = Math.signum(direction) * OVERRIDE_POWER;
            double current = getCurrentAngle();
            if ((power > 0 && current >= MAX_ANGLE) || (power < 0 && current <= MIN_ANGLE)) {
                turretMotor.setPower(0);
            } else {
                turretMotor.setPower(power);
            }
        } else {
            turretMotor.setPower(0);
        }
    }

    /** Sync targetAngle to physical position before switching to manual mode. */
    public void syncManualTarget() {
        targetAngle    = getCurrentAngle();
        integral       = 0;
        lastError      = 0;
        lastUpdateTime = System.nanoTime();
    }

    /** Raw motor power for calibration (no encoder, no limits). */
    public void manualRotateRaw(double power) {
        turretMotor.setPower(power);
    }

    // ── Lifecycle ────────────────────────────────────────────────────────────

    public void setPIDF(double p, double i, double d, double f) {
        kP = p; kI = i; kD = d; kF = f;
    }

    public void setPID(double p, double i, double d) {
        setPIDF(p, i, d, 0.0);
    }

    public void resetEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetAngle    = 0.0;
        integral       = 0;
        lastError      = 0;
        lastUpdateTime = System.nanoTime();
    }

    public void stop() {
        turretMotor.setPower(0);
        integral       = 0;
        lastError      = 0;
        lastUpdateTime = System.nanoTime();
    }
}
