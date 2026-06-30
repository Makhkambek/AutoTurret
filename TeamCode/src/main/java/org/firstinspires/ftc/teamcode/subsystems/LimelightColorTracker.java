package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Обёртка над Limelight 3A в режиме <b>Color pipeline</b> (pipeline 0).
 *
 * Limelight сам находит крупнейший цветной контур и считает {@code tx/ty/ta/tv} —
 * здесь только читаем готовые значения. Никакого кода на камере: HSV/exposure
 * настраиваются мышкой в web-UI.
 *
 * Что делает класс (по образцу FTC {@code Vision.java}):
 *  - кэширование результата на один loop (избегаем повторных I2C/USB чтений);
 *  - detection отключения камеры через {@code getStaleness()} (&gt; 500мс = мертва);
 *  - сырой признак цели + tx/ty/ta.
 *
 * Persistence/grace СОЗНАТЕЛЬНО вынесены в {@link org.firstinspires.ftc.teamcode.logic.TargetFilter}
 * — чистая логика тестируется отдельно от железа.
 */
public class LimelightColorTracker {

    /** Минимальная площадь цели (% кадра) — отсекает шум/далёкую мелочь. */
    public static double MIN_AREA = 0.05;

    private static final long CACHE_VALIDITY_MS       = 15;   // ~один poll при 100Hz
    private static final long CAMERA_DISCONNECT_MS    = 500;
    private static final int  COLOR_PIPELINE          = 0;

    private Limelight3A limelight;
    private boolean     active = false;

    private LLResult cached    = null;
    private long     cacheTime = 0;

    public void init(HardwareMap hw) {
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(COLOR_PIPELINE);
        limelight.start();
        active = true;
    }

    public void start() {
        if (limelight != null) limelight.start();
        active = true;
    }

    public void stop() {
        if (limelight != null) limelight.stop();
        active = false;
    }

    public boolean isActive() { return active; }

    /** Свежий валидный результат с кэшем и disconnect-детектом, или null. */
    private LLResult fresh() {
        long now = System.currentTimeMillis();
        if (cached != null && (now - cacheTime) < CACHE_VALIDITY_MS) return cached;

        cacheTime = now;
        cached    = null;
        if (!active || limelight == null) return null;

        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return null;
        // getLatestResult() отдаёт старый кэш даже когда камера отвалилась —
        // ловим по возрасту данных.
        if (r.getStaleness() > CAMERA_DISCONNECT_MS) return null;

        cached = r;
        return r;
    }

    /** Сырой признак «цель в кадре» (для подачи в TargetFilter). */
    public boolean hasRawTarget() {
        LLResult r = fresh();
        return r != null && r.getTa() > MIN_AREA;
    }

    /** Горизонтальная ошибка, ° (+ = цель справа). 0 если цели нет. */
    public double getTx() { LLResult r = fresh(); return r != null ? r.getTx() : 0.0; }

    /** Вертикальная ошибка, ° (+ = цель выше). 0 если цели нет. */
    public double getTy() { LLResult r = fresh(); return r != null ? r.getTy() : 0.0; }

    /** Площадь цели, % кадра. 0 если цели нет. */
    public double getTa() { LLResult r = fresh(); return r != null ? r.getTa() : 0.0; }
}
