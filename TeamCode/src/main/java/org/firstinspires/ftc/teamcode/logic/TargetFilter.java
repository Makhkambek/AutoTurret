package org.firstinspires.ftc.teamcode.logic;

/**
 * Стабилизирует сырой сигнал видимости от Limelight.
 *
 * Две защиты (адаптировано из persistence-фильтра FTC Vision.java):
 *  - <b>Persistence:</b> захват подтверждается только после {@code framesToConfirm}
 *    последовательных кадров с целью → убирает одиночные блики.
 *  - <b>Grace:</b> после подтверждения короткие пропуски (&lt; {@code graceMs})
 *    не сбрасывают видимость → турель держит цель сквозь мерцание/кратковременный
 *    заслон. Если цель вернулась в пределах grace — видимость продолжается без
 *    повторного подтверждения. Когда grace истёк, повторный захват снова требует
 *    {@code framesToConfirm} кадров.
 *
 * Чистая логика, без железа: время передаётся в {@link #update(boolean, long)}
 * параметром {@code nowMs} → детерминированно тестируется.
 */
public class TargetFilter {

    private final int  framesToConfirm;
    private final long graceMs;

    private int     consecutiveDetections = 0;
    private boolean confirmed             = false;
    private long    lastSeenMs            = 0;

    public TargetFilter(int framesToConfirm, long graceMs) {
        this.framesToConfirm = framesToConfirm;
        this.graceMs         = graceMs;
    }

    /**
     * Обновляет фильтр. Вызывать ровно раз на каждый Limelight poll.
     *
     * @param rawVisible сырой признак «цель в кадре» из Limelight ({@code tv})
     * @param nowMs      текущее время в миллисекундах (монотонное)
     */
    public void update(boolean rawVisible, long nowMs) {
        if (rawVisible) {
            if (confirmed) {
                // Уже видимы — просто продлеваем grace-окно.
                lastSeenMs = nowMs;
            } else {
                consecutiveDetections++;
                if (consecutiveDetections >= framesToConfirm) {
                    confirmed  = true;
                    lastSeenMs = nowMs;
                }
            }
        } else {
            consecutiveDetections = 0;
            // Подтверждённую видимость держим, пока не вышли за grace-окно.
            if (confirmed && (nowMs - lastSeenMs) > graceMs) {
                confirmed = false;
            }
        }
    }

    /** @return стабильная видимость для FSM (с учётом persistence + grace). */
    public boolean isVisible() {
        return confirmed;
    }

    /** Сбрасывает состояние (смена цели / рестарт). */
    public void reset() {
        consecutiveDetections = 0;
        confirmed             = false;
        lastSeenMs            = 0;
    }
}
