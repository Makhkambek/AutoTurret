package org.firstinspires.ftc.teamcode.logic;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Before;
import org.junit.Test;

/**
 * TargetFilter: persistence (N-frame confirm) + grace (hold through brief loss).
 *
 * Контракт:
 *  - update(rawVisible, nowMs) вызывается раз на каждый Limelight poll.
 *  - isVisible() = стабильная видимость для FSM.
 *  - Захват: нужно framesToConfirm подряд rawVisible=true.
 *  - Потеря: после подтверждения короткие пропуски (< graceMs) НЕ сбрасывают видимость.
 *  - В пределах grace цель вернулась → видимость держится без повторного подтверждения.
 *  - grace истёк → невидим; повторный захват снова требует framesToConfirm кадров.
 *
 * framesToConfirm = 3, graceMs = 300.
 */
public class TargetFilterTest {

    private TargetFilter filter;

    @Before
    public void setUp() {
        filter = new TargetFilter(3, 300);
    }

    @Test
    public void startsNotVisible() {
        assertFalse(filter.isVisible());
    }

    @Test
    public void singleDetectionNotEnough() {
        filter.update(true, 0);
        assertFalse(filter.isVisible());
    }

    @Test
    public void twoDetectionsNotEnough() {
        filter.update(true, 0);
        filter.update(true, 10);
        assertFalse(filter.isVisible());
    }

    @Test
    public void threeConsecutiveDetectionsConfirm() {
        filter.update(true, 0);
        filter.update(true, 10);
        filter.update(true, 20);
        assertTrue(filter.isVisible());
    }

    @Test
    public void flickerNeverConfirms() {
        // true,false,true,false — никогда нет 3 подряд
        filter.update(true, 0);
        filter.update(false, 10);
        filter.update(true, 20);
        filter.update(false, 30);
        filter.update(true, 40);
        assertFalse(filter.isVisible());
    }

    @Test
    public void briefLossWithinGraceStaysVisible() {
        confirmVisible(0);                 // подтверждён к t=20
        filter.update(false, 120);         // пропуск через 100мс < 300мс grace
        assertTrue(filter.isVisible());
    }

    @Test
    public void lossBeyondGraceDropsVisibility() {
        confirmVisible(0);                 // последний true в t=20
        filter.update(false, 400);         // 380мс > 300мс grace
        assertFalse(filter.isVisible());
    }

    @Test
    public void returnWithinGraceResumesWithoutReconfirm() {
        confirmVisible(0);                 // visible, последний true t=20
        filter.update(false, 100);         // пропуск, в пределах grace
        assertTrue(filter.isVisible());
        filter.update(true, 150);          // цель вернулась — сразу видим, без 3 кадров
        assertTrue(filter.isVisible());
    }

    @Test
    public void afterGraceExpiryReacquireNeedsReconfirm() {
        confirmVisible(0);
        filter.update(false, 400);         // grace истёк → невидим
        assertFalse(filter.isVisible());

        filter.update(true, 410);          // 1 кадр — недостаточно
        assertFalse(filter.isVisible());
        filter.update(true, 420);          // 2 кадра
        assertFalse(filter.isVisible());
        filter.update(true, 430);          // 3 кадра → снова видим
        assertTrue(filter.isVisible());
    }

    @Test
    public void resetClearsState() {
        confirmVisible(0);
        assertTrue(filter.isVisible());
        filter.reset();
        assertFalse(filter.isVisible());
        filter.update(true, 500);          // после reset нужен повторный захват
        assertFalse(filter.isVisible());
    }

    /** Helper: три подряд true → подтверждённая видимость. Последний true в startMs+20. */
    private void confirmVisible(long startMs) {
        filter.update(true, startMs);
        filter.update(true, startMs + 10);
        filter.update(true, startMs + 20);
    }
}
