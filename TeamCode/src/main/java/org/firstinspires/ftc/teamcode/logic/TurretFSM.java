package org.firstinspires.ftc.teamcode.logic;

/**
 * Конечный автомат наведения турели.
 *
 * <pre>
 *   IDLE → SEARCH → TRACK → LOCKED → (FIRE) → обратно в TRACK/SEARCH
 * </pre>
 *
 *  - <b>IDLE:</b> старт не нажат — турель стоит. {@link #start()} → SEARCH.
 *  - <b>SEARCH:</b> цели нет; как только {@code visible} → TRACK.
 *  - <b>TRACK:</b> ведём по ошибке камеры; потеря цели → SEARCH.
 *  - <b>LOCKED:</b> цель в deadband {@code framesToLock} кадров подряд. При входе —
 *    разовый «выстрел» ({@link #shouldFire()} == true, {@code shotCount++}).
 *    Цель ушла из центра → TRACK (перевзвод); потеря → SEARCH.
 *
 * «Выстрел» срабатывает ровно один раз на каждый захват центра: пока цель удержана
 * в LOCKED, повторно не палит; ушла и вернулась — новый выстрел.
 *
 * Чистая логика: моторных команд не выдаёт — только состояние и события.
 * Привязка к моторам/телеметрии — в OpMode.
 */
public class TurretFSM {

    public enum State { IDLE, SEARCH, TRACK, LOCKED }

    private final double deadbandDeg;
    private final int    framesToLock;

    private State   state         = State.IDLE;
    private int     centeredCount = 0;
    private boolean fireEvent     = false;
    private int     shotCount     = 0;

    public TurretFSM(double deadbandDeg, int framesToLock) {
        this.deadbandDeg  = deadbandDeg;
        this.framesToLock = framesToLock;
    }

    /** IDLE → SEARCH (начать наведение). */
    public void start() {
        state         = State.SEARCH;
        centeredCount = 0;
        fireEvent     = false;
    }

    /** В IDLE из любого состояния (стоп). */
    public void stop() {
        state         = State.IDLE;
        centeredCount = 0;
        fireEvent     = false;
    }

    /**
     * Один тик автомата.
     *
     * @param visible стабильная видимость цели (из {@link TargetFilter})
     * @param tx      горизонтальная ошибка камеры, градусы
     * @param ty      вертикальная ошибка камеры, градусы
     * @return текущее состояние после перехода
     */
    public State update(boolean visible, double tx, double ty) {
        fireEvent = false; // событие выстрела действует только на тик входа в LOCKED

        switch (state) {
            case IDLE:
                break; // ждём start()

            case SEARCH:
                if (visible) {
                    state         = State.TRACK;
                    centeredCount = 0;
                }
                break;

            case TRACK:
                if (!visible) {
                    state         = State.SEARCH;
                    centeredCount = 0;
                    break;
                }
                if (isCentered(tx, ty)) {
                    centeredCount++;
                    if (centeredCount >= framesToLock) {
                        state     = State.LOCKED;
                        fireEvent = true;
                        shotCount++;
                    }
                } else {
                    centeredCount = 0;
                }
                break;

            case LOCKED:
                if (!visible) {
                    state         = State.SEARCH;
                    centeredCount = 0;
                } else if (!isCentered(tx, ty)) {
                    state         = State.TRACK; // ушла из центра — перевзвод выстрела
                    centeredCount = 0;
                }
                break;
        }
        return state;
    }

    private boolean isCentered(double tx, double ty) {
        return Math.abs(tx) < deadbandDeg && Math.abs(ty) < deadbandDeg;
    }

    /** @return true ровно на тике входа в LOCKED (разовый «выстрел»). */
    public boolean shouldFire() {
        return fireEvent;
    }

    public State getState()    { return state; }
    public int   getShotCount() { return shotCount; }
}
