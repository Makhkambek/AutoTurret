package org.firstinspires.ftc.teamcode.logic;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Before;
import org.junit.Test;

import org.firstinspires.ftc.teamcode.logic.TurretFSM.State;

/**
 * TurretFSM: IDLE → SEARCH → TRACK → LOCKED, с FIRE разово на захват.
 *
 * Контракт:
 *  - update(visible, tx, ty) вызывается раз в loop, возвращает текущее состояние.
 *  - IDLE до start(); start() → SEARCH; stop() → IDLE из любого состояния.
 *  - SEARCH + visible → TRACK.
 *  - TRACK + !visible → SEARCH.
 *  - TRACK: |tx|<deadband && |ty|<deadband framesToLock кадров подряд → LOCKED.
 *  - При входе в LOCKED: shouldFire()==true ОДИН раз, shotCount++.
 *  - LOCKED держится пока в центре; цель ушла из центра → TRACK (перевзвод выстрела).
 *  - LOCKED + !visible → SEARCH (перевзвод).
 *
 * deadband = 1.0°, framesToLock = 3.
 */
public class TurretFSMTest {

    private TurretFSM fsm;

    @Before
    public void setUp() {
        fsm = new TurretFSM(1.0, 3);
    }

    @Test
    public void startsIdle() {
        assertEquals(State.IDLE, fsm.getState());
    }

    @Test
    public void staysIdleUntilStart() {
        fsm.update(true, 0, 0);             // даже с целью без start() — IDLE
        assertEquals(State.IDLE, fsm.getState());
    }

    @Test
    public void startGoesToSearch() {
        fsm.start();
        assertEquals(State.SEARCH, fsm.getState());
    }

    @Test
    public void searchToTrackWhenVisible() {
        fsm.start();
        assertEquals(State.TRACK, fsm.update(true, 20, 5));
    }

    @Test
    public void trackToSearchWhenLost() {
        fsm.start();
        fsm.update(true, 20, 5);            // TRACK
        assertEquals(State.SEARCH, fsm.update(false, 0, 0));
    }

    @Test
    public void centeredBelowFramesToLockStaysTrack() {
        fsm.start();
        fsm.update(true, 20, 5);            // TRACK (off-center)
        fsm.update(true, 0.2, 0.1);         // centered кадр 1
        fsm.update(true, 0.2, 0.1);         // centered кадр 2
        assertEquals(State.TRACK, fsm.getState());  // ещё не 3 → не LOCKED
    }

    @Test
    public void locksAfterFramesToLockCentered() {
        fsm.start();
        fsm.update(true, 20, 5);            // TRACK
        fsm.update(true, 0.2, 0.1);
        fsm.update(true, 0.2, 0.1);
        State s = fsm.update(true, 0.2, 0.1); // 3-й centered → LOCKED
        assertEquals(State.LOCKED, s);
    }

    @Test
    public void firesOnceOnLock() {
        driveToLock();
        assertTrue(fsm.shouldFire());
        assertEquals(1, fsm.getShotCount());
    }

    @Test
    public void doesNotFireAgainWhileHeldLocked() {
        driveToLock();
        assertTrue(fsm.shouldFire());
        fsm.update(true, 0.2, 0.1);         // остаёмся в центре
        assertFalse(fsm.shouldFire());
        fsm.update(true, 0.1, 0.0);
        assertFalse(fsm.shouldFire());
        assertEquals(1, fsm.getShotCount()); // всё ещё один выстрел
    }

    @Test
    public void reArmsAfterLeavingCenter() {
        driveToLock();                       // shot 1
        assertEquals(1, fsm.getShotCount());
        fsm.update(true, 15, 3);             // цель ушла из центра → TRACK
        assertEquals(State.TRACK, fsm.getState());
        // снова навелись
        fsm.update(true, 0.2, 0.1);
        fsm.update(true, 0.2, 0.1);
        fsm.update(true, 0.2, 0.1);          // LOCKED снова
        assertTrue(fsm.shouldFire());
        assertEquals(2, fsm.getShotCount()); // второй выстрел
    }

    @Test
    public void lockedToSearchWhenLost() {
        driveToLock();
        assertEquals(State.LOCKED, fsm.getState());
        assertEquals(State.SEARCH, fsm.update(false, 0, 0));
    }

    @Test
    public void stopGoesToIdleFromAnyState() {
        driveToLock();
        fsm.stop();
        assertEquals(State.IDLE, fsm.getState());
    }

    @Test
    public void deadbandUsesBothAxes() {
        fsm.start();
        fsm.update(true, 20, 5);            // TRACK
        // tx в зоне, ty вне зоны → НЕ должно лочиться
        fsm.update(true, 0.2, 5.0);
        fsm.update(true, 0.2, 5.0);
        fsm.update(true, 0.2, 5.0);
        assertEquals(State.TRACK, fsm.getState());
    }

    /** Helper: start → TRACK → 3 centered кадра → LOCKED (shot fired). */
    private void driveToLock() {
        fsm.start();
        fsm.update(true, 20, 5);
        fsm.update(true, 0.2, 0.1);
        fsm.update(true, 0.2, 0.1);
        fsm.update(true, 0.2, 0.1);
    }
}
