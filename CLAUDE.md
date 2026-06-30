# CLAUDE.md — AutoTurret

Guidance for Claude Code when working in this repo.

## Project Overview

**AutoTurret DSTS** — автономная турельная платформа наведения (UzDynamics). MVP-версия на **FTC SDK 11.0.0 + REV Control Hub + goBILDA Yellow Jacket моторы + Limelight 3A**.

Задача MVP: Limelight (color-pipeline) детектит шар → `tx/ty` → Control Hub наводит pan/tilt турель на цель, держит в центре, при стабильном захвате логирует «выстрел». Лазер всегда включён (отдельно).

Дизайн vision-части: см. Obsidian `07_Projects/autoturret/vision-design.md`.

## Build Commands

```bash
./gradlew assembleDebug                  # сборка APK
./gradlew :TeamCode:testDebugUnitTest    # юнит-тесты чистой логики
./gradlew installDebug                   # установка на Control Hub
```

## Architecture (3 слоя)

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── subsystems/   # Hardware abstraction
│   ├── AxisMotor.java            # PIDF-ось pan/tilt (TICKS_PER_DEGREE, kF, soft-лимиты)
│   └── LimelightColorTracker.java # color-pipeline tx/ty/ta/tv + кэш + disconnect
├── logic/        # ЧИСТАЯ логика (юнит-тестируется, без железа)
│   ├── TargetFilter.java         # persistence (3 кадра) + grace (300мс)
│   └── TurretFSM.java            # IDLE→SEARCH→TRACK→LOCKED→FIRE
└── opmodes/
    ├── AutoTurretOpMode.java     # интеграция Limelight→Filter→FSM→2×AxisMotor
    └── testers/
        ├── AxisMotorTester.java         # ручная проверка моторов + энкодеров
        └── LimelightColorTester.java    # телеметрия tx/ty/ta/tv
```

**Контракты модулей:** Limelight→FSM передаёт только `{visible, tx, ty}`. FSM→моторы — только целевые углы/мощность. Логику (filter+FSM) можно тестировать без Control Hub.

## Hardware Configuration Names

- `panMotor` — DcMotorEx + энкодер (Motor Port 0), горизонт 270°
- `tiltMotor` — DcMotorEx + энкодер (Motor Port 1), вертикаль −15°…+85°
- `limelight` — Limelight 3A (USB), pipeline 0 = Color

## Workflow / Conventions

- **TDD:** для чистой логики тесты пишутся первыми (`src/test/java`), показываются, реализация — после одобрения. Тесты не править ради прохождения — чинить код.
- Один файл за раз, объяснять план перед кодом.
- Naming: `camelCase` методы/поля, `PascalCase` классы, `SCREAMING_SNAKE_CASE` константы.
- Manual control через `targetAngle` + PID, не прямой `setPower()` (кроме калибровочных тестеров).
- Комментарии: русский для инлайн-пояснений, английский для публичного API.
- **Старт всегда из механического home** (центр) — энкодеры обнуляются там.

## Reused FTC patterns

Адаптировано из FTC-34241-Code (`.reference/`): `TurretMotor.java` → `AxisMotor`, persistence-фильтр из `Vision.java` → `TargetFilter`.
