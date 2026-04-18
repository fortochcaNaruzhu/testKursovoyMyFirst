# Тесты и диагностика визуализации / реплея

Скрипты и заметки, чтобы не смешивать их с основным кодом симулятора.

| Файл | Назначение |
|------|------------|
| `LOGGING_REFERENCE.md` | Откуда берутся логи и с какой номинальной частотой |
| `analyze_log_spacing.py` | Статистика Δt между строками и шаг координат в мм |

Запуск из корня репозитория `Drone_Swarm_Simulator_v2`:

```bash
python3 tests/visualization/analyze_log_spacing.py pair logs/two_drones_log_run_exchange_sync.csv
python3 tests/visualization/analyze_log_spacing.py experiment logs/replay_from_run
```

Опционально сохранить отчёт:

```bash
python3 tests/visualization/analyze_log_spacing.py pair logs/...csv -o tests/visualization/out/spacing_report.txt
```

Папку `out/` можно создавать вручную; в `.gitignore` добавлена, чтобы не коммитить отчёты.
