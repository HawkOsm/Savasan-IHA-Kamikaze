# Savaşan İHA — Kamikaze (Strike) Mission — Team Anafarta

ArduPilot-based autonomous strike (kamikaze) mission stack for **Team Anafarta** (Yaşar University), built for Turkey's **Savaşan İHA** national UAV competition.

This repository documents the guidance, communication, mission, and vision systems developed and flight-tested (real UAV + Gazebo SITL) over the course of the project, along with the code behind them.

## Contents

- `docs/` — write-ups, diagrams, and curated test results
  - `algorithm/` — guidance, communication, and mission-logic design docs
  - `competition/` — Savaşan İHA context and mission requirements
  - `diagrams/` — system and flow diagrams
  - `gazebo/` — simulation setup notes
  - `jetson/` — onboard companion-computer pipeline docs
  - `logs/` — curated flight-test results (summaries/plots only — raw logs are not published, see below)
- `src/` — implementation
  - `scripts/ardupilot/` — ArduPilot-facing mission/attitude control code
  - `scripts/communication/` — TCP/ground-comm code
  - `scripts/gui/` — PyQt ground-station app (live map, HUD, MAVLink telemetry, Gazebo camera feed)
  - `scripts/jetson/` — onboard camera/mission code
  - `sim/` — Gazebo world, plane model, and parameter files used for SITL testing
- `assets/media/` — screenshots, photos, and videos from testing

## Status

Actively being documented and rewritten from ~8–10 months of prior development. Expect docs and code to fill in incrementally.

## Flight logs

Raw `.BIN`/`.param`/`.tlog` files are intentionally not published here. `docs/logs/` contains curated summaries (plots, tables, outcomes) only, to keep the repo useful without redistributing raw data.

## Credits

Third-party contributions this project builds on are listed in [CREDITS.md](CREDITS.md).

## License

MIT — see [LICENSE](LICENSE).
