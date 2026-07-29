# Runtime logs / 运行日志

This directory is the default root for direct CLI logs. Each run receives an
isolated directory containing the rotating `hgasp.log`, rotating
`events.jsonl`, atomically replaced `algorithm.status.json`, and, when SP is
invoked, `gurobi.log`. The two rotating logs default to one current 8 MiB
segment plus two backups. Set `HGASP_LOG_DIR` to use an explicit run directory
and `HGASP_LOG_CONSOLE=0` to disable console mirroring. Rotation is configured
with `HGASP_LOG_SEGMENT_BYTES` and `HGASP_LOG_BACKUPS`.

本目录是直接运行 CLI 时的默认日志根目录。每次运行使用独立子目录，包含轮转的
`hgasp.log`、轮转的 `events.jsonl`、原子覆盖的
`algorithm.status.json`，实际调用 SP 时还包含 `gurobi.log`。两个轮转日志默认
最多保存一个当前 8 MiB 段和两个备份。可通过 `HGASP_LOG_DIR` 指定本次运行目录，
通过 `HGASP_LOG_CONSOLE=0` 关闭终端镜像；轮转大小和备份数分别由
`HGASP_LOG_SEGMENT_BYTES`、`HGASP_LOG_BACKUPS` 设置。

Generated log files are ignored by Git. Experiment automation stores
attempt-specific logs under each run's `logs/attempt_NNN/` directory and adds
the supervisor-owned `process.status.json` heartbeat there.
