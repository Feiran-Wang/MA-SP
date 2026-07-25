# Runtime logs / 运行日志

This directory is the default root for direct CLI logs. Each run receives an
isolated directory containing `hgasp.log` and, when SP is enabled,
`gurobi.log`. Set `HGASP_LOG_DIR` to use an explicit run directory and
`HGASP_LOG_CONSOLE=0` to disable console mirroring.

本目录是直接运行 CLI 时的默认日志根目录。每次运行使用独立子目录，包含
`hgasp.log`，启用 SP 时还包含 `gurobi.log`。可通过 `HGASP_LOG_DIR` 指定本次
运行的目录，通过 `HGASP_LOG_CONSOLE=0` 关闭终端镜像。

Generated log files are ignored by Git. Experiment automation stores
attempt-specific logs under each run's `logs/attempt_NNN/` directory.
