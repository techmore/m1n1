# Agent rules (techmore fork)

This fork's `elementaryos-mseries` branches are where the Mac mini M4 (J773g) Omarchy port is built and booted. The rules for working here are in one place: **[AGENT-RULES.md](https://github.com/techmore/ElementaryOS_MSeries-Kernel/blob/gravity-series-validation/gravity/AGENT-RULES.md)**. Read it before you push.

The short version:
- Work only in the `techmore` forks. Nothing goes upstream until it passes the release gate on the owner's M4 mini and the owner approves it. Check upstream's open PRs and branches first.
- CI runs on the M4 mini's self-hosted runners (repo variable routing, Ubuntu 26.04 only). Batch your pushes: a new kernel push cancels the running build.
- Cut `mini/<area>` topic branches from today's `elementaryos-mseries`, and keep each one to its own files.
- On macOS: never `git add -A` in a kernel checkout, always write `${var}` in zsh, and never commit keys.
