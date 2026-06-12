# Core Architecture Governance

This document defines the rules for the core-architecture refinement program.
Every implementation phase must preserve these rules or update this document
and receive explicit approval before continuing.

## Delivery Gate

Work proceeds one approved phase at a time.

1. Implement one phase only.
2. Run its focused tests and the full test suite.
3. Update behavior and architecture documentation in the same phase.
4. Create one focused commit and push it.
5. Stop and provide a verification report.
6. Do not begin the next phase until the user approves it.

## Module Boundaries

- A module has one primary responsibility.
- Calibration stages perform computation only. They do not capture cameras,
  write logs, render overlays, stream frames, publish MQTT, or drive hardware.
- Runtime entrypoints own lifecycle and transport integration. They do not
  implement calibration mathematics.
- Each configured stage processes a frame at most once.
- Stage order and dependencies are explicit.
- Stages exchange typed results rather than unrelated mutable global state.
- Capabilities such as positioning cannot silently affect steering. Calibration
  must explicitly declare them as required dependencies.

## Extensibility Rules

- Replaceable stages and capabilities use configured `module.path:ClassName`
  imports.
- Built-in implementations are the defaults when no plugin is configured.
- Plugin contracts are validated at startup.
- Any configured plugin or capability load/runtime failure is fatal. Existing
  shutdown handling must center steering before terminating the runtime.
- Independent capabilities consume the shared frame context and publish their
  own result without changing calibration inputs.

## Telemetry Rules

- Safety-critical outputs remain typed and separate from telemetry.
- Additional telemetry is a JSON-serializable dictionary.
- Producer-owned fields are namespaced, for example
  `calibration.geometry.vp_x` or `capabilities.positioning.x`.
- Duplicate producer IDs and non-JSON telemetry values are errors.
- Consumers must tolerate missing optional fields.
- Full telemetry is persisted as JSONL. CSV is a configured projection for
  compatibility and analysis.

## Current Refinement Boundary

The overlapping `LineDetector` implementation has been removed. Live and
offline entrypoints now share `UnifiedCalibrator.process_frame()` as the single
calibration computation path, and each frame is processed once by vision and
pair selection.

The current implementation still has mixed telemetry/runtime responsibilities,
broad dictionary contracts, longest-pair selection, and a PD-only controller.
Each limitation will change only in its own approved gate.
