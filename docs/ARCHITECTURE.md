# Architecture

The stack has four local trust boundaries:

1. **Camera adapters and native capture** discover hardware, own the RealSense pipelines, and publish only the newest RGB-D packet.
2. **Godot runtime** reconstructs camera surfaces, maintains serial-keyed site alignment, renders the selected module's optional geometry, and relays module settings.
3. **Operator server** authenticates browsers, compresses RGB-D, serves the selected module, relays semantic commands/views, and grants ownership to only the newest page.
4. **Robot module process** translates negotiated semantics into hardware writes and independently applies watchdogs, limits, feedback checks, and Hold.

The browser is the only operational UI. The Godot editor plugin calls the same repository CLI for setup, diagnostics, launch, and safe shutdown.

## Data flow

```text
camera(s) -> native Godot capture -> latest-only RGB-D source
                                  -> temporal RGB/depth encoder
                                  -> authenticated newest browser

newest browser -> robot-teleop/v1 command -> selected module operator -> hardware process
newest browser -> shared view/tracking settings -> Godot runtime + selected Godot module
hardware status -> selected module operator -> browser health + Godot overlay
auxiliary camera -> selected module view route -> browser module
```

Capture, encoding, requests, and client rendering are decoupled. A slow encoder or superseded HTTP request cannot block the camera drain or build a stale FIFO. Temporal RGB and depth encode in parallel; absolute ordered tiles update a persistent reference, while full keyframes make the stream recoverable.

## Module boundary

Core knows only the manifest, `RobotAdapter`, `GenericRobotOperator`, `robot-teleop/v1`, browser hooks, a renderer descriptor, and the Godot `robot_module` group. It does not know motor count, link names, hardware packets, calibration stages, or UI labels.

The manifest is both negotiation and composition: it names supported command spaces, inputs, safety actions, views, and features, then points each runtime at module-owned code. Private Python entrypoints and state filenames are never exposed to the browser.

The SO-101 folder exercises the complete boundary. Removing it leaves a valid vision-only application.

## Local-only state

Camera serials are discovery keys, not source constants. Shared camera transforms and world level live in Godot `user://`. A module declares only its durable state filenames; physical identities, motor ports, profiles, and calibration tables remain in ignored local configuration. A clean clone therefore cannot move hardware.

## Safety ownership

The server validates protocol shape, negotiated capability, page ownership, and transport freshness. The robot process remains the final authority: browser authorization is not permission to bypass a module's deadman, watchdog, limits, feedback checks, or physical emergency stop.
