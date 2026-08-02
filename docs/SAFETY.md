# Safety

This is research teleoperation software, not a safety-rated controller.

Before any motion:

- Clear the complete swept volume, including cables and camera brackets.
- Keep a physical power disconnect within immediate reach.
- Support a newly assembled or repaired arm for its first enable test.
- Confirm the correct voltage/current source for the arm hardware.
- Run `robot-teleop doctor` and start in hardware-disabled or follower dry-run mode.

The software provides defense in depth: newest-browser ownership, explicit Enable/Hold, command-age watchdogs, joint/workspace limits, measured following-error stopping, stale telemetry handling, modeled base-plane checks, and rollback of incomplete calibration. Those checks cannot see every person, loose object, attachment, structural crack, or collision.

Automatic calibration deliberately refuses to move without fresh reference-camera depth and follower telemetry. A rejected stage Holds and keeps the previous complete registration. Never loosen those evidence gates merely to make a difficult scene pass.

Automatic return-to-rest is opt-in because modeled clearance cannot detect scene obstacles. Hold interrupts it. A saved rest pose must match the same follower serial and pass the modeled base-plane path preflight.

## Emergency response

Remove physical motor power first. Then use **Hold Arm** or:

```bash
robot-teleop stop
```

Do not rely on closing a browser as an emergency stop.
