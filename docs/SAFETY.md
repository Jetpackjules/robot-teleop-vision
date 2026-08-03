# Safety

This is research teleoperation software, not a safety-rated controller.

Before any motion:

- Clear the complete swept volume, including cables, tools, and camera brackets.
- Keep a physical power disconnect within immediate reach.
- Support newly assembled or repaired hardware during its first enable test.
- Confirm the correct voltage/current source for the selected robot.
- Run `robot-teleop doctor` and begin with motion disabled or the module's dry-run mode.

The shared stack provides newest-browser ownership, authenticated control, command freshness, explicit enable/Hold, and safe shutdown. A robot module is responsible for deadman behavior, watchdogs, joint/workspace limits, following-error/contact handling, stale feedback, modeled collision checks, and calibration rollback. Software checks cannot see every person, loose object, attachment, structural crack, or collision.

Automatic calibration must refuse motion without the evidence its module requires. Never weaken an evidence or geometry gate simply to make a difficult scene pass. Automatic return-to-rest should be opt-in because a modeled path cannot detect every obstacle.

## Emergency response

Remove physical actuator power first. Then use the selected module's **Hold** control or:

```bash
robot-teleop stop
```

Do not rely on closing a browser as an emergency stop.
