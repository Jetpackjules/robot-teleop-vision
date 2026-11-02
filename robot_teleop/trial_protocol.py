"""Durable, host-owned task settings, separate from robot calibration."""
import json
from pathlib import Path
from robot_teleop.recording import write_json

TASKS = {
    'single_block': {'instruction': 'Move the object from one plate to the other', 'success_metric': 'pass_fail'},
    'count_blocks': {'instruction': 'Put as many blocks in the cup as possible', 'success_metric': 'block_count'},
    'custom': {'instruction': 'Balance test: move the objects into the cup', 'success_metric': 'pass_fail'},
}


def validate_protocol(value):
    if (not isinstance(value, dict) or set(value) != {'version', 'time_limit_seconds', 'tasks'}
            or type(value['version']) is not int or value['version'] != 1):
        raise ValueError('Invalid trial settings.')
    limit = value['time_limit_seconds']
    if type(limit) is not int or not 30 <= limit <= 7200:
        raise ValueError('Trial time must be a whole number between 30 and 7200 seconds.')
    if not isinstance(value['tasks'], dict) or set(value['tasks']) != set(TASKS):
        raise ValueError('Configure all three activities.')
    tasks = {}
    for key, task in value['tasks'].items():
        if not isinstance(task, dict) or set(task) != {'instruction', 'success_metric'}:
            raise ValueError('Invalid task settings.')
        instruction = task['instruction']
        if (not isinstance(instruction, str) or not instruction.strip() or len(instruction) > 500
                or any(ord(c) < 32 or ord(c) == 127 for c in instruction)):
            raise ValueError('Each task needs an instruction of up to 500 characters.')
        if task['success_metric'] not in ('pass_fail', 'block_count'):
            raise ValueError('Choose pass/fail or a count in the cup.')
        tasks[key] = {'instruction': instruction.strip(), 'success_metric': task['success_metric']}
    return {'version': 1, 'time_limit_seconds': limit, 'tasks': tasks}


def load_protocol(path: Path):
    value = json.loads(path.read_text()) if path.exists() else {
        'version': 1, 'time_limit_seconds': 75, 'tasks': TASKS}
    return validate_protocol(value)


def save_protocol(path: Path, value):
    result = validate_protocol(value)
    path.parent.mkdir(parents=True, exist_ok=True)
    write_json(path, result)
    return result
