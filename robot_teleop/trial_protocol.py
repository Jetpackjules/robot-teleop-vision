"""Durable, host-owned task settings, separate from robot calibration."""
import json
from pathlib import Path
from robot_teleop.recording import write_json

TASKS = {
    'single_block': {'instruction': 'Free exploration: practise the view and arm controls', 'success_metric': 'practice'},
    'count_blocks': {'instruction': 'Put as many blocks in the cup as possible', 'success_metric': 'block_count'},
    'custom': {'instruction': 'Balance test: move the objects into the cup', 'success_metric': 'pass_fail'},
    'tray_transfer_v1': {'instruction': 'Move as many objects as possible to the other tray. Fully release each one.',
                         'success_metric': 'block_count', 'time_limit_seconds': 60},
}
STUDY_TASKS = {
    'cup_stack_v1': {'instruction': 'Build the tallest stack of cups you can.', 'success_metric': 'block_count'},
    'cube_stack_v1': {'instruction': 'Build the tallest stack of cubes you can.', 'success_metric': 'block_count'},
    'stack_to_mug_v1': {'instruction': 'Move as many cubes as possible from the stack into the mug, starting at the top.', 'success_metric': 'block_count'},
}
STUDY_TASKS['cup_stack_v1']['time_limit_seconds'] = 60
LEGACY_TASKS = {'single_block', 'count_blocks', 'custom'}


def validate_protocol(value):
    if (not isinstance(value, dict) or type(value.get('version')) is not int
            or value['version'] not in (1, 2, 3, 4)
            or set(value) != ({'version', 'time_limit_seconds', 'tasks'} if value['version'] == 1 else
                              {'version', 'time_limit_seconds', 'practice_time_limit_seconds', 'tasks'})):
        raise ValueError('Invalid trial settings.')
    limit = value['time_limit_seconds']
    if type(limit) is not int or not 30 <= limit <= 7200:
        raise ValueError('Trial time must be a whole number between 30 and 7200 seconds.')
    practice_limit = value.get('practice_time_limit_seconds', 60)
    if type(practice_limit) is not int or not 30 <= practice_limit <= 7200:
        raise ValueError('Practice time must be a whole number between 30 and 7200 seconds.')
    expected_ids = {4: TASKS.keys() | STUDY_TASKS.keys(), 3: TASKS.keys()}.get(value['version'], LEGACY_TASKS)
    if not isinstance(value['tasks'], dict) or value['tasks'].keys() != expected_ids:
        raise ValueError('Configure every activity.')
    tasks = {}
    for key, task in value['tasks'].items():
        separate_limit = key in ('tray_transfer_v1', 'cup_stack_v1')
        expected = {'instruction', 'success_metric', 'time_limit_seconds'} if separate_limit else {'instruction', 'success_metric'}
        if not isinstance(task, dict) or set(task) != expected:
            raise ValueError('Invalid task settings.')
        instruction = task['instruction']
        if (not isinstance(instruction, str) or not instruction.strip() or len(instruction) > 500
                or any(ord(c) < 32 or ord(c) == 127 for c in instruction)):
            raise ValueError('Each task needs an instruction of up to 500 characters.')
        metrics = ('block_count',) if separate_limit or key in STUDY_TASKS else ('practice',) if key == 'single_block' and value['version'] >= 2 else ('pass_fail', 'block_count')
        if task['success_metric'] not in metrics:
            raise ValueError('Invalid scoring method for this task.')
        tasks[key] = {'instruction': instruction.strip(), 'success_metric': task['success_metric']}
        if separate_limit:
            seconds = task['time_limit_seconds']
            if type(seconds) is not int or not 30 <= seconds <= 7200:
                raise ValueError('Task 1 time must be a whole number between 30 and 7200 seconds.')
            tasks[key]['time_limit_seconds'] = seconds
    if value['version'] == 1:
        # Upgrade future-session configuration; existing episode metadata and
        # browser session plans keep their original task and scoring.
        tasks['single_block'] = dict(TASKS['single_block'])
    tasks.setdefault('tray_transfer_v1', dict(TASKS['tray_transfer_v1']))
    for key, default in STUDY_TASKS.items():
        tasks.setdefault(key, dict(default))
    if value['version'] < 4:
        tasks['cup_stack_v1']['time_limit_seconds'] = tasks['tray_transfer_v1']['time_limit_seconds']
    return {'version': 4, 'time_limit_seconds': limit, 'practice_time_limit_seconds': practice_limit, 'tasks': tasks}


def load_protocol(path: Path):
    value = json.loads(path.read_text()) if path.exists() else {
        'version': 4, 'time_limit_seconds': 75, 'practice_time_limit_seconds': 60, 'tasks': {**TASKS, **STUDY_TASKS}}
    return validate_protocol(value)


def save_protocol(path: Path, value):
    result = validate_protocol(value)
    if value['version'] < 4 and path.exists():
        previous = load_protocol(path)['tasks']
        for key in STUDY_TASKS:
            result['tasks'][key] = previous[key]
        if value['version'] < 3:
            result['tasks']['tray_transfer_v1'] = previous['tray_transfer_v1']
    path.parent.mkdir(parents=True, exist_ok=True)
    write_json(path, result)
    return result
