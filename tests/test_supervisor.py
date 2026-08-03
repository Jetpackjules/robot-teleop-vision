from robot_teleop.supervisor import operator_ui_response_ready


def test_operator_health_accepts_controller_shell():
    assert operator_ui_response_ready(
        "https://127.0.0.1:8765/controller.html",
        200,
        b'<canvas id="hybrid-canvas"></canvas>',
    )


def test_operator_health_accepts_expected_password_gate():
    assert operator_ui_response_ready(
        "https://127.0.0.1:8765/login?next=%2Fcontroller.html",
        200,
        b'<form method="post" action="/login"><input name="password"></form>',
    )


def test_operator_health_rejects_unrelated_success_page():
    assert not operator_ui_response_ready(
        "https://127.0.0.1:8765/error",
        200,
        b"temporarily unavailable",
    )
