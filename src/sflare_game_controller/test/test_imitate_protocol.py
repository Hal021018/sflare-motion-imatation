import json
import pytest
from sflare_game_controller.imitate_protocol import decode_request, score_payload


def test_actual_action_request():
    assert decode_request('{"cmd":"arm_demo","args":{"act_name":"点赞"}}') == (
        'arm_demo', {'act_name': '点赞'})


@pytest.mark.parametrize('raw', ['[]', '{}', '{"cmd":"right_init"}',
                                '{"cmd":"arm_demo","args":{}}',
                                '{"cmd":"acts","args":[]}'])
def test_bad_request(raw):
    with pytest.raises(ValueError):
        decode_request(raw)


def test_score_angles_not_ratios():
    result = score_payload(dict(score=95, reference_positions=[103., 0, 0, 0, 0, 0, 0],
                                current_positions=[101., 1, 2, 3, 4, 5, 6]))
    assert set(result) == {'score', 'j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'j7'}
    assert result['j1'] == {'target': 103., 'actual': 101.}
    assert result['j7'] == {'target': 0., 'actual': 6.}
    json.dumps(result, allow_nan=False)
