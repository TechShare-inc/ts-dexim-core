from dexim.core.messages import ErgonomicsState, StatusInfo, TopicBuilder


def test_ergonomics_state_round_trips_and_normalizes_numeric_values() -> None:
    state = ErgonomicsState.from_dict(
        {
            "glove_id": "101",
            "side": "left",
            "timestamp": "12.5",
            "values": ["1.0", 2],
        }
    )

    assert state == ErgonomicsState(101, "left", 12.5, [1.0, 2.0])
    assert state.to_dict() == {
        "glove_id": 101,
        "side": "left",
        "timestamp": 12.5,
        "values": [1.0, 2.0],
    }


def test_ergonomics_state_serialization_copies_values() -> None:
    state = ErgonomicsState(101, "right", 12.5, [1.0])

    encoded = state.to_dict()
    encoded["values"].append(2.0)

    assert state.values == [1.0]


def test_ergonomics_topic_uses_shared_vocabulary() -> None:
    assert (
        TopicBuilder().observation.ergonomics_state("manus")
        == b"observation/manus/ergonomics_state"
    )


def test_control_epoch_status_fields_round_trip() -> None:
    info = StatusInfo.from_flat_dict(
        {
            "control_epoch_id": "epoch-1",
            "activation_time": 103.0,
            "countdown_duration": 3.0,
            "preparation_ready": True,
            "start_rejected": False,
            "activation_rejected": True,
        }
    )

    assert info.control_epoch_id == "epoch-1"
    assert info.activation_time == 103.0
    assert info.countdown_duration == 3.0
    assert info.preparation_ready
    assert not info.start_rejected
    assert info.activation_rejected
    assert info.to_flat_dict()["control_epoch_id"] == "epoch-1"
