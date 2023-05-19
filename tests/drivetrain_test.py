from drivetrain import Drivetrain
from unittest.mock import MagicMock
import pytest
from climbramp import ClimbRamp
from pytest import MonkeyPatch

@pytest.fixture
def mocktrain() -> Drivetrain:
    #start a real drivetrain objet
    mock_drive = Drivetrain()
    #mock the parts we want to test
    mock_drive.lEncoder = MagicMock()
    mock_drive.rEncoder = MagicMock()
    mock_drive.lMotor = MagicMock()
    mock_drive.rMotor = MagicMock()
    mock_drive.move = MagicMock()
    mock_drive.gyro = MagicMock()
    mock_drive.accelerometer = MagicMock()
    return mock_drive

#testting drive_straight
@pytest.mark.parametrize(('lDist', 'rDist', 'atSetpoint'), (
        (2, 3, False),
        (5, 8, False),
        (2, 2, True),
        (3.1, 3.0, True),
        (-2, 1, False)
))
def test_drive_straight(mocktrain: Drivetrain, monkeypatch: MonkeyPatch, lDist, rDist, atSetpoint):
    # setup
    auto = ClimbRamp(mocktrain)
    def mock_getLEncoderDistance(self):
        return lDist

    def mock_getREncoderDistance(self):
        return rDist
    monkeypatch.setattr(Drivetrain, "getLEncoderDistance", mock_getLEncoderDistance)
    monkeypatch.setattr(Drivetrain, "getREncoderDistance", mock_getREncoderDistance)

    # action
    auto.drive_straight()

    # assert
    if atSetpoint:
        auto.drivetrain.move.assert_called_once_with(0, auto.forward_rate)
    else:
        auto.drivetrain.move.assert_called_once()

@pytest.mark.parametrize(('gyroy', 'output'),(
        (7, False),
        (8, True),
        (3, False),
        (1, False),
        (0, False),
        (10, True)
))
def test_tip_up(mocktrain: Drivetrain, monkeypatch: MonkeyPatch, gyroy, output):
    # setup
    auto = ClimbRamp(mocktrain)
    def mock_getGyroAngleY(self):
        return gyroy

    monkeypatch.setattr(Drivetrain, "getGyroAngleY", mock_getGyroAngleY)
    # action
    result = auto.did_tip_up()
    # assert
    assert result == output

def test_reset(mocktrain: Drivetrain):
    # setup, extra step to "un-reset the auto before testing"
    auto = ClimbRamp(mocktrain)
    auto.ended_ramp = True
    auto.started_ramp = True
    auto.forward_rate = 100
    # action
    auto.reset()
    # assert
    assert auto.ended_ramp == False and auto.started_ramp == False and auto.forward_rate == .8