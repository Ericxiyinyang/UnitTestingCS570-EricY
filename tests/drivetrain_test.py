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
