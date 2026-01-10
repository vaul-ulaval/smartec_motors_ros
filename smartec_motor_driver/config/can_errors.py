"""
Fault codes definition file.

Each fault code is mapped to:
- name: short symbolic identifier
- description: human-readable explanation
- action: resulting system behavior
"""

FAULT_CODES = {
    0: {
        "name": "NormalOperation",
        "description": "Normal operation",
        "action": "Normal Operation",
    },
    1: {
        "name": "SensorOutOfRange",
        "description": "Primary sensor out of range",
        "action": "EKill",
    },
    2: {
        "name": "TooHighCurr",
        "description": "Current draw is too high",
        "action": "EKill",
    },
    4: {
        "name": "VbatTooLow",
        "description": "Battery voltage is too low for too long",
        "action": "EKill",
    },
    5: {
        "name": "VbatTooHigh",
        "description": "Battery voltage is too high for too long",
        "action": "EKill",
    },
    6: {
        "name": "TempHigh",
        "description": "Temperature is too high for too long",
        "action": "EKill",
    },
    7: {
        "name": "LostControlComm",
        "description": "Control communication was lost",
        "action": "EKill",
    },
    8: {
        "name": "LostSecurityComm",
        "description": "Security communication was lost",
        "action": "EKill",
    },
    9: {
        "name": "LapbarNonZeroWhenNeutral",
        "description": ("Lapbar neutral limit switch indicates neutral, " "but sensor is too far from neutral"),
        "action": "EKill",
    },
    10: {
        "name": "ModuleNeedsSystemCal",
        "description": "Device has not been system calibrated",
        "action": "No drive until resolved",
    },
    11: {
        "name": "HardwareSelfTestFailed",
        "description": "Hardware self-test indicates hardware damage",
        "action": "EKill",
    },
    15: {
        "name": "SensorNotReliable",
        "description": "Primary sensor is not reliable",
        "action": "EKill",
    },
    16: {
        "name": "UnsupportedConfig",
        "description": "The setting is not valid or supported",
        "action": "No drive until resolved",
    },
    17: {
        "name": "UnsupportedControlMode",
        "description": "Invalid or unsupported control mode received",
        "action": "No drive until resolved",
    },
    19: {
        "name": "ControlModeIsDisabledNeutral",
        "description": "EDM entered disabled-neutral control mode",
        "action": "EKill",
    },
    23: {
        "name": "CANErrorCountTooHigh",
        "description": "CAN error counts exceeded configured thresholds",
        "action": "Normal Operation",
    },
    24: {
        "name": "WatchdogReset",
        "description": "Device reset due to hardware watchdog",
        "action": "EKill",
    },
    25: {
        "name": "SystemCalAborted",
        "description": "System calibration aborted due to system change",
        "action": "EKill",
    },
    26: {
        "name": "InvalidState",
        "description": "State is invalid",
        "action": "EKill",
    },
    29: {
        "name": "MotorStall",
        "description": "Motor stall detected",
        "action": "EKill",
    },
    30: {
        "name": "MotorFailure",
        "description": "Motor failure detected",
        "action": "EKill",
    },
    31: {
        "name": "SwitchingHardware",
        "description": "Switching hardware failure detected",
        "action": "EKill",
    },
    32: {
        "name": "RPMBoundsViolation",
        "description": "RPM out of bounds for too long",
        "action": "EKill",
    },
    33: {
        "name": "SevereFailureCoast",
        "description": "Generic severe failure causing system to coast",
        "action": "EKill",
    },
    34: {
        "name": "BrakeCircuit",
        "description": "Brake circuit malfunction",
        "action": "EKill",
    },
    35: {
        "name": "WaterDetect",
        "description": "Moisture detected inside controller housing",
        "action": "EKill",
    },
    41: {
        "name": "LapBarNotCentered",
        "description": "Lapbar position not centered on at least one device",
        "action": "EKill",
    },
    42: {
        "name": "BrakeRequired",
        "description": "Brake is not asserted",
        "action": "No drive until resolved",
    },
    43: {
        "name": "PtoSelIsOn",
        "description": "PTO is asserted",
        "action": "No drive until resolved",
    },
    44: {
        "name": "NoOpPresDuringPto",
        "description": "Operator not present while PTO enabled",
        "action": "EKill",
    },
    45: {
        "name": "OperatorNotPresent",
        "description": "Operator is not present",
        "action": "No drive until resolved",
    },
    48: {
        "name": "TooManyFaultCodes",
        "description": "Fault buffer has reached capacity",
        "action": "EKill",
    },
    49: {
        "name": "BrakeOffAndOperNotPres",
        "description": "Brake off and operator not present",
        "action": "EKill",
    },
    50: {
        "name": "LapbarNotCenteredAndOperNotPres",
        "description": "Operator not present and lapbar not neutral",
        "action": "EKill",
    },
    51: {
        "name": "BrakeOnAndLapbarNotCentered",
        "description": "Brake on and lapbar not neutral",
        "action": "EKill",
    },
    57: {
        "name": "SerialNumberMismatch",
        "description": "ECU serial number mismatch",
        "action": "No drive until resolved",
    },
    60: {
        "name": "LostComm",
        "description": "Lost communication with another ECU",
        "action": "EKill",
    },
    61: {
        "name": "EcuHasReset",
        "description": "At least one ECU has reset",
        "action": "EKill",
    },
    62: {
        "name": "AnotherDeviceInInit",
        "description": "At least one ECU is still in reset state",
        "action": "No drive until resolved",
    },
    70: {
        "name": "DeckLostComm",
        "description": "Lost communication with a DCM",
        "action": "Enter PowerSafe™ Mode",
    },
    71: {
        "name": "DeckFaultLatched",
        "description": "DCM fault detected and all DCMs disabled",
        "action": "Enter PowerSafe™ Mode",
    },
    72: {
        "name": "WaitForPostComplete",
        "description": "POST complete message not received from GDM",
        "action": "No drive until resolved",
    },
    95: {
        "name": "CalibrationMissingSerialNumbers",
        "description": "Required expected serial numbers not available",
        "action": "EKill",
    },
    100: {
        "name": "CalibrationPhaseHallPolarityDisagreement",
        "description": "Hall and phase wiring polarity mismatch",
        "action": "EKill",
    },
    101: {
        "name": "CalibrationInsufficientIndexDifference",
        "description": "Insufficient electrical index difference detected",
        "action": "EKill",
    },
    102: {
        "name": "CalibrationNotAtRest",
        "description": "Calibration attempted while motor not at rest",
        "action": "EKill",
    },
    103: {
        "name": "CalibrationMaxRPMCouldNotBeMet",
        "description": "Maximum RPM could not be achieved",
        "action": "EKill",
    },
    104: {
        "name": "CalibrationCoastTimeOut",
        "description": "Drive took too long to coast to stop",
        "action": "EKill",
    },
    105: {
        "name": "CalibrationCouldNotSyncToBEMF",
        "description": "Failed to synchronize to BEMF",
        "action": "EKill",
    },
    106: {
        "name": "CalibrationInsufficientCurrent",
        "description": "Indexing current goal not met",
        "action": "EKill",
    },
    107: {
        "name": "CalibrationDependencyNotMet",
        "description": "Calibration dependency not met",
        "action": "EKill",
    },
    108: {
        "name": "CalibrationRotorNotLocked",
        "description": "Rotor expected locked but was not",
        "action": "EKill",
    },
    115: {
        "name": "BMSGeneralBatteryError",
        "description": "BMS reported an error",
        "action": "Enter PowerSafe™ Mode",
    },
    116: {
        "name": "BMSSOCTooLow",
        "description": "BMS reported SOC below cutoff",
        "action": "Enter PowerSafe™ Mode",
    },
    117: {
        "name": "BMSChargerConnected",
        "description": "Charger connected",
        "action": "EKill",
    },
    118: {
        "name": "BMSLostComm",
        "description": "Lost communication with BMS",
        "action": "Enter PowerSafe™ Mode",
    },
}
