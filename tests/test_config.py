from dexim.core import config
from dexim.core.config import ModbusRTUProtocolConfig, ModbusTCPProtocolConfig


def test_modbus_protocol_defaults_are_generic() -> None:
    assert ModbusTCPProtocolConfig() == ModbusTCPProtocolConfig(
        ip="192.168.1.100",
        port=502,
    )
    assert ModbusRTUProtocolConfig() == ModbusRTUProtocolConfig(
        port="/dev/ttyUSB0",
        baud=115200,
    )


def test_modbus_protocol_values_can_be_overridden() -> None:
    assert ModbusTCPProtocolConfig(ip="10.0.0.2", port=1502).port == 1502
    assert ModbusRTUProtocolConfig(port="COM4", baud=57600).baud == 57600


def test_vendor_specific_tesollo_configs_are_not_exported() -> None:
    assert not hasattr(config, "TesolloConfig")
    assert not hasattr(config, "TesolloRealConfig")
