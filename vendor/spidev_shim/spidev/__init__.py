class SpiDev:
    def __init__(self) -> None:
        raise RuntimeError(
            "spidev is Linux/Raspberry Pi only (shim installed on macOS)")


def __getattr__(name: str):
    raise RuntimeError(
        "spidev is Linux/Raspberry Pi only (shim installed on macOS)")
