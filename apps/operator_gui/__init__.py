"""Desktop operator GUI."""

__all__ = ["OperatorWindow"]


def __getattr__(name: str):
    # The camera bridge runs in the lean ROS image, which intentionally does
    # not install Qt. Keep importing this package free of desktop dependencies.
    if name == "OperatorWindow":
        from .operator_gui import OperatorWindow

        return OperatorWindow
    raise AttributeError(name)
