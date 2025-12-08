from .library_loader import *
from .InuStreamsPyth import EStreamState
from enum import IntEnum


class StreamState(IntEnum):
    """!
        Stream's state is always one of EStreamState
    """
    UNINITIALIZED = EStreamState.EStreamState.Uninitialized
    INITIALIZED = EStreamState.EStreamState.Initialized
    STARTED = EStreamState.EStreamState.Started
    SENSOR_DISCONNECTED = EStreamState.EStreamState.SensorDisconnected


class BaseStream:
    """! Common interface for all InuDev streams.
        Role: Base class for all streams classes. It defines an interface that should be implemented by all derived classes.

        Responsibilities:
              1. Knows how to control the service (Init, Terminate, Start and Stop)
              2. Implements a state machine (more details in Reference Manual)
    """

    DEFAULT_CHANNEL_ID = 4294967295

    _base_stream = None
    """! The one of InuStreamsPyth streams.
    """

    callback = None
    """! The Callback from one of InuStreamsPyth streams.
    """

    def __init__(self, stream):
        self._base_stream = stream
        """! The stream class initializer.
            @param stream  The one of InuStreamsPyth streams..
            @return  An instance of the stream initialized with the specified InuStreamsPyth stream object.
        """

    def start(self) -> None:
        """!
            Start acquisition of frames.
            Shall be invoked only after the service is successfully initialized and before any request
            for new frame (push or pull).
        """
        self._base_stream.Start()

    def stop(self) -> None:
        """! Stop acquisition of frames.
        """
        self._base_stream.Stop()

    def terminate(self) -> None:
        """!
            Service termination.
            Shall be invoked when the service is no longer in use and after frames acquisition has stopped.
        """
        self._base_stream.Register(None)
        self._base_stream.Stop()
        self._base_stream.Terminate()

    @property
    def channel_id(self) -> str:
        """!
            Final stream channel ID as received from InuService.
            Return ChannelId as string.
        """
        return self._base_stream.ChannelID

    def state(self) -> StreamState:
        """!
            Shall be invoked after requests for frames are no longer sent and before service termination
            (only if Start() was invoked).
            brief    Stream state information.
            return EStreamState    Stream's current state.
        """
        return StreamState(self._base_stream.State())

    def save(self, save_path: str) -> None:
        """!
            Save the frame.
            @param save_path  Save file path.
        """
        self._base_stream.Save(save_path)
