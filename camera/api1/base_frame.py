from .shared import ProjectorType, ProjectorLevel


class BaseFrame:
    """!     Common behavior of all InuDev NUI frames.
        Role: Base class of all InuStreams' frames.

        Responsibilities:
            1. Common members.
            2. Construction is enabled only from derived classes.
    """

    def __init__(self, frame):
        """! The Base Frame class initializer.
            @param frame  The one of frames from InuStreamsPyth.
            @return  An instance of the BaseFrame initialized with the specified InuStreamsPyth.frame  object.
        """
        self._frame = frame

    _frame = None
    """! InuStreamsPyth.BaseF.
    """

    SCORE_MIN = 0
    SCORE_MAX = 100

    @property
    def time_stamp(self) -> int:
        """! The frame acquisition time in nanoseconds relates to host's system clock.
            It should be used for frames synchronization.
        """
        return self._frame.Timestamp

    @property
    def frame_index(self) -> int:
        """! The frame acquisition unique index, should be used for synchronization.
        """
        return self._frame.FrameIndex

    @property
    def chunk_index(self) -> int:
        """! The index of a chunk inside a frame, should be used for synchronization.
        """
        return self._frame.ChunkIndex

    @property
    def valid(self) -> bool:
        """! Indicates if this frame is valid data or not.
        """
        return self._frame.Valid

    @property
    def score(self) -> int:
        """! The confidence of current frame. Range is from SCORE_MIN up to SCORE_MAX.
        """
        return self._frame.Score

    @property
    def service_time_stamp(self) -> int:
        """! The  time stamp which is set when frame is received from USB (used for statistics and analysis).
        """
        return self._frame.ServiceTimestamp

    @property
    def stream_time_stamp(self) -> int:
        """! The time stamp which is set when frame is received from InuService (used for statistics and analysis.
        """
        return self._frame.StreamTimestamp

    @property
    def calibration_temperature(self) -> int:
        """!  Calibration temperature which was applied while this frame was acquired.
            std::numeric_limits<int32_t>::max() if it is not valid.
        """
        return self._frame.CalibrationTemperature

    @property
    def was_recorded(self) -> bool:
        """!  Indicated if the trame was recorded by InuService or doesn't.
        """
        return self._frame.WasRecorded

    # @property
    # def active_projector(self) -> EProjectorType:
    #     """
    #        The active projector received by FW for this frame: default(Pattern)
    #     """
    #     return EProjectorType(self._frame.ActiveProjector)

    @property
    def projector_level(self) -> ProjectorLevel:
        """! The projector level received by FW for this frame.
        """
        return ProjectorLevel(self._frame.mProjectorLevel)

    @property
    def projector_type(self) -> ProjectorType:
        """!   The projector type by FW for this frame.
        """
        return ProjectorType(self._frame.mProjectorType)
