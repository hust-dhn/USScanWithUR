from .base_stream import BaseStream
from .base_frame import BaseFrame
from .shared import ROIParameters
from .errors import Error
from .InuStreamsPyth import HistogramF, HistogramS, InuError, Point2D, VectorHistogramROI, HistogramR, VectorUint, \
    InuError


class HistogramROI(ROIParameters):
    """!  Histogram ROI.

    Role: Represents Histogram data and ROI.

    """

    # @brief    InuStreamsPyth.HistogramR.
    #
    data: HistogramR = None

    def __init__(self, data: HistogramR):
        self.data = data
        ROIParameters.__init__(data.ROITopLeft.X, data.ROITopLeft.Y, data.ROIBottomRight.X,
                               data.ROIBottomRight.Y)
        """! The Histogram ROI class initializer.
            @param data  The HistogramR  from InuStreamsPyth.
            @return  An instance of the HistogramROI initialized with the specified InuStreamsPyth.HistogramR  object.
        """

    @property
    def histogram(self) -> VectorUint:
        # @brief
        # #
        # @return histogram roi as VectorUint
        return self.data.Histogram

    @property
    def accumulator(self) -> int:
        # @brief    All histogram roi
        #
        # @return all histogram Accumulator
        return self.data.Accumulator


class HistogramFrame(BaseFrame):
    """!  Histogram frame.

    Role: Represents collection of Histogram data that is provided by Histogram.

    """

    def __init__(self, frame: HistogramF):
        self.histogram_frame = frame
        BaseFrame.__init__(self, frame)
        """! The Imu Frame class initializer.
            @param frame  The HistogramFrame  from InuStreamsPyth.
            @return  An instance of the HistogramFrame initialized with the specified InuStreamsPyth.HistogramF  object.
        """

    # @brief    InuStreamsPyth.HistogramF.
    #
    histogram_frame = None

    @property
    def histograms(self) -> VectorHistogramROI:
        # @brief    All histograms roi
        #
        # @return all histograms roi as VectorHistogramROI
        return self.histogram_frame.Histograms


class HistogramStream(BaseStream):
    """! Interface for Histogram service.

    Role: Controls Histogram streaming service and provides general or Histogram frames.
           Histogram frames are provided only if the connected device supports Histogram HW components.
           The caller application should be familiar with provided frames and should know how to interpret them.

    Responsibilities:
          1. Derives BaseStream class
          2. Knows how to acquire one depth image frame (pull)
          3. Knows how to provide a continuous stream of histogram frames (push)
    """

    # @brief    InuStreamsPyth.HistogramS.
    #
    _stream = None

    def __init__(self, stream: HistogramS):
        """! The Histogram stream class initializer.
            @param stream  The InuStreamsPyth.HistogramStream.
            @return  An instance of the Imu stream initialized with the specified InuStreamsPyth.HistogramS object.
        """
        BaseStream.__init__(self, stream)
        self._stream = stream

    def init(self) -> None:
        # @brief    Service initialization.
        #
        # Hall be invoked once before starting frames acquisition.
        self._stream.Init()

    def terminate(self) -> None:
        """!
            Stop frames acquisition, stop ant termination service.
        """
        self.register = None
        self.stop()
        self._stream.Terminate()

    def register(self, callback) -> None:
        # @brief    Registration/De registration for receiving stream frames (push)
        #
        # The provided callback function is called when a new frame is ready (non-blocking).
        # It shall be called only after a start() was invoked but before any invocation of a stop() is invoked.
        # @param  callback    The Callback function which is invoked when a new frame is ready.
        #                     Send nullptr to unregister for receiving frames.
        def _callback_cast(stream: HistogramS, frame: HistogramF, error: InuError) -> None:
            # @brief    Prototype of callback function which is used by the Register method.
            #
            # This function is invoked any time a frame is ready, or if an error occurs. The parameters of this function
            # are: Caller stream object, received Histogram frame and result code.
            BaseStream.callback(HistogramStream(stream), HistogramFrame(frame), Error(error))
        BaseFrame.callback = callback
        self._stream.Register(_callback_cast)
    register = property(None, register)

    @property
    def frame(self) -> HistogramFrame:
        # @brief   Retrieves new frame (pull)
        #
        # This method returns when a new frame is ready (blocking) or if an input timeout has elapsed.
        # It shall be called only after a start() was invoked but before any invocation of a stop() is invoked.
        # @return  The returned stereo frame.
        return HistogramFrame(self._stream.GetFrame())
