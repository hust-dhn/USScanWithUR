from .base_stream import BaseStream
from .depth_properties import DepthProperties, CroppingROI
from .image_frame import ImageFrame
from .errors import Error
from .InuStreamsPyth import DepthS, ImageF, InuError

from enum import IntEnum


class DepthOutputFormat(IntEnum):
    """!  OutputFormat enum class.
        Define Depth Stream's Output formats.
    """
    # Provides ImageFrame with EImageFormat::Depth
    DEFAULT = DepthS.EOutputFormat.Default
    # Provides ImageFrame with EImageFormat::eDepth or EImageFormat::eDisparity  as streamed by Inuitive's chip,
    #   no additional processing on Host
    RAW = DepthS.EOutputFormat.Raw
    # Provides ImageFrame with EImageFormat::BGRA
    BGRA = DepthS.EOutputFormat.BGRA
    # Provides ImageFrame with EImageFormat::RGBA
    RGBA = DepthS.EOutputFormat.RGBA
    # Provides ImageFrame with EImageFormat::Depth
    DEPTH = DepthS.EOutputFormat.Depth
    # Provides ImageFrame with EImageFormat::RGB
    RGB = DepthS.EOutputFormat.RGB


class DepthPostProcessing(IntEnum):
    """!  PostProcessing enum class.
        Define Depth Stream's Post Processing methods.
    """
    # No algorithm is running
    NONE_PROCESSING = DepthS.EPostProcessing.NoneProcessing
    # Remove/Fill Holes
    BLOB = DepthS.EPostProcessing.Blob
    # Apply TemporalFilter to depth data
    TEMPORAL = DepthS.EPostProcessing.Temporal
    # Remove Outliers/Blobs by size
    OUTLIER_REMOVE = DepthS.EPostProcessing.OutlierRemove
    # Fill Holes by radius
    HOLE_FILL = DepthS.EPostProcessing.HoleFill
    # Static Temporal filter
    STATIC_TEMPORAL = DepthS.EPostProcessing.StaticTemporal
    # Default, defined by host
    DEFAULT = DepthS.EPostProcessing.DefaultPP


class DepthStream(BaseStream, DepthProperties, CroppingROI):
    """! Interface for Depth service.

    Role: Controls depth images streaming service and provides depth image frames.

    Responsibilities:
          1. Derives BaseStream class
          2. Knows how to acquire one depth image frame (pull)
          3. Knows how to provide a continuous stream of depth image frames (push)
    """

    # @brief    InuStreamsPyth.DepthS.
    #
    _stream = None

    def __init__(self, stream: DepthS):
        """! The Depth stream class initializer.
            @param stream  The InuStreamsPyth.DepthS.
            @return  An instance of the Depth stream initialized with the specified InuStreamsPyth.DepthS object.
        """
        BaseStream.__init__(self, stream)
        DepthProperties.__init__(self, stream)
        CroppingROI.__init__(self, stream)
        self._stream = stream

    def init(self, form: DepthOutputFormat = DepthOutputFormat.DEFAULT,
             processing: DepthPostProcessing = DepthPostProcessing.NONE_PROCESSING) -> None:
        # @brief    Service initialization.
        #
        # Hall be invoked once before starting frames acquisition.
        # @param  format            The Output format that should be invoked.
        # @param  processing        The PostProcessing algorithms that should be invoked.
        self._stream.Init(DepthS.EOutputFormat(form), DepthS.EPostProcessing(processing))

    def terminate(self) -> None:
        """!
            Stop frames acquisition, stop ant termination service.
        """
        self.register = None
        self.stop()
        self._stream.Terminate()

    def register(self, callback) -> None:
        """!
            Registration/De registration for receiving stream frames (push)

            The provided callback function is called when a new frame is ready (non-blocking).
            It shall be called only after a start() was invoked but before any invocation of a stop() is invoked.
            @param  callback  The Callback function which is invoked when a new frame is ready. Send nullptr to
                unregister for receiving frames.
        """
        def _callback_cast(stream: DepthS, frame: ImageF, error: InuError) -> None:
            """!
                Prototype of callback function which is used by the Register method.

                This function is invoked any time a frame is ready, or if an error occurs. The parameters of this
                function are:
                @param stream Caller depth stream object.
                @param frame  Caller depth frame object.
                @param error  Result code.
            """
            BaseStream.callback(DepthStream(stream), ImageFrame(frame), Error(error))
        BaseStream.callback = callback
        self._stream.Register(_callback_cast)
    register = property(None, register)

    @property
    def frame(self) -> ImageFrame:
        # @brief   Retrieves new frame (pull)
        #
        # This method returns when a new frame is ready (blocking) or if an input timeout has elapsed.
        # It shall be called only after a start() was invoked but before any invocation of a stop() is invoked.
        # @return  The returned depth frame (Z-buffer).
        fr = self._stream.GetFrame()
        print(" Depth.frame {} Depth.frame ".format(fr.FrameIndex))
        return ImageFrame(fr)

    @property
    def default_post_processing_mode(self) -> int:
        # @brief default_post_processing_mode
        #
        # Detailed description:        Get current PostProcessingModeDefault.
        #                              It's possible to use this function before or without starting the _stream.
        # @return                      Return the default PostProcessingMode
        return self._stream.DefaultPostProcessingMode()

    def mipi_on(self, on: bool) -> None:
        # @brief    start/stop Mipi
        #
        # Shall be invoked once after a call to on = True in order to start transmitting frames over Mipi.
        # Shall be invoked once after a call to on = False in order to stop transmitting frames over Mipi.
        # This functionality is applicable only when complex graph with mipi connectors are in used.
        # If several streams should be transmitted over mipi then the order of activation should be:
        # "Start / Stop" all streams then "startMipi" of all streams
        if on:
            self._stream.startMipi()
        else:
            self._stream.stopMipi()
    mipi_on = property(None, mipi_on)
