from .base_stream import BaseStream
from .depth_properties import CroppingROI
from .base_frame import BaseFrame
from .image_frame import ImageFrame
from .errors import Error
from .InuStreamsPyth import StereoF, StereoS, InuError
from enum import IntEnum


class StereoOutputFormat(IntEnum):
    """!  OutputFormat enum class.
        Represents all Stereo Stream's Output formats.
    """
    # Provides CImageFrames with ImageFormat::BGRA, on Android ImageFormat::RGBA
    DEFAULT = StereoS.EOutputFormat.Default
    # Provides ImageFrames as streamed by Inuitive chip, no additional processing on Host
    RAW = StereoS.EOutputFormat.Raw
    # Provides CImageFrames of ImageFormat::BGRA format
    BGRA = StereoS.EOutputFormat.BGRA
    # Provides CImageFrames of ImageFormat::RGBA format
    RGBA = StereoS.EOutputFormat.RGBA


class StereoFrame(BaseFrame):
    """!  Stereo Image frame.

    Role: Represents Stereo Image which comprises left and right sensor images.

    Responsibilities:
          1. Access to Left and Right frames separately
    """

    def __init__(self, frame: StereoF):
        self.stereo_frame = frame
        BaseFrame.__init__(self, frame)
        """! The Stereo Frame class initializer.
            @param frame  The StereoFrame  from InuStreamsPyth.
            @return  An instance of the Image initialized with the specified InuStreamsPyth.StereoFrame  object.
        """

    # @brief    InuStreamsPyth.StereoF.
    #
    stereo_frame = None

    @property
    def left_frame(self) -> ImageFrame:
        # @brief left getter
        #
        # @Detailed description:        Get left frame Image
        # @return                       The left frame Image
        try:
            return ImageFrame(self.stereo_frame.LeftFrame)
        except Exception as e:
            return None

    @property
    def right_frame(self) -> ImageFrame:
        # @brief left getter
        #
        # @Detailed description:        Get right frame Image
        # @return                       The right frame Image
        try:
            return ImageFrame(self.stereo_frame.RightFrame)
        except Exception as e:
            return None


class StereoStream(BaseStream, CroppingROI):
    """! Interface for Stereo service.

    Role: Controls Stereo images streaming service and provides Stereo Image frames.

    Responsibilities:
          1. Derives BaseStream class
          2.  Knows how to acquire one stereo frame (pull)
          3. Knows how to provide a continuous stream of stereo frames (push)
    """

    stream = None

    def __init__(self, stream: StereoS):
        """! The Stereo stream class initializer.
            @param stream  The InuStreamsPyth.StereoStream.
            @return  An instance of the Stereo stream initialized with the specified InuStreamsPyth.StereoStream object.
        """
        BaseStream.__init__(self, stream)
        CroppingROI.__init__(self, stream)
        self.stream = stream

    def init(self, form: StereoOutputFormat = StereoOutputFormat.DEFAULT) -> None:
        # @brief    Service initialization.
        #
        # Hall be invoked once before starting frames acquisition.
        # @param  format            The Output format that should be invoked.
        self.stream.Init(StereoS.EOutputFormat(form))

    def register(self, callback) -> None:
        """!
             Registration/De registration for receiving stream frames (push)

             All streams should use the same callback function when calling for “register”.
             You can find an example for a callback function here: “multithread_callback_example.py” where
              “_stream_callback_func“ can receive frames from different types of streams and acts differently based
              on the stream type.

             The provided callback function is called when a new frame is ready (non-blocking).
             It shall be called only after a start() was invoked but before any invocation of a stop() is invoked.
             If you need more than 1 stream you have to give in number of streams only 1 Callback function and after
             checking Stream type inside perform needed process.
             The parameters of this function are:
             @param  callback  The Callback function which is invoked when a new frame is ready.
              Send None to unregister for receiving frames.
         """

        def _callback_cast(stream: StereoS, frame: StereoF, error: InuError) -> None:
            """!
                Prototype of callback function which is used by the Register method.

                This function is invoked any time a frame is ready, or if an error occurs. The parameters of this
                function are:
                @param stream Caller image stream object.
                @param frame  Caller image frame object.
                @param error  Result code.
            """
            BaseStream.callback(StereoStream(stream), StereoFrame(frame), Error(error))
        BaseStream.callback = callback
        if callback is None:
            self.stream.Register(None)
        else:
            self.stream.Register(_callback_cast)
    register = property(None, register)

    @property
    def frame(self) -> StereoFrame:
        # @brief   Retrieves new frame (pull)
        #
        # This method returns when a new frame is ready (blocking) or if an input timeout has elapsed.
        # It shall be called only after a start() was invoked but before any invocation of a stop() is invoked.
        # @return  The returned stereo frame.
        fr = self.stream.GetFrame()
        return StereoFrame(fr)
