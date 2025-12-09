from .base_stream import BaseStream
from .depth_properties import DepthProperties, CroppingROI
from .image_frame import ImageFrame
from .errors import Error
from .InuStreamsPyth import ImageS, ImageF, InuError

from enum import IntEnum


class ImageOutputFormat(IntEnum):
    """!  OutputFormat enum class.
        Define Depth Stream's Output formats
    """
    # Provides CImageFrames with ImageFormat::BGRA, on Android ImageFormat::RGBA
    DEFAULT = ImageS.EOutputFormat.Default
    # Provides CImageFrames as streamed by Inuitive chip, no additional processing on Host
    RAW = ImageS.EOutputFormat.Raw
    # Provides CImageFrames of ImageFormat::BGRA format
    BGRA = ImageS.EOutputFormat.BGRA
    # Provides CImageFrames of ImageFormat::RGBA format
    RGBA = ImageS.EOutputFormat.RGBA
    # Provides CImageFrame of ImageFormat::BGR format
    BGR = ImageS.EOutputFormat.BGR


class ImagePostProcessing(IntEnum):
    """!  ImagePostProcessing enum class.
        Define Depth Stream's Post Processing methods.
    """
    # No algorithm is running
    NONE_PROCESSING = 0
    # GammaCorrected image, format is similar to eDefault
    GAMMA_CORRECT = ImageS.EPostProcessing.GammaCorrect


class ImageStream(BaseStream, CroppingROI):
    """! Interface for Image service.

    Role: Controls depth images streaming service and provides image frames.

    Responsibilities:
          1. Derives BaseStream class
          2. Knows how to acquire one depth image frame (pull)
          3. Knows how to provide a continuous stream of depth image frames (push)
    """

    # @brief    InuStreamsPyth.ImageS.
    #
    stream: ImageS = None

    def __init__(self, stream: ImageS):
        BaseStream.__init__(self, stream)
        CroppingROI.__init__(self, stream)
        self.stream = stream
        """! The Depth stream class initializer.
            @param stream  The InuStreamsPyth.ImageS.
            @return  An instance of the Image stream initialized with the specified InuStreamsPyth.ImageS object.
        """

    def init(self, form: ImageOutputFormat = ImageOutputFormat.DEFAULT,
             processing: ImagePostProcessing = ImagePostProcessing.NONE_PROCESSING) -> None:
        # @brief    Service initialization.
        #
        # Hall be invoked once before starting frames acquisition.
        # @param  format            The Output format that should be invoked.
        # @param  processing        The PostProcessing algorithms that should be invoked.
        # @param  regChannelID      The Register Channel ID - in the case of  RegisteredImage only
        self.stream.Init(ImageS.EOutputFormat(form), ImageS.EPostProcessing(processing))

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

        def _callback_cast(stream: ImageS, frame: ImageF, error: InuError) -> None:
            """!
                Prototype of callback function which is used by the Register method.

                This function is invoked any time a frame is ready, or if an error occurs. The parameters of this
                function are:
                @param stream Caller image stream object.
                @param frame  Caller image frame object.
                @param error  Result code.
            """
            BaseStream.callback(ImageStream(stream), ImageFrame(frame), Error(error))
        BaseStream.callback = callback
        if callback is None:
            self.stream.Register(None)
        else:
            self.stream.Register(_callback_cast)
    register = property(None, register)

    @property
    def frame(self) -> ImageFrame:
        # @brief   Retrieves new frame (pull)
        #
        # This method returns when a new frame is ready (blocking) or if an input timeout has elapsed.
        # It shall be called only after a start() was invoked but before any invocation of a stop() is invoked.
        # @return  The returned depth frame (Z-buffer).
        return ImageFrame(self.stream.GetFrame())
