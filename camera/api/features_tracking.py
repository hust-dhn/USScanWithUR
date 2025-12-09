from .base_stream import BaseStream
from .depth_properties import CroppingROI
from .base_frame import BaseFrame
from .InuStreamsPyth import FeaturesTrackingF, FeaturesTrackingS, InuError, FeaturesTrackingParsedData,\
    FeaturesTrackingProcessedData

from typing import Union

from enum import IntEnum


class FeaturesTrackingOutputFormat(IntEnum):
    """!  OutputFormat enum class.
        Define FeaturesTracking Stream's Output types.
    """
    # Provides FeaturesTracking Frames with Parsed output type
    DEFAULT = FeaturesTrackingS.EOutputType.Parsed
    # Provides FeaturesTracking Frames with Raw output type
    RAW = FeaturesTrackingS.EOutputType.Raw
    # Provides FeaturesTracking Frames with Parsed output type
    PARSED = FeaturesTrackingS.EOutputType.Parsed
    # Provides FeaturesTracking Frames with Processed output type
    PROCESSED = FeaturesTrackingS.EOutputType.Processed


class FeaturesTrackingFrame(BaseFrame):
    """!  Features Tracking frame.

    Role: Represents Features Tracking frame.

    Responsibilities:
          1. Access to Features Tracking dats.
    """

    # @brief    InuStreamsPyth.FeaturesTrackingF.
    #
    features_tracking_frame = None

    def __init__(self, frame: FeaturesTrackingF):
        self.features_tracking_frame = frame
        BaseFrame.__init__(self, frame)
        """! 
            The Features Tracking Frame class initializer. 
            @param frame  The FeaturesTrackingF  from  InuStreamsPyth. 
            @return  An instance of the Features Tracking frame initialized with the specified
             `InuStreamsPyth.FeaturesTrackingF  object.
        """

    @property
    def data(self) -> Union[FeaturesTrackingParsedData, FeaturesTrackingProcessedData]:
        # @brief data getter
        #
        # @Detailed description:        Get ParsedData array
        # @return                       The ParsedData or ProcessedData array
        if self.output_type == FeaturesTrackingOutputFormat.DEFAULT or \
                self.output_type == FeaturesTrackingOutputFormat.PARSED:
            return self.features_tracking_frame.ParsedData
        elif self.output_type == FeaturesTrackingOutputFormat.PROCESSED:
            return self.features_tracking_frame.ProcessedData
        else:   # self.output_type == Raw:
            return None

    @property
    def image_width(self) -> int:
        # @brief Image Width getter
        #
        # @Detailed description:        Get Image Width
        # @return                       The Image Width
        return self.features_tracking_frame.ImageWidth

    @property
    def image_height(self) -> int:
        # @brief Image Height getter
        #
        # @Detailed description:        Get Image Height
        # @return                       The Image Height
        return self.features_tracking_frame.ImageHeight

    @property
    def output_type(self) -> FeaturesTrackingOutputFormat:
        # @brief Output Type getter
        #
        # @Detailed description:        Get Output Type
        # @return                       The Output Type
        return FeaturesTrackingOutputFormat(self.features_tracking_frame.OutputType)

    @property
    def key_point_number(self) -> int:
        # @brief Key Point Number getter
        #
        # @Detailed description:        Get Key Point Number
        # @return                       The Key Point Number
        return self.features_tracking_frame.KeyPointNumber

    @property
    def key_point_number_right(self) -> int:
        # @brief Key Point Number Right getter
        #
        # @Detailed description:        Get Key Point Right Number
        # @return                       The Key Point Right Number
        return self.features_tracking_frame.KeyPointNumberRight

    @property
    def key_point_number_left(self) -> int:
        # @brief Key Point Number Left getter
        #
        # @Detailed description:        Get Key Point Left Number
        # @return                       The Key Point Left Number
        return self.features_tracking_frame.KeyPointNumberLeft

    @property
    def descriptor(self):
        # @brief descriptor getter
        #
        # @Detailed description:        Get descriptor
        # @return                       The descriptor
        return self.features_tracking_frame.Descriptor

    @property
    def descriptor_size(self) -> int:
        # @brief descriptor size getter
        #
        # @Detailed description:        Get descriptor size
        # @return                       The descriptor size
        return self.features_tracking_frame.DescriptorSize


class FeaturesTrackingStream(BaseStream, CroppingROI):
    """! Interface for FeaturesTracking service.

    Role: Controls FeaturesTracking streaming service and provides Stereo FeaturesTracking frames.

    Responsibilities:
          1. Derives BaseStream class
          2. Knows how to acquire one stereo frame (pull)
          3. Knows how to provide a continuous stream of stereo frames (push)
    """

    # @brief    InuStreamsPyth.FeaturesTrackingS.
    #
    _stream = None

    def __init__(self, stream: FeaturesTrackingS):
        """! The FeaturesTracking stream class initializer.
            @param stream  The InuStreamsPyth.FeaturesTrackingS.
            @return  An instance of the FeaturesTracking stream initialized with the specified
            InuStreamsPyth.FeaturesTrackingS object.
        """
        BaseStream.__init__(self, stream)
        self._stream = stream

    def init(self, form: FeaturesTrackingOutputFormat = FeaturesTrackingOutputFormat.DEFAULT) -> None:
        # @brief    Service initialization.
        #
        # Hall be invoked once before starting frames acquisition.
        # @param  format            The Output type that should be invoked.
        self._stream.Init(FeaturesTrackingS.EOutputType(form))

    def register(self, callback) -> None:
        """!
            Registration/De registration for receiving stream frames (push)

            All streams should use the same callback function when calling for “register”.
            You can find an example for a callback function here: “multithread_callback_example.py” where
             “_stream_callback_func“ can receive frames from different types of streams and acts differently based
             on the stream type.

            The provided callback function is called when a new frame is ready (non-blocking).
            It shall be called only after a start() was invoked but before any invocation of a stop() is invoked.
            @param  callback  The Callback function which is invoked when a new frame is ready. Send nullptr to
                unregister for receiving frames.
        """
        def _callback_cast(stream: FeaturesTrackingS, frame: FeaturesTrackingF, error: InuError) -> None:
            """!
                Prototype of callback function which is used by the Register method.

                This function is invoked any time a frame is ready, or if an error occurs. The parameters of this
                function are:
                @param stream Caller image stream object.
                @param frame  Caller image frame object.
                @param error  Result code.
            """
            BaseStream.callback(FeaturesTrackingStream(stream), FeaturesTrackingFrame(frame), error)
        BaseStream.callback = callback
        if callback is None:
            self._stream.Register(None)
        else:
            self._stream.Register(_callback_cast)
    register = property(None, register)

    @property
    def frame(self) -> FeaturesTrackingFrame:
        # @brief   Retrieves new frame (pull)
        #
        # This method returns when a new frame is ready (blocking) or if an input timeout has elapsed.
        # It shall be called only after a start() was invoked but before any invocation of a stop() is invoked.
        # @return  The returned FeaturesTracking frame.
        return FeaturesTrackingFrame(self._stream.GetFrame())
