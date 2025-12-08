from .library_loader import *
from .base_stream import StreamState
from .shared import StreamType
from .image_frame import ImageFrame
from .stereo import StereoFrame
from .features_tracking import FeaturesTrackingFrame
from .InuStreamsPyth import InjectionS, Sensor, ImageF, StereoF, FeaturesTrackingF
from typing import Union
import numpy as np


class InjectionStream:
    """! Interface for Injection Input  service.

    Role: Controls input injection streaming frames service.

    Responsibilities:
          1. Knows how to inject input frame buffer
    """
    # @brief    InuStreamsPyth.InjectionS.
    #
    _stream: InjectionS = None

    # @brief    InuStreamsPyth.InuSensor.
    #
    _sensor: Sensor = None

    def __init__(self, stream: InjectionS, sensor: Sensor):
        """! The Injection stream class initializer. @param stream  The InuStreamsPyth.InjectionStream. @param sensor
         The InuStreamsPyth.InuSensor. @return  An instance of the Injection stream initialized with the specified
            InuStreamsPyth.InjectionStream and InuStreamsPyth.InuSensor objects.
        """
        self._stream = stream
        self._sensor = sensor

    def init(self, buffer_size: int = 0) -> None:
        # @brief    Service initialization.
        #
        # @param  buffer_size    Frame  Data buffer size used in memory pool build.
        # @return InuError    Operation status which indicates on success or failure.
        if buffer_size == 0:
            self._stream.Init()
        else:
            self._stream.Init(buffer_size)

    def terminate(self) -> None:
        # @brief    Service termination.
        #
        # Shall be invoked when the service is no longer in use and after frames acquisition has stopped.
        self._stream.Stop()
        self._stream.Terminate()

    def start(self) -> None:
        # @brief    Start acquisition of frames.
        #
        # Shall be invoked only after the service is successfully initialized and before any request
        # for new frame (push or pull).
        self._stream.Start()

    def stop(self) -> None:
        # @brief    Stop acquisition of frames.
        #
        self._stream.Stop()

    def state(self) -> StreamState:
        # @brief    Stop acquisition of frames.
        #
        return StreamState(self._stream.State())

    def inject_file_to_stream(self, stream_type: StreamType, frame_path: str) -> Union[
        ImageFrame, FeaturesTrackingFrame, StereoFrame]:
        #   @brief Inject frame file to Stream
        #
        #   @param stream_type - EStreamType currently support EStreamType.Depth, EStreamType.Stereo,
        #       EStreamType.Tracking and EStreamType.FeaturesTracking
        #   @param  frame_path - Image Frame path
        #   @return result frame
        if stream_type == StreamType.DEPTH:
            frame = ImageF()
            self._sensor.InjectStereoImage2DepthStream(frame_path, frame)
            return ImageFrame(frame)
        elif stream_type == StreamType.STEREO:
            frame = StereoF()
            self._sensor.InjectStereoImage2DepthStream(frame_path, frame)
            return StereoFrame(frame)
        elif stream_type == StreamType.TRACKING:
            frame = ImageF()
            self._sensor.InjectTrackingImage2TrackingStream(frame_path, frame)
            return ImageFrame(frame)
        elif stream_type == StreamType.FEATURES_TRACKING:
            frame = FeaturesTrackingFrame()
            self._sensor.InjectTrackingImage2FeaturesTrackingStream(frame_path, frame)
            return FeaturesTrackingFrame(frame)
        return None

    def inject_data_to_stream(self, stream_type: StreamType, data: np.array) -> Union[ImageFrame,
    FeaturesTrackingFrame, StereoFrame]:
        # @brief Inject frame data to Stram
        #
        #   @param stream_type - EStreamType currently support EStreamType.Depth, EStreamType.Stereo,
        #       EStreamType.Tracking and EStreamType.FeaturesTracking
        #   @param data - pointer to data buffer
        #   @return result frame
        height, width, bytes_per_pixel = data.shape
        if stream_type == StreamType.DEPTH:
            frame = ImageF()
            self._sensor.InjectStereoImage2DepthStream(width, height, bytes_per_pixel, data, frame)
            return ImageFrame(frame)
        elif stream_type == StreamType.STEREO:
            frame = StereoF()
            self._sensor.InjectStereoImage2StereoStream(width, height, bytes_per_pixel, data, frame)
            return StereoFrame(frame)
        elif stream_type == StreamType.TRACKING:
            frame = ImageF()
            self._sensor.InjectTrackingImage2TrackingStream(width, height, bytes_per_pixel, data, frame)
            return ImageFrame(frame)
        elif stream_type == StreamType.FEATURES_TRACKING:
            frame = FeaturesTrackingF()
            self._sensor.InjectTrackingImage2FeaturesTrackingStream(width, height, bytes_per_pixel, data, frame)
            return FeaturesTrackingFrame(frame)
        return None
