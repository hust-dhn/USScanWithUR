from .base_frame import BaseFrame
from .InuStreamsPyth import ImageF
import numpy as np


class ImageFrame(BaseFrame):
    """!  Image frame.

    Role: Represents  an image that is  provided  by InuDev streams

    Responsibilities:
          1. Image attributes: format, scale, width, height and number of bytes per pixel.
          2. Knows how to manage the image buffer.
    """

    def __init__(self, frame: ImageF):
        self.image_frame = frame
        BaseFrame.__init__(self, frame)
        """! 
            The Image Frame class initializer.
            @param frame  The ImageF  from InuStreamsPyth.
            @return  An instance of the Image initialized with the specified InuStreamsPyth.ImageF  object.
        """

    # @brief    InuStreamsPyth.ImageF.
    #
    image_frame = None

    @property
    def width(self) -> int:
        # @brief    The Image width.
        return self.image_frame.Width

    @width.setter
    def width(self, value: int) -> None:
        # @brief    Image width setter.
        #
        # @param[in]   value	    New Image width.
        self.image_frame.Width = value

    @property
    def height(self) -> int:
        # @brief    The image height.
        return self.image_frame.Height

    @height.setter
    def height(self, value: int) -> None:
        # @brief    Image height setter.
        #
        # @param[in]   value	 The new Image height.
        self.image_frame.Height = value

    @property
    def cropped_image_width(self) -> int:
        # @brief    The width of image that is cropped by depth engine, is relevant when the image padding is requested.
        #           cropped_image_width <=  Width.
        return self.image_frame.CroppedImageWidth

    @cropped_image_width.setter
    def cropped_image_width(self, value: int) -> None:
        # @brief    The Image Cropped Image Width setter.
        #
        # @param[in]   value	    The new Image CroppedImageWidth.
        self.image_frame.CroppedImageWidth = value

    @property
    def cropped_image_height(self) -> int:
        # @brief    The height of image  that is cropped by depth engine, is relevant when the image padding is
        #           requested cropped_image_height <=  Height.
        return self.image_frame.CroppedImageHeight

    @cropped_image_height.setter
    def cropped_image_height(self, value: int) -> None:
        # @brief    The Image Cropped Image Width setter.
        #
        # @param[in]   value	    The new Image CroppedImageWidth.
        self.image_frame.CroppedImageWidth = value

    @property
    def cropped_image_top_left_w(self) -> int:
        # @brief    The width of image that is cropped by depth engine, is relevant when the image padding is
        #           requested. CroppedImageWidth <=  Width.
        return self.image_frame.CroppedImageTopLeftW

    @cropped_image_top_left_w.setter
    def cropped_image_top_left_w(self, value: int) -> None:
        # @brief    The Image Cropped ImageTop Left W setter.
        #
        # @param[in]   value	    The new Image CroppedImageTopLeftW.
        self.image_frame.CroppedImageTopLeftW = value

    @property
    def cropped_image_top_left_h(self) -> int:
        # @brief    The top left height offset of cropped image, is relevant when the image padding is requested.
        #           CroppedImageTopLeftH >= 0.
        return self.image_frame.CroppedImageTopLeftH

    @cropped_image_top_left_h.setter
    def cropped_image_top_left_h(self, value: int) -> None:
        # @brief    The Image Cropped Image Top Left H setter.
        #
        # @param[in]   value	    The new Image CroppedImageTopLeftH.
        self.image_frame.CroppedImageTopLeftH = value

    @property
    def format(self) -> int:
        # @brief    The Image format getter.
        return self.image_frame.Format

    @format.setter
    def format(self, value: int) -> None:
        # @brief    Image format setter.
        #
        # @param[in]   value	    New Image format.
        self.image_frame.Format = value

    @property
    def bytes_per_pixel(self) -> int:
        # @brief    The number of bytes that are used to represent each pixel.
        return self.image_frame.BytesPerPixel

    @property
    def buffer(self) -> np.array:
        # @brief   Pixels data buffer.
        if self.bytes_per_pixel == 2:
            return np.asanyarray(self.image_frame, np.uint32)
        elif self.bytes_per_pixel == 4:
            img = np.frombuffer(self.image_frame, np.uint8)
            img = img.reshape(self.height, self.width, 4)
            return img
        else:
            raise Exception(f"This pixel size {self.bytes_per_pixel}  doesn't supported yet.")
