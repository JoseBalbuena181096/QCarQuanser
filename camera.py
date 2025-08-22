from pal.utilities.vision import Camera2D

class CameraProcessor:
    def __init__(self, camera_id='3', image_width=1640, image_height=820, sample_rate=60):
        self.sample_rate = sample_rate
        self.sample_time = 1 / self.sample_rate
        self.pos_x = 0
        self.counter = 0
        self.image_width = image_width
        self.image_height = image_height
        self.camera_id = camera_id

        print('Sample Time: ', self.sample_time)

        # Initialize the CSI camera
        self.my_cam = Camera2D(
            cameraId=self.camera_id,
            frameWidth=self.image_width,
            frameHeight=self.image_height,
            frameRate=self.sample_rate
        )

    def take_frame(self):
        self.my_cam.read()
        return self.my_cam.imageData

    def end_camera(self):
        self.my_cam.terminate()

# Usage example:
# camera_processor = CameraProcessor()
# frame = camera_processor.take_frame()
# # Process the frame as needed
# camera_processor.end_camera()