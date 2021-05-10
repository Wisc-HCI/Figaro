from pyk4a import PyK4A

# Load camera with the default config
k4a = PyK4A()
k4a.connect()

# Get the next color frame without the depth (blocking function)
img_color = k4a.get_capture(color_only=True)

# Display with pyplot
from matplotlib import pyplot as plt
plt.imshow(img_color[:, :, 2::-1]) # BGRA to RGB
plt.show()