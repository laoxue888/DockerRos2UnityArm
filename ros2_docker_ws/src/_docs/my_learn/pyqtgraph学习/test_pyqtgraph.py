"""
This example demonstrates the use of ImageView with 3-color image stacks.
ImageView is a high-level widget for displaying and analyzing 2D and 3D data.
ImageView provides:

  1. A zoomable region (ViewBox) for displaying the image
  2. A combination histogram and gradient editor (HistogramLUTItem) for
     controlling the visual appearance of the image
  3. A timeline for selecting the currently displayed frame (for 3D data only).
  4. Tools for very basic analysis of image data (see ROI and Norm buttons)

"""

import numpy as np

import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets

# Interpret image data as row-major instead of col-major
pg.setConfigOptions(imageAxisOrder='row-major')

app = pg.mkQApp("ImageView Example")

## Create window with ImageView widget
win = QtWidgets.QMainWindow()
win.resize(800,800)
imv = pg.ImageView(discreteTimeLine=True, levelMode='rgba')
win.setCentralWidget(imv)
win.show()
win.setWindowTitle('pyqtgraph example: ImageView')
imv.setHistogramLabel("Histogram label goes here")

## Create random 3D data set with time varying signals
dataRed = np.ones((100, 200, 200)) * np.linspace(90, 150, 100)[:, np.newaxis, np.newaxis]
dataRed += pg.gaussianFilter(np.random.normal(size=(200, 200)), (5, 5)) * 100
dataGrn = np.ones((100, 200, 200)) * np.linspace(90, 180, 100)[:, np.newaxis, np.newaxis]
dataGrn += pg.gaussianFilter(np.random.normal(size=(200, 200)), (5, 5)) * 100
dataBlu = np.ones((100, 200, 200)) * np.linspace(180, 90, 100)[:, np.newaxis, np.newaxis]
dataBlu += pg.gaussianFilter(np.random.normal(size=(200, 200)), (5, 5)) * 100

data = np.concatenate(
    (dataRed[:, :, :, np.newaxis], dataGrn[:, :, :, np.newaxis], dataBlu[:, :, :, np.newaxis]), axis=3
)


# 创建矩形ROI
# roi = pg.RectROI([20, 20], [40, 40], pen='r')
# imv.addItem(roi)

roi_1 = pg.PolyLineROI([[20, 20], [40, 40], [50, 30], [20,10]], closed=True)
imv.addItem(roi_1)

imv.clear()

# Display the data and assign each frame a time value from 1.0 to 3.0
imv.setImage(data, xvals=np.linspace(1., 3., data.shape[0]))
imv.play(10)

# Start up with an ROI
imv.ui.roiBtn.setChecked(True)
imv.roiClicked()

if __name__ == '__main__':
    pg.exec()
