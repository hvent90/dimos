OpenCV chessboard calibration fixtures.

Source: https://github.com/opencv/opencv/tree/4.x/samples/data

Files: `left01.jpg` through `left09.jpg`, downloaded from the official OpenCV
`4.x` branch raw URLs under `samples/data`.

These are real chessboard calibration sample images used by OpenCV examples.
The test suite verifies their SHA-256 hashes, image dimensions, and `9x6` inner
corner detections before using them as a folder fixture for `dimos
cameracalibrate`.
