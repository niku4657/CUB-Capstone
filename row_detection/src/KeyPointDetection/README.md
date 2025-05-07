The model files are too large to store in github, you can find them in our google drive: https://drive.google.com/drive/folders/1ovHwZw78PklW2kKdfv22cv4wUIOKaT6h

The model files must be unzipped and added to this directory. You can (and probably should) run the predict directory as a Python module using the command `python -m predict`. The proper command line arguments must also be given. You need to specify the image(s) to make a prediction on as well as the directory that contains the model file `saved_model.pb`.

Currently, we interop between C++ and Python. This isn't the most effecient thing in the world, and ideally keypoint detection should be ported over to C++. We also have 3x more training data is unused, and should be used to create a better model than the one we have provided in the drive. 
