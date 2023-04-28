!pip install roboflow
## using roboflow to annote the dataset

from roboflow import Roboflow
rf = Roboflow(api_key="cKTgDzmweV3S7pXkCHGY")
project = rf.workspace("object-detection-iadvs").project("hard-hat-sample-g5otd")
dataset = project.version(5).download("yolov5")
## in this dataset i used around 700 pictures and labeled them through roboflow
## the labled dataset download to a local folder with .yaml(classes file),folder(train,valid,test)
## the data are also in the csv format in order to call it when training the ML model