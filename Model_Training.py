## the model is trained on the Ultrlytics
hub.login('5af8a02bc2ccf284181d09a8897695bf197139c412')

model = YOLO('https://hub.ultralytics.com/models/NIqogMYrmgv8VW6hWjOF')
model.train()