import torch
import cv2
import numpy as np

# this code is running in the Yahboom robot (Ros enviroment)

# Load the pre-trained machine learning model
model_path = 'Documents/Carrots_Detector/model1.pt'
model = torch.load(model_path)

# Initialize the camera
camera = cv2.VideoCapture(0)

# Set camera resolution
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# Loop to continuously read and process frames from the camera
while True:
    # Read a frame from the camera
    ret, frame = camera.read()

    # Preprocess the frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    resized = cv2.resize(gray, (28, 28), interpolation=cv2.INTER_AREA)
    scaled = resized / 255.0

    # Convert the frame to a PyTorch tensor
    tensor = torch.from_numpy(scaled).float()

    # Add a batch dimension to the tensor
    tensor = tensor.unsqueeze(0)

    # Run the model on the tensor
    output = model(tensor)

    # Process the output
    _, predicted = torch.max(output.data, 1)

    # Display the output
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame, str(predicted.item()), (10, 30), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
    cv2.imshow('Output', frame)

    # Check for keypresses
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

# Release the camera and close all windows
camera.release()
cv2.destroyAllWindows()
