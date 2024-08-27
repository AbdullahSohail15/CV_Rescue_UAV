import cv2
import numpy as np


def InferenceTensorFlow(interpreter,image):
  """
  Function to perform inference on the image using TensorFlow Lite
  
  Args: 
    interpreter: TensorFlow Lite Interpreter
    image: Image to perform inference on

  Returns:
    rectangles: List of detected rectangles
    scores: List of scores of the detected rectangles
  """

  input_details = interpreter.get_input_details()
  output_details = interpreter.get_output_details()
  height = input_details[0]['shape'][1]
  width = input_details[0]['shape'][2]

  floating_model = False
  if input_details[0]['dtype'] == np.float32:
    floating_model = True

  initial_h, initial_w, channels = image.shape

  picture = cv2.resize(image, (width, height))

  input_data = np.expand_dims(picture, axis=0)
  if floating_model:
    input_data = (np.float32(input_data) - 127.5) / 127.5

  interpreter.set_tensor(input_details[0]['index'], input_data)

  interpreter.invoke()

  detected_boxes = interpreter.get_tensor(output_details[0]['index'])
  detected_classes = interpreter.get_tensor(output_details[1]['index'])
  detected_scores = interpreter.get_tensor(output_details[2]['index'])
  num_boxes = interpreter.get_tensor(output_details[3]['index'])

  scores=[]
  rectangles = []
  for i in range(int(num_boxes)):
    top, left, bottom, right = detected_boxes[0][i]
    classId = int(detected_classes[0][i])
    score = detected_scores[0][i]
    if score > 0.5 and classId==27:
      xmin = left * initial_w
      ymin = bottom * initial_h
      xmax = right * initial_w
      ymax = top * initial_h
      box = [int(xmin), int(ymin), int(xmax), int(ymax)]
      rectangles.append(box)
      scores.append(score)
  return rectangles,scores