import cv2
import numpy as np
import time  
import cv2
import pytesseract
import numpy as np
from pytesseract import Output
from time import sleep
pytesseract.pytesseract.tesseract_cmd = r'C:/Program Files/Tesseract-OCR/tesseract.exe' # Need to change this for the PI
# kernal cap init
kernel = np.ones((5,5),np.uint8) # kernal is used for filter 
cap = cv2.VideoCapture(0) # cv video capture

time.sleep(0.5)

# window size default change 
cap.set(3,320)
cap.set(4,240)


# windows
cv2.namedWindow('tracking')

while(1):
    _, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #canny_img = canny(gray)
    #thresh = thresholding(gray)
    #opening_img = opening(gray)
    d = pytesseract.image_to_data(gray, output_type=Output.DICT, config='-l eng --oem 3 --psm 12')
    n_boxes = len(d['text'])
    for i in range(n_boxes):
        if int(d['conf'][i]) > 30:
            (text, x, y, w, h) = (d['text'][i], d['left'][i], d['top'][i], d['width'][i], d['height'][i])
            # don't show empty text
            if text and text.strip() != "":
                if text.upper() == "A" or text.upper() == "B" or text.upper() == "C":
                    print(text)
                    text = f"{text}"
                    frame = cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    frame = cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
                    
    #sleep(10)
    # Display the resulting frame
    cv2.imshow('tracking',frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows() 
        break

cap.release()
cv2.destroyAllWindows() 