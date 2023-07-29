import cv2
import numpy as np

def color_filter(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    lower_red = np.array([0, 100, 100])  # Plage de valeurs pour le rouge (vous pouvez ajuster ces valeurs si nécessaire)
    upper_red = np.array([10, 255, 255])  # Plage de valeurs pour le rouge
    red_mask = cv2.inRange(hsv, lower_red, upper_red)
    masked_image = np.zeros_like(image)
    masked_image[red_mask != 0] = [255, 255, 255]  # Remplacez le rouge par du blanc
    return masked_image

img = cv2.imread("/home/skn/Musique/Filtre_Task1/test4.jpg")
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
img_filtree = color_filter(img_rgb)
img_gray = cv2.cvtColor(img_filtree, cv2.COLOR_RGB2GRAY)
contours, _ = cv2.findContours(img_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
mask = np.zeros_like(img)
cv2.drawContours(mask, contours, -1, (255, 255, 255), thickness=cv2.FILLED)
mask_outside_contours = cv2.bitwise_not(mask)
# Inverser le masque pour obtenir les couleurs en dehors du contour en noir
img_with_black = cv2.bitwise_and(img, cv2.bitwise_not(mask_outside_contours))

cv2.imshow("Image avec contours encadrant le rouge", img_with_black)
cv2.waitKey(0)
cv2.destroyAllWindows()
