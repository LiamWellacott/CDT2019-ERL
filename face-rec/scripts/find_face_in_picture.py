from PIL import Image
import face_recognition
import matplotlib.pyplot as plt


image = face_recognition.load_image_file("../faces/unknown/em3.jpeg")

face_locations = face_recognition.face_locations(image)

print("I found {} face(s) in this photograph.".format(len(face_locations)))

for face_loc in face_locations:
    top, right, bottom, left = face_loc
    print("A face is located at pixel Top: {}, Left: {}, Bottom: {}, Right: {}.".format(top, left, bottom, right))

    face_image = image[top:bottom, left:right]
    plt.imshow(face_image)
    plt.show()
    #pil_img = Image.fromarray(face_image)
    #pil_img.show()
