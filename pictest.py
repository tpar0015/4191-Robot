import camera
import ceiling


camera.capture(0, "/home/hanlinpi/Desktop/Project_4191/test_pic1.jpg")
while True:
    camera.capture(0.2, "/home/hanlinpi/Desktop/Project_4191/test_pic2.jpg")
    x_translation, y_translation, rotation = ceiling.ceiling_base_localization(
        "/home/hanlinpi/Desktop/Project_4191/test_pic1.jpg",
        "/home/hanlinpi/Desktop/Project_4191/test_pic2.jpg",
        None,
    )
    print(
        "\n x_trans: ",
        x_translation,
        " y_trans: ",
        y_translation,
        " rot: ",
        rotation,
        "\n",
    )
    pass
