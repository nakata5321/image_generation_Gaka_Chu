#!/usr/bin/env python3
from PIL import Image, ImageFont, ImageDraw
from googletrans import Translator
import rospkg
import rospy


def create_image(data: str, dirpath: str) -> str:
    """
    function create image and insert text on it

    -----
    :param dirpath: path to the folder with ROS package
    :rtype: str
    :param data: text to insert
    """
    # create empty image
    img = Image.new("RGB", (1024, 1024), color=(255, 255, 255))
    draw = ImageDraw.Draw(img)
    # use a truetype font
    font = ImageFont.truetype("../font/usual_font.ttc", 100)
    draw.text((512, 512), data, (0, 0, 0), font=font, anchor="mm")
    img.show()
    path_to_pic = dirpath + "result" + data + ".png"
    img.save(path_to_pic)
    return path_to_pic


def translate(word: str, language: str = 'ja') -> str:
    translator = Translator()
    trans = translator.translate(word, dest=language)
    return trans.text


def main() -> None:
    # init ros node
    rospy.init_node("image_generation", anonymous=True, disable_signals=True)
    rospack = rospkg.RosPack()
    dirpath = rospack.get_path("image_generation")

    # translate word
    tr_word = translate("Hello World!")

    # create image
    create_image(tr_word, dirpath)


if __name__ == "__main__":
    main()
