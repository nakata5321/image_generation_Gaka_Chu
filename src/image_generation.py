#!/usr/bin/env python3
from PIL import Image, ImageFont, ImageDraw
from googletrans import Translator
from std_msgs.msg import String
import rospkg
import rospy
import pandas as pd
from pytrends.request import TrendReq


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
    font = ImageFont.truetype(dirpath + "/font/usual_font.ttc", 100)
    draw.text((512, 512), data, (0, 0, 0), font=font, anchor="mm")
    img.show()
    path_to_pic = dirpath + "result" + data + ".png"
    img.save(path_to_pic)
    return path_to_pic


def translate(word: str, language: str = 'ja') -> str:
    """
    translate word to Japanese language

    :param word: given word to translate
    :param language: destination language

    :return:
    """
    rospy.loginfo(f"translate word to {language}")
    translator = Translator()
    trans = translator.translate(word, dest=language)
    rospy.loginfo(f"translation is {trans.text}")
    return trans.text


def get_word() -> str:
    """
    Request data from Google's Hot Searches section and return top word
    """
    pytrend = TrendReq()
    df = pytrend.trending_searches()
    return df[0][0]


def main() -> None:
    # init ros node
    rospy.init_node("image_generation", anonymous=True, disable_signals=True)
    pub = rospy.Publisher('run', String, queue_size=1)
    rospack = rospkg.RosPack()
    dirpath = rospack.get_path("image_generation")
    rospy.loginfo("node is up")
    # get word
    rospy.loginfo("find the most popular word")
    word = get_word()
    rospy.loginfo(f"the word is: {word}")
    # translate word
    tr_word = translate(word)
    # create image
    rospy.loginfo("create picture with word")
    path_to_pic = create_image(tr_word, dirpath)
    rospy.loginfo(f"path to picture is: {path_to_pic}")
    pub.publish(path_to_pic)


if __name__ == "__main__":
    main()
