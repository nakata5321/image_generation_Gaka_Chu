#!/usr/bin/env python3
from image_generation.srv import ImageGenerate, ImageGenerateResponse
from PIL import Image, ImageFont, ImageDraw
from pytrends.request import TrendReq
from googletrans import Translator
from std_msgs.msg import String
import rospy


def create_image(data: str, dirpath: str) -> None:
    """
    function create image and insert text on it

    -----
    :param dirpath: path to the folder with ROS package
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
    rospy.loginfo(f"path to picture is: {path_to_pic}")
    pub_run.publish(path_to_pic)


def translate(data: String) -> None:
    """
    translate word to Japanese

    :param data: given word to translate
    """
    rospy.loginfo(f"get word - {data.data}")
    rospy.loginfo(f"translate word to Japanese")
    dir_path = rospy.get_param("~dir_path")
    translator = Translator()
    trans = translator.translate(data.data, dest='ja')
    rospy.loginfo(f"translation is {trans.text}")

    rospy.loginfo("creating picture with word")
    create_image(trans.text, dir_path)


def get_word(req: None) -> str:
    """
    Request data from Google's Hot Searches section and return top word
    """
    rospy.loginfo("find the most popular word")
    pytrend = TrendReq()
    df = pytrend.trending_searches()
    # publish generated word to topic '/word_for_gakachu'
    pub_gen.publish(df[0][0])
    return ImageGenerateResponse(df[0][0])


if __name__ == "__main__":
    rospy.init_node("image_generation", anonymous=True, disable_signals=True)
    rospy.loginfo("node initializing")
    pub_run = rospy.Publisher('run', String, queue_size=1)
    pub_gen = rospy.Publisher('/word_for_gakachu', String, queue_size=1)
    rospy.Subscriber("/word_for_gakachu", String, translate)
    rospy.loginfo("node is up")
    generate_image_service = rospy.Service('generate_image', ImageGenerate, get_word)
    rospy.loginfo("if you want to generate word, call 'generate_image' service")
    rospy.spin()
