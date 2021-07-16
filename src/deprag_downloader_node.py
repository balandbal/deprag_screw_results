#!/usr/bin/env python
import rospy
from deprag_downloader.srv import download_request
from selenium.webdriver import Firefox, FirefoxProfile
from selenium.webdriver.firefox.options import Options
from selenium.webdriver.support.select import Select

downloadDirectory = "/home/adm-awi/Downloads/TestFolder"  #directory files will be saved in
depragIP = "172.10.25.100" #ip address of the deprag screwdriver

def callback_download(data):
    #copy past download part with data.iTarget as Index
    #set up headles browse
    opts = Options()
    opts.set_headless()
    fxProfile = FirefoxProfile()
    
    fxProfile.set_preference("browser.download.folderList",2)
    fxProfile.set_preference("browser.download.manager.showWhenStarting",False)
    fxProfile.set_preference("browser.download.dir",downloadDirectory)
    fxProfile.set_preference("browser.helperApps.neverAsk.saveToDisk","text/csv/ast")
    browser = Firefox(options=opts,firefox_profile=fxProfile)

    #interact with duck duck
    browser.get("http://" + depragIP + "/cgi-bin/cgiread?site=-&request=curves&args=&mode=-1-")
    
    
    dropdown = browser.find_element_by_class_name("dd-selected")
    dropdown.click()
    choice = browser.find_elements_by_class_name("dd-option")
    choice[1].click()
    downloads = browser.find_elements_by_class_name("download")
    downloads[data.iTarget].click()



    browser.close()
    return []

def downloader():
    rospy.init_node("deprag_downloader",anonymous=True)
    service_download = rospy.Service('deprag_download', download_request, callback_download)
    rospy.spin()


if __name__ == '__main__':  #apparetly good practise in python
    try:
        downloader()
    except rospy.ROSInterruptException: pass 