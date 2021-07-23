#!/usr/bin/env python
import rospy
from deprag_downloader.srv import download_request
from selenium.webdriver import Firefox, FirefoxProfile
from selenium.webdriver.firefox.options import Options
from selenium.webdriver.support.select import Select
import requests as requester
import os
from deprag_downloader.msg import screwing
import csv
import shutil

downloadDirectory = "/home/adm-awi/Downloads/TestFolder"

depragIP = "172.10.25.100" #ip address of the deprag screwdriver

def callback_download(data): #if data is -1 publishe curve to topic
    #copy past download part with data.iTarget as Index
    #set up headles browse
    global downloadDirectory
    backupDownload = downloadDirectory
    pub = False
    if(data.iTarget == -1):
        pub = True
        data.iTarget = 1 
        downloadDirectory = "/tmp/deprag_downloaderERmabs4k23lknad33"
        os.mkdir(downloadDirectory)
    

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

    if(pub):
        _kurve_name = os.listdir(downloadDirectory)[0]
        print(_kurve_name)
        subDir = "/sub" #sub directory to determine what program was used
        os.mkdir(downloadDirectory + subDir) 


        opts = Options()
        opts.set_headless()
        fxProfile = FirefoxProfile()
    
        fxProfile.set_preference("browser.download.folderList",2)
        fxProfile.set_preference("browser.download.manager.showWhenStarting",False)
        fxProfile.set_preference("browser.download.dir",downloadDirectory + subDir)
        fxProfile.set_preference("browser.helperApps.neverAsk.saveToDisk","text/csv/ast")
        browser = Firefox(options=opts,firefox_profile=fxProfile)

        #interact with duck duck
        browser.get("http://" + depragIP + "/cgi-bin/cgiread?site=-&request=fvalues&args=&mode=-1-")
        
        
        
        downloads = browser.find_elements_by_class_name("download")
        downloads[ len(downloads) - 2].click()
        
        browser.close()

        info = open(downloadDirectory + subDir + "/actual.csv","r")
        _info = info.read()
        info.close()
        _info = _info[20:]
        c = _info.find(",")
        _info = _info[:c]
        
        _prgrmNumber = int(_info)
        print(_prgrmNumber)

        #get programm info
        _str = "http://"+ depragIP +"/cgi-bin/cgiread?site=getprg&program=" + str(_prgrmNumber)
        r = requester.get(_str)
        prgmInfo = r.text
        print(prgmInfo)  #for debuging can be disabled

        msg = screwing()
        msg.prgrmInfo = prgmInfo
        os.rename(downloadDirectory + "/" + _kurve_name,downloadDirectory + "/data.txt")

        _csv = open(downloadDirectory + "/data.txt","r")
        _csv_str =  _csv.read()
        c = _csv_str.find("Temperatur")
        c = c+15
        _csv_str = _csv_str[c:]
        print(_csv_str)
        msg.csv = _csv_str
        _csv.close()
        publer.publish(msg)

        shutil.rmtree(downloadDirectory, ignore_errors=True)

        
    
    


    downloadDirectory = backupDownload
    return []

publer = 0

def downloader():
    global publer
    rospy.init_node("deprag_downloader",anonymous=True)
    service_download = rospy.Service('deprag_download', download_request, callback_download)
    publer = rospy.Publisher('deprag/curves', screwing, queue_size=10)
   
    

    rospy.spin()


if __name__ == '__main__':  #apparetly good practise in python
    try:
        downloader()
    except rospy.ROSInterruptException: pass 