#!/usr/bin/env python
import os
import shutil
import time

import requests as requester
import rospy
from selenium.webdriver import Firefox
from selenium.webdriver.common.by import By
from selenium.webdriver.firefox.options import Options

from deprag_downloader.msg import screwing
from deprag_downloader.srv import download_request

downloadDirectory = "/home/adm-awi/Downloads/TestFolder"

depragIP = "172.10.25.100"  # ip address of the deprag screwdriver


def find_nth(haystack, needle, n):
    start = haystack.find(needle)
    while start >= 0 and n > 1:
        start = haystack.find(needle, start + len(needle))
        n -= 1
    return start


def _get_browser_options(download_directory: str) -> Options:
    opts = Options()

    opts.add_argument("--headless")

    opts.set_preference("browser.download.folderList", 2)
    opts.set_preference("browser.download.manager.showWhenStarting", False)
    opts.set_preference("browser.download.dir", download_directory)
    opts.set_preference("browser.helperApps.neverAsk.saveToDisk", "text/csv/ast")

    return opts


def callback_download(data):  # if data is -1 publishe curve to topic
    # copy past download part with data.iTarget as Index
    # set up headles browse
    global downloadDirectory
    backupDownload = downloadDirectory
    pub = False
    if data.iTarget < 0:
        pub = True
        data.iTarget = 1
        downloadDirectory = "/tmp/deprag_downloaderERmabs4k23lknad33"
        os.mkdir(downloadDirectory)

    opts = _get_browser_options(download_directory=downloadDirectory)

    with Firefox(options=opts) as browser:
        # interact with duck duck
        browser.get(
            "http://"
            + depragIP
            + "/cgi-bin/cgiread?site=-&request=curves&args=&mode=-1-"
        )

        dropdown = browser.find_element(By.CLASS_NAME, "dd-selected")
        dropdown.click()
        choice = browser.find_elements(By.CLASS_NAME, "dd-option")
        choice[1].click()
        downloads = browser.find_elements(By.CLASS_NAME, "download")
        downloads[data.iTarget].click()

    if pub:
        _kurve_name = os.listdir(downloadDirectory)[0]
        print(_kurve_name)
        subDir = "/sub"  # sub directory to determine what program was used
        os.mkdir(downloadDirectory + subDir)

        opts = _get_browser_options(download_directory=downloadDirectory + subDir)

        with Firefox(options=opts) as browser:
            # interact with duck duck
            browser.get(
                "http://"
                + depragIP
                + "/cgi-bin/cgiread?site=-&request=fvalues&args=&mode=-1-"
            )

            downloads = browser.find_elements(By.CLASS_NAME, "download")
            downloads[len(downloads) - 2].click()

        info = open(downloadDirectory + subDir + "/actual.csv", "r")
        _info = info.read()
        info.close()
        _info = _info[20:]
        c = _info.find(",")
        _info = _info[:c]

        _prgrmNumber = int(_info)
        print(_prgrmNumber)

        # get programm info
        _str = (
            "http://"
            + depragIP
            + "/cgi-bin/cgiread?site=getprg&program="
            + str(_prgrmNumber)
        )
        r = requester.get(_str)
        prgmInfo = r.text
        print(prgmInfo)  # for debuging can be disabled

        msg = screwing()
        msg.prgrmInfo = prgmInfo
        os.rename(
            downloadDirectory + "/" + _kurve_name, downloadDirectory + "/data.txt"
        )

        with open(downloadDirectory + "/data.txt", "r") as _csv:
            _csv_str = _csv.read()
        c = _csv_str.find("Temperatur")
        c = c + 15
        _csv_str = _csv_str[c:]
        msg.csv = _csv_str
        publer.publish(msg)

        filepath = backupDownload + "/" + str(time.time()) + ".json"

        with open(filepath, "w") as _f:
            jsonInfo = prgmInfo.replace("\n", ',\n"')
            jsonInfo = jsonInfo.replace("=", '":')
            jsonInfo = '"' + jsonInfo
            c = find_nth(jsonInfo, ":", 2) + 1
            jsonInfo = jsonInfo[:c] + '"' + jsonInfo[c:]
            c = find_nth(jsonInfo, ",", 2)
            jsonInfo = jsonInfo[:c] + '"' + jsonInfo[c:]
            _f.write("{ \n")
            _f.write(jsonInfo)

            zeit = "["
            Dmess1 = "["
            Wmess1 = "["
            Dmotor = "["
            Wmotor = "["
            DZ = "["
            Schritt = "["
            Strom = "["
            Temp = "["
            lines = _csv_str.count("\n") + 1  # anzahl Zeilen
            for i in range(0, lines):
                # Zeit
                c = _csv_str.find(",")
                zeit = zeit + _csv_str[:c] + ","
                _csv_str = _csv_str[c + 1 :]
                # Mess1 Dreh
                c = _csv_str.find(",")
                Dmess1 = Dmess1 + _csv_str[:c] + ","
                _csv_str = _csv_str[c + 1 :]
                # W mess1
                c = _csv_str.find(",")
                Wmess1 = Wmess1 + _csv_str[:c] + ","
                _csv_str = _csv_str[c + 1 :]
                # D motor
                c = _csv_str.find(",")
                Dmotor = Dmotor + _csv_str[:c] + ","
                _csv_str = _csv_str[c + 1 :]
                # W Motor
                c = _csv_str.find(",")
                Wmotor = Wmotor + _csv_str[:c] + ","
                _csv_str = _csv_str[c + 1 :]
                # Drehzahl
                c = _csv_str.find(",")
                DZ = DZ + _csv_str[:c] + ","
                _csv_str = _csv_str[c + 1 :]
                # Schritt
                c = _csv_str.find(",")
                Schritt = Schritt + _csv_str[:c] + ","
                _csv_str = _csv_str[c + 1 :]
                # Strom
                c = _csv_str.find(",")
                Strom = Strom + _csv_str[:c] + ","
                _csv_str = _csv_str[c + 1 :]
                # temp
                c = _csv_str.find("\n")
                Temp = Temp + _csv_str[:c] + ","
                _csv_str = _csv_str[c + 1 :]

            zeit = zeit.replace("\n", "")

            zeit = zeit[: (len(zeit) - 2)] + "],"
            Dmess1 = Dmess1[: (len(zeit) - 2)] + "],"
            Wmess1 = Wmess1[: (len(zeit) - 2)] + "],"
            Dmotor = Dmotor[: (len(zeit) - 2)] + "],"
            Wmotor = Wmotor[: (len(zeit) - 2)] + "],"
            DZ = DZ[: (len(zeit) - 2)] + "],"
            Schritt = Schritt[: (len(zeit) - 2)] + "],"
            Strom = Strom[: (len(zeit) - 2)] + "],"
            Temp = Temp[: (len(zeit) - 2)] + "]"

            _f.write('csv" : { \n')
            t = '   "Zeit(0.001 ms)":' + zeit
            t.replace("\n", "")
            _f.write(t + "\n")
            _f.write('  "Drehmoment Messsystem 1 (Nm)":' + Dmess1 + "\n")
            _f.write('  "Winkel Messsystem 1":' + Wmess1 + "\n")
            _f.write('  "Drehmoment Motor (Nm)":' + Dmotor + "\n")
            _f.write('  "Winkel Motor":' + Wmotor + "\n")
            _f.write('  "Drehzahl (U/min)":' + DZ + "\n")
            _f.write('  "Schritt":' + Schritt + "\n")
            _f.write('  "Stromstaerke (A)":' + Strom + "\n")
            _f.write('  "Temperatur (C)":' + Temp + "\n")

            _f.write("}\n")
            _f.write("  }")

        shutil.rmtree(downloadDirectory, ignore_errors=True)

    downloadDirectory = backupDownload
    return []


publer = 0


def downloader():
    global publer
    rospy.init_node("deprag_downloader", anonymous=True)
    service_download = rospy.Service(
        "deprag_download", download_request, callback_download
    )
    publer = rospy.Publisher("deprag/curves", screwing, queue_size=10)

    rospy.spin()


if __name__ == "__main__":  # apparetly good practise in python
    try:
        downloader()
    except rospy.ROSInterruptException:
        pass
