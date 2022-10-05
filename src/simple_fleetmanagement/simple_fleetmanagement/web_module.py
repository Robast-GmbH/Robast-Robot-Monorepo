from urllib import request
import requests
import time
from multiprocessing import get_logger


def checkConnection(url):
    noConnection = True
    errorSent = False

    while noConnection:
        try:
            _ = requests.get(url)
            noConnection = False
        except requests.exceptions.RequestException:
            if not errorSent:
                get_logger().info("connection to " + url + " failed.")
                errorSent = True
            time.sleep(1)  # if fixed in use wait until

    if errorSent:
        get_logger().info("connection established.")
    return url


def getDataFromServer(url):
    checkConnection(url)
    response = requests.get(url)
    if(response.status_code != 200):
        get_logger().warning('Response code from api_url ' + str(url) + ' is ' + str(response.status_code))
        return None
    else:
        return response


def setDataOnServer(url, data):
    checkConnection(url)
    response = requests.put(url, json=data)
    if(response.status_code != 200):
        get_logger().warning('Response code from api_url ' + str(url) + ' is ' + str(response.status_code))
        return None
    else:
        return response

def deleteDataOnServer(url, data):
    checkConnection(url)
    if(data== None):
        response = requests.delete(url)
    else:
        response = requests.delete(url, json=data)
    if(response.status_code != 200):
        get_logger().warning('Response code from api_url ' + str(url) + ' is ' + str(response.status_code))
        return None
    else:
        return response