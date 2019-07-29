from __future__ import print_function
import rospy
import serial
import time


from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
from time import sleep


class GSAobject(object):
    def __init__(self, pdop, hdop, vdop):
        self.PDOP = pdop
        self.HDOP = hdop
        self.VDOP = vdop


class GGAobject(object):
    def __init__(self, time, lat, lon, alt, valid, nsat, hdop):
        self.time = time
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.valid = valid
        self.nsat = nsat
        self.hdop = hdop


def readString():
    while 1:
        while ser.read().decode("utf-8") != '$':  # Wait for the begging of the string
            pass  # Do nothing
        try:        
            line = ser.readline()
            line = line.decode("utf-8")  # Read the entire string
            return line
        except:
            print("Erro ao decodificar UTF-8")
            continue


def getTime(string, format, returnFormat):
    # Convert date and time to a nice printable format
    return time.strftime(returnFormat, time.strptime(string, format))


def fmtLatLng(latString, ns, lngString, ew):
    print("[fmtLatLng] Lat:",latString, ns, "Lon:", lngString, ew)
    lat = latString[:2].lstrip('0') + "." + "%.7s" % str(float(latString[2:])*1.0/60.0).lstrip("0.")
    lat = float(lat[:-1])
    lon = lngString[:3].lstrip('0') + "." + "%.7s" % str(float(lngString[3:])*1.0/60.0).lstrip("0.")
    lon = float(lon[:-1])
    if ns == 'S':
        lat = -lat
    if ew == 'W':
        lon = -lon
    return lat, lon


def getLatLng(latString, lngString):
    print("[getLatLng] Lat:", latString, "Lon:", lngString)
    lat = latString[:2].lstrip(
        '0') + "." + "%.7s" % str(float(latString[2:])*1.0/60.0).lstrip("0.")
    lng = lngString[:3].lstrip(
        '0') + "." + "%.7s" % str(float(lngString[3:])*1.0/60.0).lstrip("0.")
    return lat, lng


def printGGA(lines):
    print("========================================GGA========================================")
    # print(lines, '\n')
    try:
        print("Fix taken at:", getTime(lines[1], "%H%M%S.%f", "%H:%M:%S"), "UTC")
        time = lines[1]
        latlng = getLatLng(lines[2], lines[4])
        latlon = fmtLatLng(lines[2], lines[3], lines[4], lines[5])
        lat = latlon[0]
        lon = latlon[1]
        print("Lat,Long: ", latlng[0], lines[3], ", ", latlng[1], lines[5], sep='')
        print("Fix quality (0 = invalid, 1 = fix, 2..8):", lines[6])
        valid = float(lines[6])
        print("Satellites:", lines[7].lstrip("0"))
        nsat = float(lines[7])
        print("Horizontal dilution:", lines[8])
        hdop = float(lines[8])
        print("Altitude: ", lines[9], lines[10], sep="")
        alt = float(lines[9])+float(lines[11])
        print("Height of geoid: ", lines[11], lines[12], sep="")
        print("Time in seconds since last DGPS update:", lines[13])
        print("DGPS station ID number:", lines[14].partition("*")[0])
        obj = GGAobject(time, lat, lon, alt, valid, nsat, hdop)
        return obj
    except:
        print("ERRO: Satelites insuficientes para gerar dados!")
        return None

# def printGSA(lines):
#     print("========================================GSA========================================")
#     # print(lines, '\n')
#     obj = GSAobject()

#     print("Selection of 2D or 3D fix (A=Auto,M=Manual):", lines[1])
#     print("3D fix (1=No fix,2=2D fix, 3=3D fix):", lines[2])
#     print("PRNs of satellites used for fix:", end='')
#     for i in range(0, 12):
#         prn = lines[3+i].lstrip("0")
#         if prn:
#             print(" ", prn, end='')
#     print("\nPDOP", lines[15])
#     print("HDOP", lines[16])
#     print("VDOP", lines[17].partition("*")[0])
#     obj.PDOP = lines[15]
#     obj.HDOP = lines[16]
#     obj.VDOP = lines[17].partition("*")[0]
#     return obj


def checksum(line):
    checkString = line.partition("*")
    checksum = 0
    for c in checkString[0]:
        checksum ^= ord(c)

    try:  # Just to make sure
        inputChecksum = int(checkString[2].rstrip(), 16)
    except:
        print("Error in string")
        return False

    if checksum == inputChecksum:
        return True
    else:
        print("=====================================================================================")
        print("===================================Checksum error!===================================")
        print("=====================================================================================")
        print(hex(checksum), "!=", hex(inputChecksum))
        return False


if __name__ == '__main__':
    # add here the node name. In ROS, nodes are unique named.
    rospy.init_node('gps_receive')

    # publish messages to a topic using rospy.Publisher class
    pub = rospy.Publisher('gps_data', NavSatFix, queue_size=10)

    # you can define functions to provide the required functionality

    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Open Serial port

    # set a publishing rate. Below is a publishing rate of 10 times/second
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        fix = NavSatFix()
        fix.position_covariance_type = fix.COVARIANCE_TYPE_APPROXIMATED
        fix.header.stamp = rospy.Time.now()

        line = readString()
        lines = line.split(",")
        if checksum(line):
                if lines[0][2:] == "GGA":
                    ggaobj = printGGA(lines)
                    if ggaobj is None:
                        print("Empty object received.")
                        continue
                    fix.altitude = ggaobj.alt
                    fix.latitude = ggaobj.lat
                    fix.longitude = ggaobj.lon
                    fix.position_covariance[0] = ggaobj.hdop ** 2
                    fix.position_covariance[4] = ggaobj.hdop ** 2
                    fix.position_covariance[8] = (2 * ggaobj.hdop) ** 2
                    pub.publish(fix)

                    pass
                elif lines[0][2:] == "GSA":
                    # printGSA(lines)
                    pass
                elif lines[0][2:] == "GSV":
                    #     printGSV(lines)
                    pass
                elif lines[0][2:] == "RMC":
                    #     printRMC(lines)
                    pass
                elif lines[0][2:] == "GLL":
                    #     printGLL(lines)
                    pass
                elif lines[0][2:] == "VTG":
                    #     printVTG(lines)
                    pass
                else:
                    print("\n\nUnknown type:", lines[0], "\n\n")

        rate.sleep()
