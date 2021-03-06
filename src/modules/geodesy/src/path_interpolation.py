#!/usr/bin/env python

"""Interpolates path from previously plotted points"""

import rospy

import gps_parser
from path_interpolator_ECEF import PathInterpolatorECEF
from path_interpolator_UTM  import PathInterpolatorUTM
from path_interpolator_ENU  import PathInterpolatorENU

def main():
    rospy.init_node('interpolation_node', anonymous = True)

    chosenHeight = -6.0 # meters

    # From https://www.maps.ie/coordinates.html at SJSU
    filePath = rospy.get_param("~file_path")
    inputLatitudes, inputLongitudes, inputHeights = gps_parser.read_file_coarse_points(filePath, chosenHeight)
    
    conversionType = rospy.get_param("~conversion_type")
    cmPerPoint = float(rospy.get_param("~cm_per_point"))

    if conversionType == "ECEF":
        interpolatorECEF = PathInterpolatorECEF(inputLatitudes, inputLatitudes, inputHeights)

        try:
            rospy.loginfo("ECEF Conversion publishing..,")
            interpolatorECEF.interpolation_publish_ECEF()
        except rospy.ROSInterruptException:
            pass
    elif conversionType == "ENU":
        interpolatorENU = PathInterpolatorENU(inputLatitudes, inputLongitudes, inputHeights, centimetersPerPoint=cmPerPoint)
        
        try:
            rospy.loginfo("ENU Conversion publishing...")
            interpolatorENU.interpolation_publish_ENU()
        except rospy.ROSInterruptException:
            pass
    elif conversionType == "UTM":
        rospy.loginfo("UTM instantiation...")
        interpolatorUTM = PathInterpolatorUTM(inputLatitudes, inputLongitudes)
        
        try:
            rospy.loginfo("UTM Conversion publishing...")
            interpolatorUTM.interpolation_publish_UTM()
        except rospy.ROSInterruptException:
            pass
    else:
        raise NameError("'{}' is not a valid conversion. Please provide valid conversion in launch file\n".format(conversionType))

if __name__ == '__main__':
    main()

