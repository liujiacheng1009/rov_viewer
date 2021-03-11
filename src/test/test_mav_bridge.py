import sys 
sys.path.append("/home/bluerov/Downloads/rov_viewer/src/bluerov")

from mavlink_reader import MAVBridge
import cv2 

if __name__ == '__main__':
    bridge = MAVBridge(device='udp:192.168.2.1:14560')
    #i=0
    #filemav = open("mavlinkdata.txt", 'w')
    while True:
        bridge.update()
        bridge.print_data()
        #filemav.write("{}\n".format(bridge.data))
        #bridge.set_servo_pwm(9,1800)
        #i+=1
    #filemav.close()
        


#        if 'SCALED_PRESSURE' not in bridge.get_data():
#            print('NO PRESSURE DATA')


#        else :
#            bar30_data = bridge.get_data()['SCALED_PRESSURE']
#            print("bar30data : ",bar30_data)
#            time_boot_ms = bar30_data['time_boot_ms']
#            press_abs    = bar30_data['press_abs']
#            press_diff   = bar30_data['press_diff']
#            temperature  = bar30_data['temperature']
#            print("\n\n\n")
#            print( "time :",time_boot_ms,"press_abs :", press_abs, "press_diff :",press_diff, "temperature :", temperature)
