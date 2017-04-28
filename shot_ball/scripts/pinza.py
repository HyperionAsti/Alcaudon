#!/usr/bin/env python

import sys
import rospy
import RPi.GPIO as GPIO
from shot_ball.srv import *

#Se configura el pin de control del servomotor como una salida PWM y se le manda un ancho de pulso de 7.5% para centrarlo
GPIO.setmode(GPIO.BCM)
GPIO.setup(18,GPIO.OUT)
p = GPIO.PWM(18,50)
p.start(7.5)

#Servicio bloqueante, el nodo principal se detendrá hasta que la función termine. En función de la cadena que le envíe el nodo principal, la pinza
#se colocará en modo disparo, agarre o reposo
def service_handle (req):
	if req.estado=="R":
		p.ChangeDutyCycle(7.5)
	if req.estado=="A":
		p.ChangeDutyCycle(11)
	if req.estado=="D":
		p.ChangeDutyCycle(3.5)
	return GanchoResponse("OK")

#Función que inicia el nodo y el servicio.
def gancho_server_func():
	rospy.init_node('gancho_server')
	s = rospy.Service('gancho', Gancho, service_handle)
	rospy.spin()

if __name__ == "__main__":
	gancho_server_func()
