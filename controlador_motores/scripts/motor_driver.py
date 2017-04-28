#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
import time
from controlador_motores.msg import Speeds
from controlador_motores.srv import *

# Pines que controlan la velocidad de los motores y el sentido de giro
_GPIO_MOTOR_LEFT=5
_GPIO_MOTOR_RIGHT=6
_GPIO_LEFT_DIR=19
_GPIO_RIGHT_DIR=13

#Definimos los pines como salida y configuramos dos de ellos como PWM con una frecuencia de 20 KHz
GPIO.setmode(GPIO.BCM)
GPIO.setup(_GPIO_MOTOR_LEFT,GPIO.OUT)
GPIO.setup(_GPIO_MOTOR_RIGHT,GPIO.OUT)
GPIO.setup(_GPIO_LEFT_DIR,GPIO.OUT)
GPIO.setup(_GPIO_RIGHT_DIR,GPIO.OUT)
_LEFT_PWM=GPIO.PWM(_GPIO_MOTOR_LEFT, 20000)
_RIGHT_PWM=GPIO.PWM(_GPIO_MOTOR_RIGHT, 20000)

_LEFT_PWM.start(0)
_RIGHT_PWM.start(0)


#Servicio bloqueante. El nodo principal se detiene hasta que no termine de ejecutarse. Ocurre si el ultrasonido frontal se acerca demasido a una pared.
def handle_service(req):
	if req.right=="right":
		GPIO.output(_GPIO_LEFT_DIR, True)
		GPIO.output(_GPIO_RIGHT_DIR, False)
		_LEFT_PWM.ChangeDutyCycle(70)
    	_RIGHT_PWM.ChangeDutyCycle(0)
		tiempo0=time.time()
		while (time.time()-tiempo0)<0.4:
    	        	pass
   	else:
		GPIO.output(_GPIO_LEFT_DIR, False)
		GPIO.output(_GPIO_RIGHT_DIR, True)
    		tiempo0=time.time()
    		_LEFT_PWM.ChangeDutyCycle(0)
		_RIGHT_PWM.ChangeDutyCycle(70)
    		while (time.time()-tiempo0)<0.4:
	    		pass
	_LEFT_PWM.ChangeDutyCycle(0)
	_RIGHT_PWM.ChangeDutyCycle(0)
	return TurnResponse("")

#Función callback, cada vez que el nodo principal le mande una nueva consigna, la velocidad y dirección de los motores se actualizará. 	
def callback(data):
	
	if data.left_speed>=-100 and data.left_speed<=100:
	    if data.left_speed>=0:
	        GPIO.output(_GPIO_LEFT_DIR, True)
	        _LEFT_PWM.ChangeDutyCycle(data.left_speed)
	    else:
	        GPIO.output(_GPIO_LEFT_DIR, False)
	        _LEFT_PWM.ChangeDutyCycle(-data.left_speed)
	else:
	    pass

	if data.right_speed>=-100 and data.right_speed<=100:
	    if data.right_speed>=0:
	        GPIO.output(_GPIO_RIGHT_DIR, True)
	        _RIGHT_PWM.ChangeDutyCycle(data.right_speed)
	    else:
	        GPIO.output(_GPIO_RIGHT_DIR, False)
	        _RIGHT_PWM.ChangeDutyCycle(-data.right_speed)
	else:
	    pass;

#Función encargada de iniciar el nodo, el tópico y el servicio, así como de esperar los callbacks
def motor_driver():

	rospy.init_node('motor_driver', anonymous=True)
	rospy.Subscriber('motor_speeds', Speeds, callback)
	s = rospy.Service('turn', Turn, handle_service)
	rospy.spin()

if __name__ == '__main__':
	motor_driver()
	GPIO.cleanup()
