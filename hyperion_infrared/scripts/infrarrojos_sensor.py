#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
from hyperion_infrared.msg import opticalinf

#Establecemos como entrada los pines que utilizaremos y le asignamos un peso a cada uno en funcion de su posicion 
_GPIO_USED=[12,16,20,21]
_GPIO_WEIGTH=[-1.5,-0.5,0.5,1.5]
GPIO.setmode(GPIO.BCM)
for i in _GPIO_USED:
	GPIO.setup(i, GPIO.IN)


def infrarrojos_sensor():
	#Iniciamos el nodo, el topico donde publicaremos el error obtenido y establecemos una frecuencia de 200 Hz
	pub = rospy.Publisher('infrarrojos_topic', opticalinf, queue_size=1)
	rospy.init_node('infrarrojos_sensor', anonymous=True)
	rate = rospy.Rate(200) 
	value=opticalinf()
    
	while not rospy.is_shutdown():
    	#Algoritmo para calcular el error en los cuatro sensores infrarrojos.
    		lectura=0.0
		encendidos=0
		for i in range (0,4):
			encendidos=encendidos+GPIO.input(_GPIO_USED[i])
        		lectura=lectura+GPIO.input(_GPIO_USED[i])*_GPIO_WEIGTH[i]
		if encendidos>0:
        		lectura=lectura/encendidos
		value.sensorinf=lectura
        	pub.publish(value)
        	rate.sleep()

if __name__ == '__main__':
	try:
        	infrarrojos_sensor()
	except rospy.ROSInterruptException:
        	pass
