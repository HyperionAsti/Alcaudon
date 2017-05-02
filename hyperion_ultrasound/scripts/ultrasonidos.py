#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from hyperion_ultrasound.msg import Distances
from std_msgs.msg import Bool
import time


#Definimos los pines que vamos a utilizar configurandolos como entrada o  salida segun sea el eco o el trigger  
_GPIO_TRIGGER_IZQ=24
_GPIO_ECHO_IZQ=25
#_GPIO_TRIGGER_CENT=22
#_GPIO_ECHO_CENT=27
_GPIO_TRIGGER_DER=4
_GPIO_ECHO_DER=17
GPIO.setmode(GPIO.BCM)
GPIO.setup(_GPIO_TRIGGER_IZQ,GPIO.OUT)
GPIO.setup(_GPIO_ECHO_IZQ,GPIO.IN)
#GPIO.setup(_GPIO_TRIGGER_CENT,GPIO.OUT)
#GPIO.setup(_GPIO_ECHO_CENT,GPIO.IN)
GPIO.setup(_GPIO_TRIGGER_DER,GPIO.OUT)
GPIO.setup(_GPIO_ECHO_DER,GPIO.IN)

# La variable stop nos sirve para parar los ultrasonidos si el nodo central lo ordena. Asi evitamos un exceso de consumo
stop=0
def callback(data):
    global stop
    if data.data==1:
        stop=1
    else:
        stop=0


def ultrasonido_central():
    #Se inicia el nodo. Este publicara las distancias obtenidas al nodo central y recibira de este la senal de stop cuando sea necesario.
    #Este nodo se ejecuta a una frecuencia de 500Hz
    pub = rospy.Publisher('ultrasonido_topic', Distances, queue_size=1)
    rospy.init_node('ultrasonidos', anonymous=True)
    rospy.Subscriber("stop_topic", Bool, callback)
    rate = rospy.Rate(500) 
    value=Distances()
    global stop
    tiempo0=time.time()
    while not rospy.is_shutdown():
    
    #Algoritmo del dispositivo de ultrasonido. La distancia de un objeto sera proporcional al tiempo que permanezca encendido el pin del ECHO
   	if stop==0:
		
	        #-------ULTRASONIDO IZQUIERDO---------#
        	GPIO.output(_GPIO_TRIGGER_IZQ, True)
		time.sleep(0.00001)
        	GPIO.output(_GPIO_TRIGGER_IZQ, False)
		tiempo_inicial_izq=time.time()
		tiempo1=time.time()
        	while GPIO.input( _GPIO_ECHO_IZQ)==GPIO.LOW and (tiempo_inicial_izq-tiempo1)<0.08 :
        		tiempo_inicial_izq=time.time()
		
		tiempo_final_izq=time.time()	
        	while GPIO.input( _GPIO_ECHO_IZQ)==GPIO.HIGH:
        		tiempo_final_izq=time.time()
        
		tiempo_izq=tiempo_final_izq-tiempo_inicial_izq
        	value.distance_left=(343200*tiempo_izq)/2
        	if (value.distance_left > 500.0):
			value.distance_left=500.0		
        


        	#-------ULTRASONIDO CENTRAL---------#
        	#GPIO.output(_GPIO_TRIGGER_CENT, True)
        	#time.sleep(0.00001)
        	#GPIO.output(_GPIO_TRIGGER_CENT, False)
		#tiempo_inicial_central=time.time()
        	#tiempo1=time.time()
        	#while GPIO.input( _GPIO_ECHO_CENT)==GPIO.LOW and (tiempo_inicial_central-tiempo1)<0.08 :
            		#tiempo_inicial_central=time.time()

		#tiempo_final_central=time.time()
       		#while GPIO.input( _GPIO_ECHO_CENT)==GPIO.HIGH:
           		#tiempo_final_central=time.time()

        	#tiempo_central=tiempo_final_central-tiempo_inicial_central
        	#value.distance_front=(343200*tiempo_central)/2
		

        	#-------ULTRASONIDO DERECHO---------#
        	GPIO.output(_GPIO_TRIGGER_DER, True)
		time.sleep(0.00001)
        	GPIO.output(_GPIO_TRIGGER_DER, False)		
        	tiempo_inicial_der=time.time()
		tiempo1=time.time()
        	while GPIO.input( _GPIO_ECHO_DER)==GPIO.LOW and (tiempo_inicial_der-tiempo1)<0.08:
        		tiempo_inicial_der=time.time()
		
        	tiempo_final_der=time.time()
        	while GPIO.input( _GPIO_ECHO_DER)==GPIO.HIGH:
        		tiempo_final_der=time.time()
       
        	tiempo_der=tiempo_final_der-tiempo_inicial_der
        	value.distance_right=(343200*tiempo_der)/2
		if (value.distance_right > 500.0):
            		value.distance_right=500.0 
        	
    #Si la senal de stop esta habilitada, no se hace nada
	else:
		pass

    	pub.publish(value)
    	rate.sleep()

if __name__ == '__main__':
    try: 	
        ultrasonido_central()
	GPIO.cleanup() 
    except rospy.ROSInterruptException:
        pass

