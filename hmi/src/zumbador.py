##
# @file zumbador.py
#
# @brief Esta clase gesitona sonidos ácorde a los eventos que se producen
# en la HMI
#
# @return none
#
# @author Cristian Torres Barrios
# creado Mie 4 Nov 20:38:00 2020

import os

##
# @brief Función emite sonidos al detener movimientos del robot
# @param none
# @return none
def beep_stop():
	os.system('play --no-show-progress --null --channels 1 synth %s sine %f' %( 0.05, 500))
	return True

##
# @brief Función emite sonidos al presionar botones en la interfaz
# @param none
# @return none
def beep_button():
	os.system('play --no-show-progress --null --channels 1 synth %s sine %f' %( 0.05, 1000))
	return True

##
# @brief Función emite sonidos cuando un evento de alarma ocurre
# @param none
# @return none
def beep_alarm():
    os.system('play --no-show-progress --null --channels 1 synth %s sine %f' %( 0.05, 500))
    os.system('play --no-show-progress --null --channels 1 synth %s sine %f' %( 0.1, 1000))
    os.system('play --no-show-progress --null --channels 1 synth %s sine %f' %( 0.15, 1500))
    return True

##
# @brief Función emite sonidos a eventos importantes de la interfaz
# @param none
# @return none
def beep_primordial():
	os.system('play --no-show-progress --null --channels 1 synth %s sine %f' %( 0.05, 800))
	return True


