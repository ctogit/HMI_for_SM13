##
# @file depurador.py
#
# @brief Esta función permite depurar código a diferentes
# niveles de penetración del código (desde lo global
# hacia lo más particular) mediante la impresión en
# consola de diferentes mensaje de información.
#
# @param iS Severidad (nivel de test). Variable entera [0, 5] que le indica a
# esta función el nivel de penetración en el código.
# 0: depurador desactivado
# 1: severidad del depurador global
# 2: severidad media
# 3: severidad particular (ideal para test unitario)
# @param sSM Nombre del sub-módulo, función o clase llamante
# @param sTexto mensaje de información
#
# @return 0
#
# @author Cristian Torres Barrios
# creado Lun 03 Sep 23:03:00 2018

iNIVEL_TEST0 = int(0)
iNIVEL_TEST1 = int(1)
iNIVEL_TEST2 = int(2)
iNIVEL_TEST3 = int(3)
iNIVEL_TEST4 = int(4)

## Variable de depuración
iSeveridad = iNIVEL_TEST3

import datetime as dt

def depurador(iS, sSM, sTexto):
    global iSeveridad

    # consulta fecha y hora
    sFecha, sHora = str(dt.datetime.utcnow()).split(' ')
    sAnio, sMes, sDia = sFecha.split('-')
    sHora, sMin, sSeg = sHora.split(':')
    
    if iS <= iSeveridad:
        print(sDia+"/"+sMes+"/"+sAnio, sHora+":"+sMin+":"+sSeg, sSM, sTexto)
    return 0
