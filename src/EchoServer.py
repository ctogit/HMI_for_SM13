# Echo server program
import socket
import datetime as dt
import time

# ip RPi
print("********************************")
print("***** SM-13 COM SIMULATOR ******")
print("********************************")
HOST = '192.168.0.193'                 # Symbolic name meaning all available interfaces
PORT = 5020              # Arbitrary non-privileged port

# Valores simulados desde RTU a HMI
ui_posActArm_MSB = 1
ui_posActArm_LSB = 1
ui_posActPole_MSB = 2
ui_posActPole_LSB = 2
ui_velActArm = 5
ui_velActPole = 3

s_cwLimitArm = "ACW_RUN;" 	# "ACW_LIM;"
s_ccwLimitArm = "ACC_LIM;" 	# "ACC_RUN;"
s_cwLimitPole = "PCW_RUN;" 	# "PCW_LIM;"
s_ccwLimitPole = "PCC_LIM;" # "PCC_RUN;"
s_limitDown = "LDW_RUN;" 	# "LDW_LIM;"
s_limitUp = "LUP_RUN;" 		# "LUP_LIM;"
s_stallAlm = "STL_RUN;" 	# "STL_ALM;"
s_status = 0 #42, 43, 81

# Se arma array con valores numéricos de resolver y velocidad
dataTx = [ui_posActArm_MSB,
		  ui_posActArm_LSB,
		  ui_posActPole_MSB,
		  ui_posActPole_LSB,
		  ui_velActArm,
		  ui_velActPole]

# Se arma array con strings que reporta RTU		  
stringTx = s_cwLimitArm + s_ccwLimitArm + s_cwLimitPole + s_ccwLimitPole + s_limitDown + s_limitUp + s_stallAlm + str(s_status)	  
		  
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)

conn, addr = s.accept()
print ('Connected by', addr)
sSeg = 0
while 1:	
	"""
	print (sSeg)
	sFecha, sHora = str(dt.datetime.now()).split(' ')
	sAnio, sMes, sDia = sFecha.split('-')
	sHora, sMin, sSeg = sHora.split(':')
	print (sSeg)
	"""
	dataByteTx = bytes(dataTx)
	stringByteTx = stringTx.encode()
	
	frame_to_Tx = dataByteTx + stringByteTx

	dataRx = conn.recv(1024)
	#print(sDia+"/"+sMes+"/"+sAnio, sHora+":"+sMin+":"+sSeg, "<-- HMI_to_RTU: ", dataRx)
	print("<-- HMI_to_RTU: ", dataRx)
	if not dataRx: break
	conn.sendall(frame_to_Tx)
	#print(sDia+"/"+sMes+"/"+sAnio, sHora+":"+sMin+":"+sSeg, "RTU_to_HMI -->: ", dataByteTx)
	print("RTU_to_HMI -->: ", frame_to_Tx)
	print(" ")

	# Se simulan incrementos de resolver POLE y ARM
	dataTx[1] += 5
	if dataTx[1] >= 255:
		dataTx[1] = 0
		dataTx[0] += 1
		if dataTx[0] >= 255:
			dataTx[0] = 0
			dataTx[1] = 0
	dataTx[3] += 5
	if dataTx[3] >= 255:
		dataTx[3] = 0
		dataTx[2] += 1
		if dataTx[2] >= 255:
			dataTx[2] = 0
			dataTx[3] = 0
	#time.sleep(1)
  
conn.close()