mode COM7 BAUD=2000000 PARITY=0 DATA=8
rem COPY COM7 CON 
rem COPY COM7 itm.log
type COM7 > itm.fifo.txt