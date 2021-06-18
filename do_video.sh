#!/bin/sh

find out_video/ -name '*.png' > L.txt
sort L.txt > L1.txt

#mencoder mf://@L1.txt -mf w=608:h=544:fps=20:type=png \
#	-msglevel all=6 \
#	-ovc lavc -lavcopts vcodec=mpeg4:mbd=2:trell:vbitrate=5000000 -oac copy -o DATASET_PREVIEW.avi

mencoder mf://@L1.txt -mf w=608:h=544:fps=20:type=png \
	-msglevel all=6 \
	-ovc lavc -lavcopts vcodec=flv:vbitrate=9000:mbd=2:mv0:trell:v4mv:cbp:last_pred=3 \
	-forceidx -of lavf -o $1

