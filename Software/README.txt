The first two folders are for the AtMega328 processor on the Raspberry Pi shield
This is the processor that is on the shield designed in the eagle files.

Created by Dylan Trafford
Edited and modified by George Milheim and Skylar Tamke

The python file is just to communicate to the processor on the board.  It sends commands once to the AtMega328.  Can control servos and direction to point with 4 arguments to the code.  

run like
sudo python octaplexer.py [bearing 0<-north 4<-east...] [tilt of cameras] [0<- for pass on check] [0<- for pass on check]

ex: sudo python octaplexer.py 7 7 0 0 
