#!/usr/bin/env python



import pygtk                           #import the graphics tool kit package
pygtk.require('2.0')                   # version of the tool kit
import gtk                      #import the tool kit
import serial                   #import package for serial port communication
import threading                #import threads form threading package
from  threading import Thread     
import gobject          
gtk.gdk.threads_init()           #intialize all threads
import time                      # import time package
import math                  #import math package to use math functions (here we used log base 10)
import pango                  #for font options(style, size, )
from datetime import datetime
import datetime                #for font options(style, size, )
import struct
#from datetime import datetime
#utc_datetime = datetime.datetime.utcnow() # for universal time cordinate

#s=utc_datetime.strftime("%X %x" )#% (now.minute, now.second, now.microsecond))
s=time.strftime("%X %x %Z" ) #% (now.minute, now.second, now.microsecond))

class HAPS:
    def __init__(self):
			self._portname = '/dev/ttyMXUSB1'
			self._baudRate = 115200
			self._paritybits = serial.PARITY_NONE
			self._stopbits = serial.STOPBITS_TWO
			self._serByteSize = serial.EIGHTBITS
			self._timeout = 0.0
			self.ser1 = None
			self.serport = None
			self.gpsfile = None
			self.hapfile = None
			self.flag = False
			self.curTime = time.strftime("%X %x %Z" )
			self.startHapTime = datetime.datetime.now()
			self.endHapTime = datetime.datetime.now()
			self.headerAdded = 0x0000;
			self.checksumerrcnt = 0
			self.gpsHeaderRead = False
			
			self.lat_actual_val = 0
			self.long_actual_val = 0
			self.alt_actual_val = 0
			self.posx_actual_val = 0
			self.posy_actual_val = 0
			self.posz_actual_val = 0
			self.velx_actual_val = 0
			self.vely_actual_val = 0
			self.velz_actual_val = 0
			
			self.gps_time = 0
			
			#Haps Data Storing Variable
			self.rangeFreq = 0
			self.msgcount = 0
			self.msgid = 0
			self.snrVal_up = 0
			self.snrVal_down = 0
			self.fup=0
			self.fdown=0 
			self.dopplerFreq = 0
			self.systimedisp = 0
			self.time_elapsed = 0.0
			self.gps_time_elapsed = 0.0
			self.gps_endtime = None
			self.gps_datacnt = 0
			self.len = 0
			self.reserved = 0
			self.reservedbyte1 = 0
			self.reservedbyte2 = 0
			self.reservedbyte3 = 0
			self.reservedbyte4 = 0
			self.reservedbyte5 = 0
			self.reservedbyte6 = 0
			self.reservedbyte7 = 0
			self.reservedbyte8 = 0
			self.reservedbyte9 = 0
			self.footerval = 0
			
			self.gps_data_available = False
			self.hap_data_available = False
			self.stopDisp = False
			self.hapDataCount = 0
			
			self.windowA = gtk.Window(gtk.WINDOW_TOPLEVEL)
			color = gtk.gdk.color_parse('black')
			self.windowA.modify_bg(gtk.STATE_NORMAL, color)
			self.windowA.set_title("GPS Monitoring Starting .....")
			self.windowA.resize(gtk.gdk.screen_width(), gtk.gdk.screen_height())
			self.fixed2 = gtk.Fixed()
			self.windowA.add(self.fixed2)
			
			self.b=gtk.Button("Click here to start")
			self.b.modify_font(pango.FontDescription("sans 40"))
			self.b.set_size_request(200,100)
			self.b.connect("clicked",self.dcall)
			self.fixed2.put(self.b,380,350)
			self.b.show()
			
			self.l1=gtk.Label("WELCOME   TO   HAPS   and   GPS   GUI")
			self.l1.modify_font(pango.FontDescription("sans 40"))
			self.fixed2.put(self.l1,35,100)
			self.l1.show()
			self.l=gtk.Label("******************************************************************************************************************************")
			self.fixed2.put(self.l,50,160)
			self.l.show()
			self.fixed2.show()
			self.windowA.show()
			#window.add(self.fixed)                #add it to the window
			self.fixed  =   gtk.Fixed()
			self.fixed1 =   gtk.Fixed()
			self.fixed5  =   gtk.Fixed()
			self.tlat1  =   gtk.TextView()
			self.tlong1 =   gtk.TextView()
		
    def initSerialPort(self):
        self.serport = serial.Serial(port=self._portname,baudrate=self._baudRate,parity=self._paritybits,stopbits=self._stopbits,bytesize=self._serByteSize,timeout=self._timeout)
        # if not self.serport.isOpen():
            # self.serport.open()
            # self.serport.flushInput()
            # self.serport.flushOutput()
        if not self.serport.isOpen():
            print "Unable to Open Serial Port for GPS"
            exit(1)
        else:
            # self.serport.flushInput()
            # self.serport.flushOutput()
            print "GPS Port opened for reading"
    
    def closeSerialPort(self):
        self.serport.close()
        if not self.serport.close():
            print "Unable to close the Serial Port for GPS"
            exit(1)
    
    def readHeader(self):
        #self.initSerialPort()
        validHeadear = False
        while not validHeadear:
            while True:
				if self.serport.isOpen():
					data = self.serport.read(2)
					if len(data)==2:
						headerval =  data[1].encode('hex')+ data[0].encode('hex')
						if headerval == "bc02":
							self.gps_datacnt =self.gps_datacnt + 1
							self.gps_endtime = datetime.datetime.now()
							diff = self.gps_endtime - self.startHapTime
							self.gps_time_elapsed = diff.days*24*3600 + diff.seconds + diff.microseconds*1e-6
							
							print self.gps_datacnt
							print headerval
							validHeadear = True
							self.gpsHeaderRead = True
							break
    
    def readSerportData(self):
			self.initSerialPort()
			
			# if self.gpsHeaderRead :																							##  GPS file constraint
			
			self.gpsfile = open("gpsfile" + self.curTime + ".txt","w")
			self.gpsfile.write("%LAT\t\tLong\t\tALT\t\t\tVX\t\t\tVY\t\t\tVZ\t\t\tTime_Elapsed\n\n")
			
			a=0
			while self.flag:
				a=0
				self.readHeader()
				while a!=64:
						data = self.serport.read(2)
						a += len(data)
				a=0
				while a!=2:
					soltime1 = self.serport.read(2)
					a += len(soltime1)
				a=0
				while a!=2:
					soltime2 = self.serport.read(2)
					a += len(soltime2)
				a=0
				while a!=12:
					data = self.serport.read(2)
					a += len(data)
				a=0
				while a!=2:
					posdatax1 = self.serport.read(2)
					a += len(posdatax1)
				a=0
				while a!=2:
					posdatax2 = self.serport.read(2)
					a += len(posdatax2)
				a=0
				while a!=2:
					posdatay1 = self.serport.read(2)
					a += len(posdatay1)
				a=0
				while a!=2:
					posdatay2 = self.serport.read(2)
					a += len(posdatay2)
				a=0
				while a!=2:
					posdataz1 = self.serport.read(2)
					a += len(posdataz1)
				a=0
				while a!=2:
					posdataz2 = self.serport.read(2)
					a += len(posdataz2)
				a=0
				while a!=2:
					veldatax1 = self.serport.read(2)
					a += len(veldatax1)
				a=0
				while a!=2:
					veldatax2 = self.serport.read(2)
					a += len(veldatax2)
				a=0
				while a!=2:
					veldatay1 = self.serport.read(2)
					a += len(veldatay1)
				a=0
				while a!=2:
					veldatay2 = self.serport.read(2)
					a += len(veldatay2)
				a=0
				while a!=2:
					veldataz1 = self.serport.read(2)
					a += len(veldataz1)
				a=0
				while a!=2:
					veldataz2 = self.serport.read(2)
					a += len(veldataz2)
				a=0
				while a!=2:
					latdata1 = self.serport.read(2)
					a+=len(latdata1)
				a=0
				while a!=2:
					latdata2 = self.serport.read(2)
					a+=len(latdata2)
				a=0
				while a!=2:
					longdata1 = self.serport.read(2)
					a+=len(longdata1)
				a=0
				while a!=2:
					longdata2 = self.serport.read(2)
					a+=len(longdata2)
				a=0
				while a!=2:
					altdata1 = self.serport.read(2)
					a+=len(altdata1)
				a=0
				while a!=2:
					altdata2 = self.serport.read(2)
					a+=len(altdata2)
				
				lat_str = latdata2[1].encode('hex')+latdata2[0].encode('hex')+latdata1[1].encode('hex')+latdata1[0].encode('hex')
				lat_int_val = int(lat_str,16)
				self.lat_actual_val = lat_int_val*57.25e-9
				soltime_str = soltime2[1].encode('hex')+soltime2[0].encode('hex')+soltime1[1].encode('hex')+soltime1[0].encode('hex')
				soltime_int_val = int(soltime_str,16)
				self.gps_time = soltime_int_val
				long_str = longdata2[1].encode('hex')+longdata2[0].encode('hex')+longdata1[1].encode('hex')+longdata1[0].encode('hex')
				long_int_val = int(long_str,16)
				self.long_actual_val = long_int_val*57.25e-9
				alt_str = altdata2[1].encode('hex')+altdata2[0].encode('hex')+altdata1[1].encode('hex')+altdata1[0].encode('hex')
				alt_int_val = int(alt_str,16)
				self.alt_actual_val = alt_int_val*1e-2 -300
				posx_str = posdatax2[1].encode('hex')+posdatax2[0].encode('hex')+posdatax1[1].encode('hex')+posdatax1[0].encode('hex')
				posx_int_val = int(posx_str,16)
				self.posx_actual_val = posx_int_val*1e-2
				posy_str = posdatay2[1].encode('hex')+posdatay2[0].encode('hex')+posdatay1[1].encode('hex')+posdatay1[0].encode('hex')
				posy_int_val = int(posy_str,16)
				self.posy_actual_val = posy_int_val*1e-2
				posz_str = posdataz2[1].encode('hex')+posdataz2[0].encode('hex')+posdataz1[1].encode('hex')+posdataz1[0].encode('hex')
				posz_int_val = int(posz_str,16)
				self.posz_actual_val = posz_int_val*1e-2
				velx_str = veldatax2[1].encode('hex')+veldatax2[0].encode('hex')+veldatax1[1].encode('hex')+veldatax1[0].encode('hex')
				velx_int_val = struct.unpack('!f',velx_str.decode('hex'))[0]
				self.velx_actual_val = velx_int_val
				vely_str = veldatay2[1].encode('hex')+veldatay2[0].encode('hex')+veldatax1[1].encode('hex')+veldatax1[0].encode('hex')
				vely_int_val = struct.unpack('!f',vely_str.decode('hex'))[0]
				self.vely_actual_val = vely_int_val
				velz_str = veldataz2[1].encode('hex')+veldataz2[0].encode('hex')+veldataz1[1].encode('hex')+veldataz1[0].encode('hex')
				velz_int_val = struct.unpack('!f',velz_str.decode('hex'))[0]
				self.velz_actual_val = velz_int_val
				#print self.velx_actual_val
				
				
				if self.gpsHeaderRead :
					self.gpsHeaderRead = False
					self.gpsfile.write("%.6f\t%.6f\t%.6f\t"%(self.lat_actual_val,self.long_actual_val,self.alt_actual_val))
					self.gpsfile.write("%.6f\t"%(self.velx_actual_val))
					self.gpsfile.write("%.6f\t"%(self.vely_actual_val))
					self.gpsfile.write("%.6f\t"%(self.velz_actual_val))
					self.gpsfile.write("%d\t"%(self.gps_time))
					self.gpsfile.write("%.6f\n"%(self.gps_time_elapsed))
					self.gps_data_available = True
					
					
				#self.gpsfile.write("%0.6f\t\t0.6f\t\t0.6f\n"%(self.velx_actual_val,self.vely_actual_val,self.velz_actual_val))
				#self.gpsfile.write("%.6f\t%.6f\t%.6f\t.6f\n"%(self.velx_actual_val,self.vely_actual_val,self.velz_actual_val,self.gps_time_elapsed))
				#self.serport.flushInput()
				#self.serport.flushOutput()
				

			
			
			
			
    def etc_click(self, wid):        
        self.windowB = gtk.Window(gtk.WINDOW_TOPLEVEL)
        color = gtk.gdk.color_parse('grey')
        self.windowB.modify_bg(gtk.STATE_NORMAL, color)
        self.windowB.set_title("GPS GUI.....")
        self.windowB.resize(1300,550)
        # Create a Fixed Container
        self.fixed1 = gtk.Fixed()
        self.windowB.add(self.fixed1)
        self.fixed1.show()
        self.l=gtk.Label("LATITUDE(Deg)")
        self.l.show()
        self.fixed1.put(self.l,60,55)
        self.l=gtk.Label("LONGITUDE(Deg)")
        self.l.show()
        self.fixed1.put(self.l,300,55)
        #1. latitude1 GPS
        #self.tlat1=gtk.TextView()                
        self.tlat1.set_size_request(100,100)        
        self.tlat1.show()
        #buffer for range view object
        #add buffer to object
    
        #1. latitude1        
        #self.tblat=gtk.TextBuffer()            
        #self.tlat1.set_buffer(self.tblat)
        #now add text views
        self.slat1=gtk.ScrolledWindow()
        self.slat1.set_size_request(200,250)
        self.slat1.add(self.tlat1)
        self.slat1.show()
        #fix the position of the scroll windows
        self.fixed1.put(self.slat1,40,80)

    
        #1. longitude1 GPS
        #self.tlong1=gtk.TextView()                
        self.tlong1.set_size_request(100,100)        
        self.tlong1.show()
        #buffer for range view object
        #add buffer to object

        #1. longitude1        
        #self.tblong=gtk.TextBuffer()            
        #self.tlong1.set_buffer(self.tblong)
        #now add text views
        self.slong1=gtk.ScrolledWindow()
        self.slong1.set_size_request(200,250)
        self.slong1.add(self.tlong1)
        self.slong1.show()
        #fix the position of the scroll windows
        self.fixed1.put(self.slong1,300,80)
        #self.fixed1 = gtk.Fixed()
        #self.windowB.add(self.fixed1)

        window1 = gtk.Window(gtk.WINDOW_TOPLEVEL)
        color = gtk.gdk.color_parse('#bf3fff')         #define the color
        window1.modify_bg(gtk.STATE_NORMAL, color)     #modify the background
        window1.set_title("GPS HAPS GUI.....")
        self.fixed.show()     
        self.windowB.show() 

    def hide_windowB(windowB,event):        #hiding window
        windowB.hide()
        return True

        windowB.connect('destroy',hide_windowB)

    def dcall (self,wid):
			#1. latitude1        
			self.tblat=gtk.TextBuffer()            
			self.tlat1.set_buffer(self.tblat)
		
			#1. longitude1        
			self.tblong=gtk.TextBuffer()            
			self.tlong1.set_buffer(self.tblong)
			
			################################################################
			self.bufGPS=[]
			self.bufHAPS=[]        
			# Create a new window
			window = gtk.Window(gtk.WINDOW_TOPLEVEL)
			color = gtk.gdk.color_parse('yellow')
			window.modify_bg(gtk.STATE_NORMAL, color)
			window.set_title("GPS HAPS GUI.....")
			window.resize(gtk.gdk.screen_width(),gtk.gdk.screen_height())
			window.add(self.fixed)         
			self.fixed.show()
			
			
			#3. altitude GPS
			self.taltgps=gtk.TextView()
			self.taltgps.set_editable(False)
			self.taltgps.modify_base(gtk.STATE_NORMAL,gtk.gdk.color_parse('black')) 
			self.taltgps.modify_text(gtk.STATE_NORMAL,gtk.gdk.color_parse('yellow'))			
			self.taltgps.set_size_request(100,100)
			self.taltgps.show()
			
			#3. altitude HAPS 
			self.talt=gtk.TextView()
			self.talt.set_editable(False)
			self.talt.set_size_request(100,100)
			self.talt.modify_base(gtk.STATE_NORMAL,gtk.gdk.color_parse('black')) 
			self.talt.modify_text(gtk.STATE_NORMAL,gtk.gdk.color_parse('green'))
			self.talt.show()

			#4. range frequency HAPS
			self.trn=gtk.TextView()
			self.trn.set_editable(False)          
			self.trn.set_size_request(100,100) 
			self.trn.modify_base(gtk.STATE_NORMAL,gtk.gdk.color_parse('black')) 
			self.trn.modify_text(gtk.STATE_NORMAL,gtk.gdk.color_parse('green'))
			self.trn.show()

			#5. doppler HAPS
			self.tdop=gtk.TextView()
			self.tdop.set_editable(False)
			self.tdop.set_size_request(100,100)
			self.tdop.modify_base(gtk.STATE_NORMAL,gtk.gdk.color_parse('black')) 
			self.tdop.modify_text(gtk.STATE_NORMAL,gtk.gdk.color_parse('green'))
			self.tdop.show()

			
		#6. SNR_up    HAPS
			self.tsnr_up=gtk.TextView()
			self.tsnr_up.set_editable(False)
			self.tsnr_up.set_size_request(100,100)
			self.tsnr_up.modify_base(gtk.STATE_NORMAL,gtk.gdk.color_parse('black')) 
			self.tsnr_up.modify_text(gtk.STATE_NORMAL,gtk.gdk.color_parse('yellow'))
			self.tsnr_up.show()

		#6. SNR_down    HAPS
			self.tsnr_down=gtk.TextView()
			self.tsnr_down.set_editable(False)
			self.tsnr_down.set_size_request(100,100)
			self.tsnr_down.modify_base(gtk.STATE_NORMAL,gtk.gdk.color_parse('black')) 
			self.tsnr_down.modify_text(gtk.STATE_NORMAL,gtk.gdk.color_parse('green'))
			self.tsnr_down.show()
		#---------------------------------------------##--------------------##------------------------------------------------------#
		# 7. f_down    HAPS
			self.tf_down=gtk.TextView()
			self.tf_down.set_editable(False)
			self.tf_down.set_size_request(100,100)
			self.tf_down.modify_base(gtk.STATE_NORMAL,gtk.gdk.color_parse('black')) 
			self.tf_down.modify_text(gtk.STATE_NORMAL,gtk.gdk.color_parse('green'))
			self.tf_down.show()

		# #8. f_up    HAPS
			self.tf_up=gtk.TextView()
			self.tf_up.set_editable(False)
			self.tf_up.set_size_request(100,100)
			self.tf_up.modify_base(gtk.STATE_NORMAL,gtk.gdk.color_parse('black')) 
			self.tf_up.modify_text(gtk.STATE_NORMAL,gtk.gdk.color_parse('yellow'))
			self.tf_up.show()

		# #8. HOB OUTPUT SIGNAL    HAPS
			self.hob_op=gtk.TextView()
			self.hob_op.set_editable(False)
			self.hob_op.set_size_request(100,100)
			self.hob_op.modify_base(gtk.STATE_NORMAL,gtk.gdk.color_parse('black')) 
			self.hob_op.modify_text(gtk.STATE_NORMAL,gtk.gdk.color_parse('yellow'))
			self.hob_op.show()

		


			#9. TIME SYSTEM    
			self.ttime=gtk.TextView()
			self.ttime.set_editable(False)
			self.ttime.set_size_request(100,100)
			self.ttime.modify_base(gtk.STATE_NORMAL,gtk.gdk.color_parse('black')) 
			self.ttime.modify_text(gtk.STATE_NORMAL,gtk.gdk.color_parse('white'))
			self.ttime.show()

			#8. Reference ALTITUDE editable user input
			self.tref=gtk.TextView()           
			self.tref.set_size_request(100,100) 
			self.tref.modify_base(gtk.STATE_NORMAL,gtk.gdk.color_parse('black')) 
			self.tref.modify_text(gtk.STATE_NORMAL,gtk.gdk.color_parse('green'))
			self.tref.show()
			self.tref.get_text=0

			#buffer for range view object
			#add buffer to object

			#3. altitude        
			self.tbaltgps=gtk.TextBuffer()
			self.taltgps.set_buffer(self.tbaltgps)
			
			#3. altitude        
			self.tbalt=gtk.TextBuffer()
			self.talt.set_buffer(self.tbalt)

			#4. range        
			self.tbrn=gtk.TextBuffer()            
			self.trn.set_buffer(self.tbrn)
		#self.trn.modify_text(gtk.STATE_NORMAL,gtk.gdk.color_parse('green'))
							
			#5. doppler        
			self.tbdop=gtk.TextBuffer()
			self.tdop.set_buffer(self.tbdop)

			#6. SNR _up       
			self.tbsnr_up=gtk.TextBuffer()
			self.tsnr_up.set_buffer(self.tbsnr_up)

		#7. SNR _down       
			self.tbsnr_down=gtk.TextBuffer()
			self.tsnr_down.set_buffer(self.tbsnr_down)


		# #6. f_up        
			self.tbf_up=gtk.TextBuffer()
			self.tf_up.set_buffer(self.tbf_up)

			# #6. f_down       
			self.tbf_down=gtk.TextBuffer()
			self.tf_down.set_buffer(self.tbf_down)

			# #8. HOB OUTPUT SIGNAL       
			self.hob_op_buff=gtk.TextBuffer()
			self.hob_op.set_buffer(self.hob_op_buff)			
			
			#9. time        
			self.tbtime=gtk.TextBuffer()
			self.ttime.set_buffer(self.tbtime)

			#8. Reference        
			#self.tbref=gtk.TextBuffer()
			#self.tref.set_buffer(self.tbref)
			#p = self.tref.get_buffer()
			#create scroll containers for textviews 

			self.srn=gtk.ScrolledWindow()
			self.sdop=gtk.ScrolledWindow()
			self.salt=gtk.ScrolledWindow()
			self.saltgps=gtk.ScrolledWindow()
			self.ssnr_up=gtk.ScrolledWindow()
			self.ssnr_down=gtk.ScrolledWindow()
			#----------------------------#=----------------#---------------------------------#
			self.sf_up=gtk.ScrolledWindow()
			self.sf_down=gtk.ScrolledWindow()
			self.stime=gtk.ScrolledWindow()
			self.shob_op=gtk.ScrolledWindow()
			#set size

			self.srn.set_size_request(110,400)
			self.sdop.set_size_request(110,400)
			self.salt.set_size_request(110,400)        
			self.saltgps.set_size_request(110,400)
            
			self.ssnr_down.set_size_request(110,400)
			self.ssnr_up.set_size_request(110,400)
			#-----------------------------#---------------------#--------------------------------#
			self.sf_up.set_size_request(110,400)
			self.sf_down.set_size_request(110,400)
			self.shob_op.set_size_request(110,400)
			
			self.stime.set_size_request(110,400)
			#self.sref.set_size_request(150,50)
		
		#now add text views
			self.srn.add(self.trn)
			self.sdop.add(self.tdop)
			self.salt.add(self.talt)
			self.saltgps.add(self.taltgps)
			self.ssnr_up.add(self.tsnr_up)
			self.ssnr_down.add(self.tsnr_down)
			#-----------------------------------------#-----------------#---------------------------#
			self.sf_up.add(self.tf_up)
			self.sf_down.add(self.tf_down)
			self.shob_op.add(self.hob_op)
			self.stime.add(self.ttime)
		   # self.sref.add(self.tref)
			
			
			#make visibility true

			self.srn.show()
			self.sdop.show()
			self.salt.show()
			self.saltgps.show()		    
			self.ssnr_up.show()
			self.ssnr_down.show()
			#--------------------------------------------#----------------------#--------------------------#
			self.sf_down.show()
			self.sf_up.show()
			self.shob_op.show()
			self.stime.show()
		   # self.sref.show()
			
			#fix the position of the scroll windows
			self.fixed.put(self.srn,10,110)
			self.fixed.put(self.sdop,130,110)
			self.fixed.put(self.salt,250,110)			#250
			self.fixed.put(self.stime,490,110)		#370
			self.fixed.put(self.ssnr_up,610,110)        
			self.fixed.put(self.ssnr_down,730,110)
			self.fixed.put(self.saltgps,370,110)
			#-------------------------------------------------#-----------------------#---------------------------#
			self.fixed.put(self.sf_up,490,520)        
			self.fixed.put(self.sf_down,610,520)        
			self.fixed.put(self.shob_op,850,110)        
				
		   # self.fixed.put(self.sref,60,640)
			


			
#######     all status displays      ###########################	###############################



# ##11111111111111####1s11################   TX ON/OFF DISPLAY    ##########################
			
			# self.txonstatus_buffer = gtk.TextBuffer()
			# self.txonstatus=gtk.TextView()
			# self.txonstatus.set_buffer(self.txonstatus_buffer)
			# self.txonstatus.set_editable(False)
			# self.txonstatus.set_size_request(210,20)
			# self.txonstatus.modify_base(gtk.STATE_NORMAL,gtk.gdk.color_parse('white')) 
			# self.txonstatus.modify_text(gtk.STATE_NORMAL,gtk.gdk.color_parse('black'))
 			# self.txonstatus.modify_bg(gtk.STATE_NORMAL,gtk.gdk.color_parse('white'))						
			
			# self.fixed.put(self.txonstatus,640,585)			
			# self.txonstatus.show()


# ##11111111111111	####111####  BANDWIDTH STATUS    ##########################		

			# self.bandwidthstatus_buffer = gtk.TextBuffer()
			# self.bandwidthstatus=gtk.TextView()
			# self.bandwidthstatus.set_buffer(self.bandwidthstatus_buffer)
			# self.bandwidthstatus.set_editable(False)
			# self.bandwidthstatus.set_size_request(210,20)
			# self.bandwidthstatus.modify_base(gtk.STATE_NORMAL,gtk.gdk.color_parse('white')) 
			# self.bandwidthstatus.modify_text(gtk.STATE_NORMAL,gtk.gdk.color_parse('black'))
 			# self.bandwidthstatus.modify_bg(gtk.STATE_NORMAL,gtk.gdk.color_parse('white'))						
			
			# self.fixed.put(self.bandwidthstatus,640,610)			
			# self.bandwidthstatus.show()
			

# ####11111111111111################   WAVEFORM PERIOD    ##########################


			# self.waveformperiod_buffer = gtk.TextBuffer()
			# self.waveformperiod=gtk.TextView()
			# self.waveformperiod.set_buffer(self.waveformperiod_buffer)
			# self.waveformperiod.set_editable(False)
			# self.waveformperiod.set_size_request(210,20)
			# self.waveformperiod.modify_base(gtk.STATE_NORMAL,gtk.gdk.color_parse('white')) 
			# self.waveformperiod.modify_text(gtk.STATE_NORMAL,gtk.gdk.color_parse('black'))
 			# self.waveformperiod.modify_bg(gtk.STATE_NORMAL,gtk.gdk.color_parse('white'))						
			
			# self.fixed.put(self.waveformperiod,640,635)			
			# self.waveformperiod.show()
# ####11111111111111################   HEALTH STATUS  ##########################


			# self.healthstatus_buffer = gtk.TextBuffer()
			# self.healthstatus=gtk.TextView()
			# self.healthstatus.set_buffer(self.healthstatus_buffer)
			# self.healthstatus.set_editable(False)
			# self.healthstatus.set_size_request(210,20)
			# self.healthstatus.modify_base(gtk.STATE_NORMAL,gtk.gdk.color_parse('white')) 
			# self.healthstatus.modify_text(gtk.STATE_NORMAL,gtk.gdk.color_parse('black'))
 			# self.healthstatus.modify_bg(gtk.STATE_NORMAL,gtk.gdk.color_parse('white'))						
			
			# self.fixed.put(self.healthstatus,640,660)			
			# self.healthstatus.show()
# ##11111111111111################   HOB DISPLAY ##########################


			# self.hob_buffer = gtk.TextBuffer()
			# self.hob=gtk.TextView()
			# self.hob.set_buffer(self.hob_buffer)
			# self.hob.set_editable(False)
			# self.hob.set_size_request(100,40)
			# self.hob.modify_base(gtk.STATE_NORMAL,gtk.gdk.color_parse('white')) 
			# self.hob.modify_text(gtk.STATE_NORMAL,gtk.gdk.color_parse('black'))
 			# self.hob.modify_bg(gtk.STATE_NORMAL,gtk.gdk.color_parse('white'))						
			
			# self.fixed.put(self.hob,240,555)			

			# self.hob.show()		
			
# ##11111111111111################   HOB INDICATOR ##########################


# #			self.hob_buffer = gtk.TextBuffer()



			# self.hob_indicator=gtk.TextView()

			# self.hob_indicator.set_size_request(40,40)
			# self.hob_indicator.modify_base(gtk.STATE_NORMAL,gtk.gdk.color_parse('red')) 
			# #self.hob_indicator.modify_text(gtk.STATE_NORMAL,gtk.gdk.color_parse('black'))
 			# self.hob_indicator.modify_bg(gtk.STATE_NORMAL,gtk.gdk.color_parse('red'))						
			
			# self.fixed.put(self.hob_indicator,200,555)			

			# self.hob_indicator.show()			
						



			
		
#######     all status displays     END  ###########################	
			


			
			
				#start button
			self.bst=gtk.Button("START")
			#self.bst.modify_lbl(gtk.STATE_NORMAL,gtk.gdk.color_parse('white')) 
			self.bst.set_size_request(100,40)
			self.bst.modify_bg(gtk.STATE_NORMAL,gtk.gdk.color_parse('darkgrey')) 			
			self.bst.show()
			self.fixed.put(self.bst,80,550)
			self.bst.connect("clicked",self.start_click)    #connect click signal to the required function
			#stop button
			self.bstop=gtk.Button("STOP")
			self.bstop.set_size_request(100,40)
			self.bstop.modify_bg(gtk.STATE_NORMAL,gtk.gdk.color_parse('darkgrey')) 
			self.bstop.show()
			self.fixed.put(self.bstop,80,600)
			self.bstop.connect("clicked",self.stop_click)
			
			
			#Parametric input button
			self.bpi=gtk.Button("INPUTS FROM OBC")
			self.bpi.set_size_request(200,40)
			self.bpi.modify_bg(gtk.STATE_NORMAL,gtk.gdk.color_parse('darkgrey')) 
			self.bpi.show()
			self.fixed.put(self.bpi,80,650)
			self.bpi.connect("clicked",self.pi_click)    #connect click signal to the required function        
			
			#exit button
			self.bexit=gtk.Button("EXIT")
			self.bexit.set_size_request(100,40)
			self.bexit.modify_bg(gtk.STATE_NORMAL,gtk.gdk.color_parse('darkgrey')) 
			self.bexit.show()
			self.fixed.put(self.bexit,230,600)
			self.bexit.connect("clicked",self.exit_click)    #connect click signal to the required function
			
			
			
			
			
			
			#NEW button
			self.betc=gtk.Button("etc.")
			self.betc.set_size_request(70,40)
			self.betc.modify_bg(gtk.STATE_NORMAL,gtk.gdk.color_parse('darkgrey')) 
			self.betc.show()
			self.fixed.put(self.betc,800,650)
			self.betc.connect("clicked",self.etc_click)    #connect click signal to the required function
				
					   

		 #labelS
			self.l=gtk.Label("        Fr")
			self.l.modify_fg(gtk.STATE_NORMAL,gtk.gdk.color_parse('black'))
			self.l.show()
			self.fixed.put(self.l,10,55)

			self.l=gtk.Label("        Fd")
			self.l.modify_fg(gtk.STATE_NORMAL,gtk.gdk.color_parse('black'))
			self.l.show()
			self.fixed.put(self.l,130,55)

			self.l=gtk.Label("SNR_up")
			self.l.modify_fg(gtk.STATE_NORMAL,gtk.gdk.color_parse('black'))
			self.l.show()
			self.fixed.put(self.l,610,55)
		   
			self.l=gtk.Label("SNR_down")
			self.l.modify_fg(gtk.STATE_NORMAL,gtk.gdk.color_parse('black'))
			self.l.show()
			self.fixed.put(self.l,730,55)
		
			self.l=gtk.Label("       HAPS ")
			self.l.modify_fg(gtk.STATE_NORMAL,gtk.gdk.color_parse('black'))
			self.l.show()
			self.fixed.put(self.l,250,55)

			self.l=gtk.Label("t")
			self.l.modify_fg(gtk.STATE_NORMAL,gtk.gdk.color_parse('black'))
			self.l.show()
			self.fixed.put(self.l,500,55)
			#--------------------------------------------_#---------------#---------------------------#
			self.l=gtk.Label("     GPS")
			self.l.modify_fg(gtk.STATE_NORMAL,gtk.gdk.color_parse('black'))
			self.l.show()
			self.fixed.put(self.l,375,55)
		
			
			self.l = gtk.Label("ON / OFF")
			self.l.modify_fg(gtk.STATE_NORMAL,gtk.gdk.color_parse('black'))
			self.l.show()
			self.fixed.put(self.l,850,77)
			# self.l=gtk.Label("Fb_d (kHz)")
			# self.l.modify_fg(gtk.STATE_NORMAL,gtk.gdk.color_parse('black'))
			# self.l.show()
			# self.fixed.put(self.l,850,55)

			
			
			# labels under scrolled windows

			#labelS
			self.l=gtk.Label("Range_freq (kHz)")
			self.l.modify_fg(gtk.STATE_NORMAL,gtk.gdk.color_parse('black'))
			self.l.show()
			self.fixed.put(self.l,10,77)

			self.l=gtk.Label(" Doppler_freq (Hz)")
			self.l.modify_fg(gtk.STATE_NORMAL,gtk.gdk.color_parse('black'))
			self.l.show()
			self.fixed.put(self.l,130,77)

			self.l=gtk.Label("S/N (dB)")
			self.l.modify_fg(gtk.STATE_NORMAL,gtk.gdk.color_parse('black'))
			self.l.show()
			self.fixed.put(self.l,610,77)
		   
			self.l=gtk.Label("S/N (dB)")
			self.l.modify_fg(gtk.STATE_NORMAL,gtk.gdk.color_parse('black'))
			self.l.show()
			self.fixed.put(self.l,730,77)
		
			self.l=gtk.Label("       Altitude (m) ")
			self.l.modify_fg(gtk.STATE_NORMAL,gtk.gdk.color_parse('black'))
			self.l.show()
			self.fixed.put(self.l,250,77)

			self.l=gtk.Label("     Altitude (m) ")
			self.l.modify_fg(gtk.STATE_NORMAL,gtk.gdk.
			color_parse('black'))
			self.l.show()
			self.fixed.put(self.l,375,77)			
			self.l=gtk.Label("Time  (sec)")
			self.l.modify_fg(gtk.STATE_NORMAL,gtk.gdk.color_parse('black'))
			self.l.show()
			self.fixed.put(self.l,500,77)

			# #---------------------------------------_#-------------------------#---------------------#
			self.l=gtk.Label("fb_up")
			self.l.modify_fg(gtk.STATE_NORMAL,gtk.gdk.color_parse('black'))
			self.l.show()
			self.fixed.put(self.l,440,550)
		
			self.l=gtk.Label("fb_down")
			self.l.modify_fg(gtk.STATE_NORMAL,gtk.gdk.color_parse('black'))
			self.l.show()
			self.fixed.put(self.l,730,550)
			
			#labelS
			self.l=gtk.Label("HOB_status")
			self.l.modify_fg(gtk.STATE_NORMAL,gtk.gdk.color_parse('black'))
			self.l.show()
			self.fixed.put(self.l,850, 55)			
			

			#self.l=gtk.Label("Reference altitude(m)")
			#self.l.show()
			#self.fixed.put(self.l,80,600)
			self.l=gtk.Label("                                               HAPS ")
			self.l.modify_font(pango.FontDescription("sans 20"))
			self.l.modify_fg(gtk.STATE_NORMAL,gtk.gdk.color_parse('black'))
			self.l.show()
			self.fixed.put(self.l,40,10)
			self.msgcl=gtk.Label("Error:")
			self.msgcl.modify_fg(gtk.STATE_NORMAL,gtk.gdk.color_parse('black'))
			self.msgcl.show()
			self.fixed.put(self.msgcl,200,20)
			self.l=gtk.Label("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------")
			self.l.modify_fg(gtk.STATE_NORMAL,gtk.gdk.color_parse('black'))
			self.l.show()
			self.fixed.put(self.l,200,40)
			window.show()
			self.ctr12=0
			self.ctr2=0
			self.ctr11=0
			self.ctr=0
			
			self.text=0
			
    def start_click(self,wid):
			self.ctr11=0            #intialize counter for buffer reading (buf)
			self.sbuflat=''                #intialize strings to null
			self.sbuflong=''
			self.sbufalt=''
			self.sbufrn=''
			self.sbufdop=''
			self.sbufsnr=''
			self.sbuftime=''
			#self.ser1=None
			self.stopDisp = False
			self.flag=True
			self.startHapTime = datetime.datetime.now()
			#self.curTime = time.strftime("%X %x %Z" )
			#self.curTime = ''
			nowtime = datetime.datetime.now()
			self.curTime = str(nowtime.year)+'_'+str(nowtime.month)+'_'+str(nowtime.day)+'_'+str(nowtime.hour)+'_'+str(nowtime.minute)+'_'+str(nowtime.second)
			print "creating threads"
			#thread for serial port reading and storing into file and buffer
			# for GPS
			#self.tser=threading.Thread(target=self.readSerportData)
			#time.sleep(0.5)    
			# for HAPS
			
			
			self.tser1=threading.Thread(target=self.sero1,args=(wid))				# HAPS serial communication thread
			self.tser=threading.Thread(target=self.readSerportData)				# GPS serial communication thread
			#provide sleep for serial port to open
			#thread for displaying
			#self.tdisp=threading.Thread(target=self.disp)
			self.tdisp1=threading.Thread(target=self.dispHapData,args=(wid))			# Thread to display and update GUI
			
			self.tser1.start()   #haps
			self.tser.start()  #GPS
			self.tdisp1.start()
			#thread1 = MyThread(1,"Thread-1",5,hapobj)
			#thread2 = MyThread(2,"Thread-2",5,hapobj)
			#thread1.start()
			#thread2.start()
			#self.tser.start()
			#self.tdisp.start()
			
    def sero1(self,wid):
			ct11=0
			#To create a log - HAPS
			self.hapfile=open('onlyhap'+ self.curTime + '.txt',"a")
			self.hapfile.write("------------------  [ "+time.strftime("%d:%m:%Y") + "   " )                
			self.hapfile.write(time.strftime("%H:%M:%S") + " ]  ------------------" + " \n")    
			self.hapfile.write("rangefreq\tdoppfreq\tSNR-up\t\tSNR-down\tF-up\t\t\tF-down\t\tMsg cnt\t\tAltitude\tGPStime\t\tHapsTime\t\tTrack flag\t\tSB_change\n\n\n")
			self.hapfile.write("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n")
			self.ctr2=self.ctr2+1
			self.startHapTime = datetime.datetime.now()
			
			#buffer to store data for display
			self.bufHAPS=[]
			
			#intialize and open port
			port="/dev/ttyMXUSB0"
			self.ser1 = serial.Serial(port, 115200,serial.EIGHTBITS,serial.PARITY_NONE,serial.STOPBITS_ONE,1.0)
			
			if not self.ser1.isOpen():
				self.ser1.open()
				print "PROBLEM"
			
			#self.ser1.flushInput()            #flush input and output buffer
			#self.ser1.flushOutput()
			print "\n serial port opened"
			print "\n waiting for signal"
			string=''
			
			self.hapsDataWaitCnt = 0
			self.hapsCheckSumMAtchCnt = 0
			self.hapsCheckSumNotMatchCnt = 0
			while self.flag:
				
				#ct11=ct11+1
				hapreadbyte=0
				self.hapchecksum=0
				
				if self.ser1.isOpen():
					while True :
						self.hapchecksum=0
						self.tempcrclow  = 0
						self.tempcrcup = 0
						#print "b"
						data = self.ser1.read(1)
						#print "header read"
						#print data.encode('hex')
						if len(data)==1:
							headerval =  data.encode('hex')
							#print "header obtained !!..checking now !!..."
							#print headerval
						
							if ((headerval=='07')):
								
								data_header2 = self.ser1.read(1)
								headerval_2 = data_header2.encode('hex')
								if((headerval_2 == 'E0') or (headerval_2 == 'e0')) :
								
								
								
								
									self.endHapTime = datetime.datetime.now()
							#if (data[0].encode('hex')=='a5'):
									print "header matched"
								#print data[0].encode('hex'),data[1].encode('hex'),
									break
							else:
								print headerval								
								self.hapsDataWaitCnt += 1
								#print "self.hapsDataWaitCnt"
								print self.hapsDataWaitCnt
							if self.hapsDataWaitCnt > 1000:
								print "Didn't Found Header .. .. .. 1000 nos.-- 1ms  HAPS data headers UNMATCHED"
								break
						
					if self.hapsDataWaitCnt <=1000:
						self.hapsDataWaitCnt = 0
						b11 = self.ser1.read(28)     #read 11 bytes after header match successfull
						
						if b11:    
							try:    
								#self.f=open('HAPS'+ s + '.txt',"a")
								#self.sbufrn=str((ord(b11[0])+ord(b11[1])*256)*1526)#+"\n"
								for i in range(24):
									self.hapchecksum ^= int(b11[i].encode('hex'),16)
									#print b11[i].encode('hex'),
									#print i
									# print"----------------------------"
									# print self.hapchecksum
									# print " -----------before---------"
								#self.hapchecksum^=int(b11[0].encode('hex'),16)                          # MSG_ID removed from checksum
								#print int(b11[5].encode('hex'),16),self.hapchecksum
								# print self.hapchecksum
								# print "----after-----"	
								self.tempcrclow = b11[24] 
								self.tempcrcup = b11[25]
								if (self.hapchecksum != int(self.tempcrclow.encode('hex'),16) ):
									print "checksum FAILED!!!"					

									self.checksumerrcnt += 1
									self.hapsCheckSumNotMatchCnt += 1
									print"________________________________"
									print self.hapchecksum
									print int(self.tempcrclow.encode('hex'),16) 
									print (self.tempcrcup)			
									print"________________"						
				

									print self.hapsCheckSumNotMatchCnt
									#self.msgcl.set_text(str(self.checksumerrcnt))
									#self.hap_data_available=True
									#for i in range(7):
										#print b11[i].encode('hex'),
									#print
								else:
									print "checksum matched !!"
									#print self.hapsCheckSumMAtchCnt
									self.hapsCheckSumMAtchCnt += 1
								
									#print "Checksum match = {0}, Checksum Not Matched = {1}".format(self.hapsCheckSumMAtchCnt,self.hapsCheckSumNotMatchCnt)
									#print "got"
									#self.headerAdded = b11[0].encode('hex')
									self.msgcount = int(b11[1].encode('hex'),16)
									self.msgid = int(b11[0].encode('hex'),16)
									#self.msgcl.set_text(str(msgcount))
									#self.rangeFreq = 1556
									#self.status = int(b11[2].encode('hex'),16)
									#self.sbufdop=str((ord(b11[2])+ord(b11[3])*256)*1526)#+"\n"
									#self.dopplerFreq = (ord(b11[3])+ord(b11[4])*256)*1526
									#self.rangeFreq = 1556
									self.len = int(b11[2].encode('hex'),16)
									self.reserved = int(b11[3].encode('hex'),16)
									
									self.status5 = int(b11[4].encode('hex'),16)
									self.status6 = int(b11[5].encode('hex'),16)
									self.status7 = int(b11[6].encode('hex'),16)
									self.status8 = int(b11[7].encode('hex'),16)
									hob_out = int(b11[23].encode('hex'),16)
##############################									
									if(self.status5 == 170) :
										self.status5string = "ON"
									elif(self.status5 == 85):
										self.status5string = "OFF"
									else :
										self.status5string = "blank data received"										
##############################										
									if(self.status6 == 3) :
										self.status6string = "80 MHz"
									elif(self.status6 == 1):
										self.status6string = "40 MHz"
									elif(self.status6 == 5):
										self.status6string = "120 MHz"
									else :
										self.status6string = "blank data received"															

#############################
									if(self.status7 == 2):
										self.status7string = "700 usec"
									elif(self.status7 == 1):
										self.status7string = "1000 usec"
									else :
										self.status7string = "blank data received"					
#############################
									if(self.status8 == 4) :
										self.status8string = "OK"
									else :
										self.status8string = "not OK"
										

									self.hob_reg = (ord(b11[16])+ord(b11[17])*256)*(0.9155)
									
									self.rangeFreq = (ord(b11[8])+ord(b11[9])*256)*4880
									
									self.ref_alt  = (ord(b11[18])+ord(b11[19])*256)*(0.9155)

									self.ref_pitch  = (ord(b11[21])+ord(b11[22])*256)			##  NAV data ack + NAV data SEQ nO.

									if ord(b11[20])<127:
										self.dopplerFreq = ord(b11[20])*4880
									else:
										self.dopplerFreq = (ord(b11[20])-256)*4880

									
								
									# if (ord(b11[20]) > 127) :
										
										# self.dopplerFreq = ord(b11[20]-255)*1220
									# else	:
										# self.dopplerFreq = ord(b11[20])*1220
									
									if ord(b11[10])>0:
										
										#self.sbufsnr=str((10*math.log10(ord(b11[4]))))#+"\n"
										self.snrVal_up = 20*math.log10(int(b11[10].encode('hex'),16))
										#string='NaN'
									else:
										self.snrVal_up = 0
										#self.sbufsnr='NaN'+""
										#string='NaN'
									
									
									if ord(b11[11])>0:
										#self.sbufsnr=str((10*math.log10(ord(b11[4]))))#+"\n"
										self.snrVal_down = 20*math.log10(int(b11[11].encode('hex'),16))
										#string='NaN'
									else:
										self.snrVal_down = 0
										#self.sbufsnr='NaN'+""
										#string='NaN'
										
									self.fup = ((ord(b11[12])+ord(b11[13])*256))*4880		#Fb-up 
										
									
									self.fdown = ((ord(b11[14])+ord(b11[15])*256))*4880	#Fb-down 

									
									# self.reservedbyte1 = (b11[17].encode('hex'))
									# self.reservedbyte2 = (b11[18].encode('hex'))
									# self.reservedbyte3 = (b11[19].encode('hex'))
									# self.reservedbyte4 = (b11[20].encode('hex'))
									# self.reservedbyte5 = (b11[21].encode('hex'))
									# self.reservedbyte6 = (b11[22].encode('hex'))
									self.hob_status = (b11[23].encode('hex'))			## HOB OUT Signal-------------------------------------
									# self.reservedbyte8 = (b11[24].encode('hex'))			# RESERVED bytes start here--uptil----byte b11[43]----------------
									# self.reservedbyte9 = (b11[25].encode('hex'))
									self.footerval1 = (b11[26].encode('hex'))
									self.footerval2 = (b11[27].encode('hex'))
									#print"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF"
									#print (self.footerval1)
									#print (self.footerval2)
									#print"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF"
									
									if(self.hob_status == '00') :
										self.hob_on_off = 'ON'
									elif((self.hob_status == 'FF' ) or(self.hob_status == 'ff')) :	
										self.hob_on_off = 'OFF'
									

									
									if((self.footerval1 == 'ff') and (self.footerval2 == 'ff')) : 
										
										 #print "*******************"
										print "footer detected"
										print self.hapDataCount
										#print "*******************"
										
									else:
										print "wrong footer"
									
								
										
																
																		
									
									
									self.hapfile.write("\n")
									self.hapfile.write(str(self.rangeFreq) + "\t\t" )
									#self.bufHAPS.append(self.sbufrn + "\n ")
									
									
									self.hapfile.write("%04d\t\t"%(self.dopplerFreq) )
									#self.bufHAPS.append(self.sbufdop +"\n")
									self.hapfile.write("%0.2f\t\t"%(self.snrVal_up))
									#self.bufHAPS.append(self.sbufsnr+ "\n")
									self.hapfile.write("%0.2f\t\t"%(self.snrVal_down))
									#self.bufHAPS.append(self.sbufsnr+ "\n")
									self.hapfile.write("%0.2f\t\t"%(self.fup))
									#self.bufHAPS.append(self.sbufsnr+ "\n")
									
									self.hapfile.write("%0.2f\t\t"%(self.fdown))
									#self.bufHAPS.append(self.sbufsnr+ "\n")
									#now = datetime.datetime.now()
									#c = str(time.strftime ("%H:%M"))
									#a1 = (int(now.strftime ("%f")))
									#a=a1/100
									#a=int(round(time.time()))
									#m,s=divmod(seconds,60)                    
									#h,m=divmod(m,60)
									#print "%d:%02d:%02d"%(h,m,s)
									#self.sbuftime= (c + ":" + str(a))            
									#self.buftime1=str((self.sbuftime))
									#self.systimedisp = self.buftime1
									#self.hapfile.write(str(self.buftime1) +"\t\t")
									
									#diff_time = self.endHapTime-self.startHapTime
									diff_time = self.endHapTime
									self.time_elapsed = round((diff_time.hour*3600 + diff_time.minute*60+diff_time.second+diff_time.microsecond*1e-6),4)
									#self.time_elapsed=diff_time.days*24*3600+diff_time.seconds+diff_time.microseconds*1e-6
									self.hapfile.write(str(self.msgcount)+"\t\t")
									self.hapfile.write(str(self.rangeFreq*0.0001875)+"\t\t")
									self.hapfile.write(str(self.gps_time)+"\t\t")
									self.hapfile.write(str(self.time_elapsed)+"\t\t")
									# track_flag = int(b11[24].encode('hex'))
									# sb_change = int(b11[25].encode('hex'))

									# for e in range(20) :
										# self.hapfile.write(str(b11[e+24].encode('hex'))+"\t\t")

									
									
									
									
									
									self.hapDataCount += 1
									#self.ser1.flushInput()
									#self.ser1.flushOutput()
									#self.time_elapsed = diff.days*24*3600 + diff.seconds + diff.microseconds*1e-6
									#self.hapfile.write(str(time_elapsed)+"\n")
									#self.hapDataCount += 1
									#time.sleep(0.0002)
									if ((self.hapDataCount % 150)==0):
										self.hap_data_available = True
										print "**********"
										# if(((self.rangeFreq*0.00009375)>= (self.hob_reg-2)) & ((self.rangeFreq*0.00009375)<= (self.hob_reg+2))) :
											# self.hob_indicator.modify_base(gtk.STATE_NORMAL,gtk.gdk.color_parse('green')) 
											# #self.hob_indicator.modify_text(gtk.STATE_NORMAL,gtk.gdk.color_parse('black'))
											# self.hob_indicator.modify_bg(gtk.STATE_NORMAL,gtk.gdk.color_parse('green'))						
										# else :
											# self.hob_indicator.modify_base(gtk.STATE_NORMAL,gtk.gdk.color_parse('red')) 
											# #self.hob_indicator.modify_text(gtk.STATE_NORMAL,gtk.gdk.color_parse('black'))
											# self.hob_indicator.modify_bg(gtk.STATE_NORMAL,gtk.gdk.color_parse('red'))						
													# #print self.hapDataCount/62			
			
						#print "####"
					
						#print "####"
						#print self.hap_data_available
					
					
									#print "****"
										#print self.hap_data_available
						#ct11=ct11+1
							#except :
								 #print "UUUUUUUUUUUUUUUUUUUUUUUUU"
								 #print "Unable to read HAPS data after Header" 
							finally:
								pass
					else:
						self.hapsDataWaitCnt = 0
						###############################-----------REVIEW-----------####################
						# self.ser1.close()
						# self.ser1 = serial.Serial(port, 115200,serial.EIGHTBITS,serial.PARITY_NONE,serial.STOPBITS_ONE,1.0)
						# self.ser1.open()
						# self.ser1.flushInput()            #flush input and output buffer
						# self.ser1.flushOutput()
			self.hapfile.close()
			self.ser1.close()
				
    
    def dispHapData(self,wid):
        while not self.stopDisp:
					if self.hap_data_available :
						#print self.hap_data_available/62
						#self.hap_data_available = False
						#t=self.tblat.get_start_iter()
						#self.tblat.insert(t,self.bufGPS[self.ctr])
						#self.tlat1.queue_draw()
						#t=self.tblong.get_start_iter()
						#self.tblong.insert(t,self.bufGPS[self.ctr+1])
						#self.tlong1.queue_draw()
						
						
						t=self.tbrn.get_start_iter()				#  display range frequency
						self.tbrn.insert(t,str(self.rangeFreq/1000)+"\n")
						self.trn.queue_draw()


						self.msgcl.set_text(str(self.checksumerrcnt ))		#  display checksum error count


						#time.sleep(0.2)
						t=self.tbdop.get_start_iter()				#  display doppler frequency
						self.tbdop.insert(t,str(self.dopplerFreq)+"\n")
						self.tdop.queue_draw()
				
						t=self.tbalt.get_start_iter()				#  display altitutde HAPS
						self.tbalt.insert(t,str(self.rangeFreq*0.0001875)+"\n")
						self.talt.queue_draw()


						t=self.tbsnr_up.get_start_iter()				#  display SNR-up value
						self.tbsnr_up.insert(t,str(round(self.snrVal_up,2))+"\n")
						self.tsnr_up.queue_draw()

						t=self.tbsnr_down.get_start_iter()				#  display SNR-down value
						self.tbsnr_down.insert(t,str(round(self.snrVal_down,2))+"\n")
						self.tsnr_down.queue_draw()

						t=self.tbf_down.get_start_iter()				#  display Fup value
						self.tbf_down.insert(t,str(self.fdown)+"\n")
						self.tf_down.queue_draw()

						t=self.tbf_up.get_start_iter()				#  display Fdown value
						self.tbf_up.insert(t,str(self.fup)+"\n")
						self.tf_up.queue_draw()

						t=self.hob_op_buff.get_start_iter()				#  update hob status scrolled window
						self.hob_op_buff.insert(t,str(self.hob_on_off)+"\n")
						self.hob_op.queue_draw()



						t=self.tbtime.get_start_iter()				# time elapsed
						self.tbtime.insert(t,str(self.time_elapsed)+"\n")
						self.ttime.queue_draw()

						
#############################################    status displays	###################					

						# t=self.txonstatus_buffer.get_start_iter()			##  clear text buffer				
						# e=self.txonstatus_buffer.get_end_iter()				
						# self.txonstatus_buffer.delete(t,e)
						
						# t=self.txonstatus_buffer.get_start_iter()				#  display TX ON/IFF STATUS
						# self.txonstatus_buffer.insert(t,str(self.status5string))
						# self.txonstatus.queue_draw()
						# print self.status5
# ################################
						
						
						# t=self.bandwidthstatus_buffer.get_start_iter()			##  clear text buffer				
						# e=self.bandwidthstatus_buffer.get_end_iter()				
						# self.bandwidthstatus_buffer.delete(t,e)

						# t=self.bandwidthstatus_buffer.get_start_iter()				#  display BANDWIDTH STATUS
						# self.bandwidthstatus_buffer.insert(t,str(self.status6string))
						# self.bandwidthstatus.queue_draw()
						# print self.status6
						
# #################################

						# t=self.waveformperiod_buffer.get_start_iter()			##  clear text buffer				
						# e=self.waveformperiod_buffer.get_end_iter()				
						# self.waveformperiod_buffer.delete(t,e)
						
						# t=self.waveformperiod_buffer.get_start_iter()				#  display  WAVEFORM PERIOD
						# self.waveformperiod_buffer.insert(t,str(self.status7string))
						# self.waveformperiod.queue_draw()
						# print self.status7
# ##################
						
						
						
						# t=self.healthstatus_buffer.get_start_iter()			##  clear text buffer				
						# e=self.healthstatus_buffer.get_end_iter()				
						# self.healthstatus_buffer.delete(t,e)

						# t=self.healthstatus_buffer.get_start_iter()				#  display HEALTH STATUS / DDS-PLL LOCK STATUS
						# self.healthstatus_buffer.insert(t,str(self.status8string))
						# self.healthstatus.queue_draw()
						# print self.status8
# ############################						
	


						# t=self.hob_buffer.get_start_iter()			##  clear text buffer				
						# e=self.hob_buffer.get_end_iter()				
						# self.hob_buffer.delete(t,e)
	
						# t=self.hob_buffer.get_start_iter()				#  display HOB IN METRE
						# self.hob_buffer.insert(t,str(self.hob_reg))
						# self.hob.queue_draw()
						# print self.hob_reg						
# #################################################################################################
						
						self.hap_data_available = False
						#print "####"
						#print self.hap_data_available
						

					
					
					if self.gps_data_available:
							#t=self.tblat.get_start_iter()
						#self.tblat.insert(t,self.bufGPS[self.ctr])
						#self.tlat1.queue_draw()
						#t=self.tblong.get_start_iter()
						#self.tblong.insert(t,self.bufGPS[self.ctr+1])
						#self.tlong1.queue_draw()
						self.gps_data_available = False
						t=self.tbaltgps.get_start_iter()
						self.tbaltgps.insert(t,str(self.alt_actual_val)+"\n")
						self.taltgps.queue_draw()
						
        if self.stopDisp :   
        	self.hapfile.close()
        	self.gpsfile.close()


   
    



    
    def pi_click(self, wid): # some callback function       

			# self.windowC = gtk.Window(gtk.WINDOW_TOPLEVEL)
			# color = gtk.gdk.color_parse('#dbaf23')
			# self.windowC.modify_bg(gtk.STATE_NORMAL, color)
			# self.windowC.set_title("Parameteric input window...")
			# self.windowC.resize(1300,550)
			
			
			
			self.windowK = gtk.Window(gtk.WINDOW_TOPLEVEL)
			color = gtk.gdk.color_parse('grey')
			self.windowK.modify_bg(gtk.STATE_NORMAL, color)
			self.windowK.set_title("newwwwwwwww....")
			self.windowK.resize(1300,550)
			# Create a Fixed Container
			self.fixed5 = gtk.Fixed()
			self.windowK.add(self.fixed5)
			self.fixed5.show()
			self.windowK.show()
			# # Create a Fixed Container
			# self.fixed5 = gtk.Fixed()
			# self.windowC.add(self.fixed5)
			# self.fixed5.show()		
						
						
						
			#7. f_down    HAPS
			self.tf_down=gtk.TextView()
			self.tf_down.set_editable(False)
			self.tf_down.set_size_request(100,100)
			self.tf_down.modify_base(gtk.STATE_NORMAL,gtk.gdk.color_parse('black')) 
			self.tf_down.modify_text(gtk.STATE_NORMAL,gtk.gdk.color_parse('green'))
			self.tf_down.show()

			#8. f_up    HAPS
			self.tf_up=gtk.TextView()
			self.tf_up.set_editable(False)
			self.tf_up.set_size_request(100,100)
			self.tf_up.modify_base(gtk.STATE_NORMAL,gtk.gdk.color_parse('black')) 
			self.tf_up.modify_text(gtk.STATE_NORMAL,gtk.gdk.color_parse('green'))
			self.tf_up.show()
			#5  f_up        
			self.tbf_up=gtk.TextBuffer()
			self.tf_up.set_buffer(self.tbf_up)

			#6. f_down       
			self.tbf_down=gtk.TextBuffer()
			self.tf_down.set_buffer(self.tbf_down)
			
			
			self.sf_up=gtk.ScrolledWindow()
			self.sf_down=gtk.ScrolledWindow()	
			self.sf_up.set_size_request(100,400)
			self.sf_down.set_size_request(100,400)
			self.sf_up.add(self.tf_up)
			self.sf_down.add(self.tf_down)
			
			self.sf_down.show()
			self.sf_up.show()
			
			self.fixed5.put(self.sf_up,30,30)        
			self.fixed5.put(self.sf_down,150,30)  
				
			t=self.tbf_down.get_start_iter()				#  display Fup value
			self.tbf_down.insert(t,str(self.fdown)+"\n")
			self.tf_down.queue_draw()

			t=self.tbf_up.get_start_iter()				#  display Fdown value
			self.tbf_up.insert(t,str(self.fup)+"\n")
			self.tf_up.queue_draw()

			
			#Feed button
			self.bfd=gtk.Button("feed")
			self.bfd.set_size_request(100,40)
			self.bfd.show()
			self.fixed5.put(self.bfd,220,450)
			self.bfd.connect("clicked",self.fd_click)    #connect click signal to the required function        
			
			
			
				#Navigation data button
			self.bnvg=gtk.Button("Navigation data")
			self.bnvg.set_size_request(130,40)
			self.bnvg.show()
			self.fixed5.put(self.bnvg,600,450)
			self.bnvg.connect("clicked",self.nvg_click)
			
				# labels
			self.l=gtk.Label("Header")
			self.l.show()
			self.fixed5.put(self.l,20,400)
			self.l=gtk.Label("Msgid")
			self.l.show()
			self.fixed5.put(self.l,20,430)
			self.l=gtk.Label("Burst altitude")
			self.l.show()
			self.fixed5.put(self.l,20,460)
			self.l=gtk.Label("Loopback")
			self.l.show()
			self.fixed5.put(self.l,20,490)
			self.l=gtk.Label("Reserved")
			self.l.show()
			self.fixed5.put(self.l,20,520)
			self.l=gtk.Label("CRC")
			self.l.show()
			self.fixed5.put(self.l,20,550)
			self.l=gtk.Label("Footer")
			self.l.show()
			self.fixed5.put(self.l,20,580)
			
			
		
			self.l=gtk.Label("Header")
			self.l.show()
			self.fixed5.put(self.l,420,400)
			self.l=gtk.Label("Msgid")
			self.l.show()
			self.fixed5.put(self.l,420,430)
			self.l=gtk.Label("Ref altitude")
			self.l.show()
			self.fixed5.put(self.l,420,460)
			self.l=gtk.Label("Ref pitch")
			self.l.show()
			self.fixed5.put(self.l,420,490)
			self.l=gtk.Label("Reserved")
			self.l.show()
			self.fixed5.put(self.l,420,520)
			self.l=gtk.Label("CRC")
			self.l.show()
			self.fixed5.put(self.l,420,550)
			self.l=gtk.Label("Footer")
			self.l.show()
			self.fixed5.put(self.l,420,580)
			
	

    def stop_click(self,wid):
			print "stop!!!!"
			self.flag=False
			self.stopDisp = True
			
			self.ser1.flushInput()
			self.ser1.flushOutput()
			self.ser1.close()
			
			self.serport.flushInput()
			self.serport.flushOutput()
			self.serport.close()
			#time.sleep(1)
			if self.ser1.isOpen():
				print"\n--------HAPS port-----still open-------\n"
			else :
				print "-----HAPS port----CLOSED=--------"
			self.hapfile.close()
			#print "\n serial port closed"
			
			
			if self.serport.isOpen():
				print"\n-------GPS port------still open-------\n"
			else :
				print "-----GPS port----CLOSED=--------"
			
			self.gpsfile.close()
        #self.ser1.flushInput()
        #self.ser1.close()
        #self.ser.flushInput()
        #self.f.close()
        #self.ser.close()
        #self.hapfile.close()
			
        #set it visible
    def exit_click(self,wid):
		 # if self.ser1.isOpen():
			# print"HAPS comm port still Open"
		 # else :
			 # print "HAPS comm port closed"
		 # if self.serport.isOpen():
			# print"GPS comm port still Open"
		 # else :
			 # print "GPS comm port closed"			 
		
		 gtk.main_quit()
		
    def quit_window(self,wid):
		print "quit"
		self.windowB.destroy()
        
    def main(self):
        # Enter the event loopp
        gtk.main()
        return 0
       
if __name__ == "__main__":
    f=HAPS()
    f.main()
