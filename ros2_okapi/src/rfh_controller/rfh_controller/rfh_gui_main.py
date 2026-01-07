#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import pickle
import os,sys
import datetime
# import tkMessageBox
import traceback
from rfidbot_tags_reader.msg import TagReader
from tkinter import *
from tkinter import messagebox
from std_msgs.msg import String,Bool,Int16MultiArray,Int32,Int16
import subprocess
import time
import argparse



NAV_MODE_INVENT = 1
NAV_MODE_LOCALIZE = 2

RFD8500_START_INV = 1
RFD8500_STOP_INV = -1
RFD8500_STOP_EXIT = 0

class okapi_gui(Node):
    def __init__(self):
        
        super().__init__("okapi_gui_")
        self.nodename=self.get_name()
        self.get_logger().info("%s started "%self.nodename)
        
        self.send_str=''
        self.uniqueEPC=[]
        self.readCount = 0
        
        self.root = Tk()
        self.root.protocol('WM_DELETE_WINDOW', self.reader_exit) 
        self.mainFrame = Frame(self.root)
        self.mainFrame.pack(fill=BOTH, expand=YES)


        #set the defualt size of the window
        w = self.root.winfo_screenwidth() * 0.2 
        h =  self.root.winfo_screenheight()
        self.root.geometry("%dx%d+0+0" % (w, h))

         #set the column and row weight
        for i in range(0,4):
            self.mainFrame.columnconfigure(i, weight=1)
        for j in range(0,18):
            self.mainFrame.rowconfigure(j, weight=1)
        #row 3,4 need specific weight
        self.mainFrame.rowconfigure(3, weight=2)    
        self.mainFrame.rowconfigure(4, weight=3)    

        
        #Set the label display for tag reading
        self.var = StringVar()
        self.label_uniqueTagNum  = Label(self.mainFrame, text="",font="Times 28 bold")#,fg = "orange")    
        self.label_uniqueTagSign = Label(self.mainFrame, text="Unique Tags",font="Times 15 bold")   
        self.label_tagReads      = Label(self.mainFrame, text="",font="Times 28 bold")#,fg = "orange")   
        self.label_readsSign     = Label(self.mainFrame, text="Reads",font="Times 15 bold")  
        
        #Set buttons
        self.buttonsFrame = Canvas(self.mainFrame)
        self.buttonsFrame.grid()
        self.buttonsFrame.config(relief=GROOVE, bd=2)

        self.button_start_inv = Button(self.buttonsFrame,text='Start Inventory',
                                width=12, command=self.start_inventory)
        #self.button_start_inv.pack(side=LEFT,expand=YES)#grid(row=0, column=0, sticky=E)
        self.button_start_inv.grid(row=0, column=0, sticky=E, padx=2,pady=2)
        self.button_stop_inv = Button(self.buttonsFrame, text='Stop Inventory',
                                width=12, command=self.stop_inventory)
        #self.button_stop_inv.pack(side=LEFT,expand=YES)#grid(row=0, column=1, sticky=W)
        self.button_stop_inv.grid(row=0, column=1, sticky=W,padx=2,pady=2)

        self.button_save_tags = Button(self.buttonsFrame,text='Save',
                                width=12, command=self.save2File)
        self.button_save_tags.grid(row=1, column=0, sticky=E, padx=2,pady=2)
        self.button_read_tags = Button(self.buttonsFrame,text='Read',
                                width=12, command=self.readFromFile)
        self.button_read_tags.grid(row=1, column=1, sticky=W, padx=2,pady=2)
        
        self.button_localize_alltags = Button(self.buttonsFrame,text='Localize All Tags',
                                width=12, command=self.localizeAllTags)
        self.button_localize_alltags.grid(row=2, column=0, sticky=E,padx=2,pady=2)

        self.button_inv_report = Button(self.buttonsFrame,text='inv report',
                                width=12, command=self.reportUniquetags)
        self.button_inv_report.grid(row=2, column=1, sticky=W,padx=2,pady=2)

        #Set menu
        menubar = Menu(self.root,activebackground='LIGHT BLUE')
        filemenu = Menu(menubar, tearoff=0,bg='WHITE',activebackground='LIGHT BLUE')
        filemenu.add_command(label="Exit", command=self.reader_exit)
        menubar.add_cascade(label="File", menu=filemenu)

        statisticsmenu = Menu(menubar, tearoff=0,bg='WHITE',activebackground='LIGHT BLUE')
        statisticsmenu.add_command(label="Unique Tag Numbers", command=self.unique_tags)
        menubar.add_cascade(label="Statistics", menu=statisticsmenu)
         
        #Set EPC show ListBox with scrollbar
        self.EPCListFrame=Frame(self.mainFrame)
        self.EPCListFrame.place(x=30,y=100)
        self.scrollbarY = Scrollbar(self.EPCListFrame) 
        self.scrollbarY.pack(side=RIGHT, fill=Y)
        self.scrollbarX = Scrollbar(self.EPCListFrame,orient=HORIZONTAL)
        self.scrollbarX.pack(side=BOTTOM, fill=X)
        
        self.EPClist = Listbox(self.EPCListFrame, 
                                yscrollcommand = self.scrollbarY.set,
                                xscrollcommand = self.scrollbarX.set, 
                                font="Courier 14 bold", width=80, height=20)  #height is in lines
        self.scrollbarY.config(command=self.EPClist.yview)
        self.scrollbarX.config(command=self.EPClist.xview)
        title='         '+'EPC'+'             '
        self.EPClist.insert(END, str(title))
        self.EPClist.pack(fill=X, expand=YES)  #FILL x (fill horizontally), Y (fill vertically), or BOTH


        #place the label& frame in the main frame
        self.label_uniqueTagSign.grid(row=0, column=0, sticky=W+E+N+S)
        self.label_uniqueTagNum.grid(row=0, column=1, sticky=W+E+N+S)
        self.label_readsSign.grid(row=1, column=0, sticky=W+E+N+S)
        self.label_tagReads.grid(row=1, column=1, sticky=W+E+N+S)

        self.EPCListFrame.grid(row=2, column=0, rowspan=3,
                                columnspan=2, padx=4, sticky=W+E+N+S)
        self.buttonsFrame.grid(row=5, column=0,
                                columnspan=2,padx=2, pady=4,sticky=W+E+N+S)
                                
        #initialize the message publisher
        self.send_EPC = self.create_publisher(String, "/loc_a_tag", 10)
        self.RFD8500_pub = self.create_publisher(Int16, "/set_rfd8500", 1)
        self.saverawdata_pub = self.create_publisher(Bool, "/save_raw_data", 1)
        self.readrawdata_pub = self.create_publisher(String, "/read_raw_data", 1)
        self.localize_all_tag_pub = self.create_publisher(Bool, "/loc_all_tag", 1)
        #for python fixreader 
        self.pausefixreaderinv_pub = self.create_publisher(Bool, "/pause_inventory_delete_pos", 1)
        self.resumefixreaderinv_pub = self.create_publisher(Bool, "/resume_inventory_enable_new_pos", 1)
        #for zebra reader with "zebra_rfid_reader"
        self.zebraReaderInv_pub=self.create_publisher(Bool, "/FX9600inventory", 1)
     

        #initial message subscribers
        self.tag_reader_sub = self.create_subscription(TagReader, "rfid_tags", self.tag_readingCallback, 10)
        

        self.root.config(menu=menubar)
        self.root.update()
        self.root.after(1000,self.display)
        # self.root.mainloop()  # Tk mainloop handled in spin()
        


    def start_inventory(self):
        self.get_logger().warn('start inventory')
        self.RFD8500_pub.publish(Int16(data=RFD8500_START_INV))
        self.resumefixreaderinv_pub.publish(Bool(data=True))
        self.zebraReaderInv_pub.publish(Bool(data=True))
        
    def stop_inventory(self):
        self.get_logger().warn('stop inventory')
        self.RFD8500_pub.publish(Int16(data=RFD8500_STOP_INV))
        self.pausefixreaderinv_pub.publish(Bool(data=True))
        self.zebraReaderInv_pub.publish(Bool(data=False))

    def save2File(self):
        self.get_logger().warn('save rawdata to file')
        self.saverawdata_pub.publish(Bool(data=True))

    def readFromFile(self):
        self.get_logger().warn('read rawdata from file, not completed!')
        #filename = raw_input("please input the file name(no parents folder):  ")
        #self.readrawdata_pub.publish(filename)  
  
    def localizeAllTags(self):
        self.get_logger().warn('localize all tags')
        self.localize_all_tag_pub.publish(Bool(data=True))


    def reportUniquetags(self):
        '''
        * description: save the Unique Tags into file
        * input: raw
        '''
        path = sys.path[0]
        pardir = os.path.abspath(os.path.join(path, os.pardir))
        inventoryReportPardir = pardir + '/inventoryreports'
        
        filenamePrefix = 'inventory'
        #generate year month day hour minute as suffix filename
        ymd = datetime.datetime.now()   
        filenameSuffix = '_'+str(ymd.year)+'_'+str(ymd.month)+'_'+str(ymd.day)
        filenameSuffix = filenameSuffix + '_'+str(ymd.hour) +'_'+ str(ymd.minute)+ '.txt'
        invReportName = inventoryReportPardir +'/'+ filenamePrefix + filenameSuffix
        
        #rospy.logwarn('save inventory to %s',invReportName)
        
        with open(invReportName, 'wb') as f:
            #pickle.dump(self.uniqueEPC, f)
            for i in self.uniqueEPC:
                f.write(i)
                f.write(
"\n")
                
        self.get_logger().warn("save raw data to file %s",invReportName)


    def unique_tags(self):
        try:
            if self.tagList is not None:
                msg = "Total tag count is %s.\nUnique number of Tags is %s"%(self.totalTagCount,len(self.tagList))
        except:
            self.all_tags()
            msg = "Total tag count is %s.\nUnique number of Tags is %s"%(self.totalTagCount,len(self.tagList))
            self.root.option_add('*font','Helvetica -20')
            messagebox.showinfo("Tag Info",msg)
            self.root.option_clear()  

   
    def reader_exit(self):
        self.RFD8500_pub.publish(Int16(data=RFD8500_STOP_EXIT))
        self.get_logger().info("Exit!!!!!!!!!!!!!!!!!!!!!")
        rclpy.shutdown()
        self.root.destroy()
        quit()
        
       
    def tag_readingCallback(self,msg):
        tagEPC = msg.EPC.lower()
        self.readCount +=1
        if tagEPC not in self.uniqueEPC:
            self.uniqueEPC.append(tagEPC)
            #update to gui list
            #self.EPClist.insert(END, tagEPC)
            self.EPClist.insert(1, tagEPC)
            self.EPClist.pack(fill=X, expand=YES)
        else:
            idx = self.EPClist.get(0, END).index(tagEPC)
            self.EPClist.delete(idx)
            self.EPClist.insert(1, tagEPC)
            self.EPClist.pack(fill=X, expand=YES)
            
            #self.root.update()  
        #self.display() move to spin 
        
        
    def send(self,EPC_str):
        EPC_select = String()
        EPC_select.data = EPC_str
        if EPC_select.data!=self.send_str:
            self.send_EPC.publish(EPC_select)
            self.send_str=EPC_select.data  
        
    def display(self):
        read_cur=self.EPClist.curselection()

        #localize the selected tag
        if len(read_cur)!=0:
            #rospy.logwarn('Read_cur len: %s',EPC_list[int(read_cur[0])-1])
            str__=self.EPClist.get(int(read_cur[0]))
            str_send=''
            for i in range(len(str__)):
                if str__[i]!=' ':
                    str_send=str_send+str__[i]
                else:
                    break
            self.send(str_send)
        #update the tags reads    
        self.label_uniqueTagNum["text"]=len(self.uniqueEPC)
        self.label_tagReads["text"]= self.readCount
        
        self.root.after(50,self.display)
        
    def spin(self):
        self.r=100
        period = 1.0 / float(self.r)
        while rclpy.ok():
            # process ROS2 callbacks
            rclpy.spin_once(self, timeout_sec=0.0)
            # update Tkinter GUI
            try:
                self.root.update_idletasks()
                self.root.update()
            except Exception:
                break
            time.sleep(period)
    
   
if __name__ == '__main__':
    rclpy.init(args=None)
    okapiGUI=okapi_gui()
    try:
        okapiGUI.spin()
    finally:
        okapiGUI.destroy_node()
        rclpy.shutdown()

