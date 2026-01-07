import sys
import os
from datetime import datetime
import traceback
try:
    from config import *
    if len(sys.argv)<2 or not sys.argv[1].endswith('.csv') or not os.path.isfile(sys.argv[1].strip()):
       print "No proper csv input given!"
       sys.exit() 
except:
    print "No config file found in the directory!"
    sys.exit()
    

class XacroGenerator:

    def __init__(self, csvFile, fileName = 'environment.xacro'):
        if fileName:
            self.fileName = fileName
            cHead['robot_name'] = fileName.split('.')[0]
        self.csvFile = csvFile
        self.fileContent = ''
        self.csvContent = []
        self.racks = {}
        self.globalOrigin = {}
        self.items = []

    def generateXacro(self):
        self.readCSV()
        self.addGlobalFrame()
        self.addRacks()
        ### adding a sample item
        self.addItem()
        self.addRoot()
        self.writeToXacro()
        #print self.fileContent

    def addRoot(self):
        cHead['body'] = self.body
        self.fileContent += cRoot%cHead

    def addGlobalFrame(self):
        self.body = cMaterials
        self.globalOrigin.update(cDefault)
        self.globalOrigin['vGeometry'] = eval(self.globalOrigin['vGeometry'])%self.globalOrigin
        self.globalOrigin['frame_name'] = 'env_global_frame'
        self.globalOrigin['origin'] = self.formStrPoint(self.globalOrigin)
        self.body += cLinks%self.globalOrigin

    def formStrPoint(self,point):
        if isinstance(point,dict):
            return str(point['x']) + ' ' + str(point['y']) + ' ' + str(point['z'])
        else:
            return str(point[0]) + ' ' + str(point[1]) + ' ' + str(point[2])

    def addRacks(self):
        try: 
            for i,r in enumerate(self.racks):
                rack = {}
                rack.update(self.racks[r])
                rackName = 'rack'+str(i)+'frame'
                rackType = rack['barcodes'][0]['parenttype'].upper()
                rackWidth = float(cRackConfig[rackType]['vRSize'][0])
                rackHeight = float(cRackConfig[rackType]['vRSize'][-1])
                self.getMidPoint(rack,rackType)
                if rackType in cRackWallBarcode:
                    rack['mid']['x'] = rack['mid']['x']-rackWidth/2.0-float(cBarcodeLabel['vSize'].split(" ")[0])/2.0
                else:
                    rack['mid']['x'] = rack['mid']['x']-(float(cRackConfig[rackType]['vLSize'][0]) + rackWidth/2.0)
                    
                rack['mid']['z'] = rackHeight/2 ###need review
                origin  = self.getRelativePos(self.globalOrigin,rack['mid'])
  
                tRack = {}
                tRack.update(cRack) 
                tRack['vSize'] = self.formStrPoint(cRackConfig[rackType]['vRSize'])
                tRack['frame_name'] = rackName
                tRack['vGeometry'] = eval(tRack['vGeometry'])%tRack

                tConnect = {}
                tConnect.update(cConnect)
                tConnect['join_name'] = rackName + 'join_' + self.globalOrigin['frame_name']
                tConnect['lOrigin'] = self.formStrPoint(origin) ###possible issue
                tConnect['parent_frame'] = self.globalOrigin['frame_name'] 
                tConnect['child_frame'] = rackName                  
                self.body += cJoins%tConnect

                self.body += cLinks%tRack

                self.addLevels(origin, rackName, rackType, rack['barcodes'][:])
                    
        except:
            print traceback.format_exc()
            print 'Error while generating racks!'
            sys.exit()

    def addLevels(self,rackOrigin,rackName,rackType='T1',barcodes=[]):
        '''
        * description: explain the function
        * input: 
        * output:
        '''
        levels = cRackConfig[rackType]['vLevels']
        if isinstance(levels,int):
            levelsZOrigin = None
            noOfLevels = levels
        else:
            if rackType in cVariableSpaced:
                levelsZOrigin = levels
            else:
                levelsZOrigin = None
            noOfLevels = len(levels)
        levelSpace = cRackConfig[rackType]['vRSize'][-1]/(noOfLevels-1) ###Count only spaces in-between
        for l in xrange(noOfLevels):
            levelName = 'level'+str(l)+rackName
            if levelsZOrigin:  ### varibale level space
                levelZOrigin = levelsZOrigin[l]
            else:
                levelZOrigin = l*levelSpace

            if rackType in cDoublesided:
                levelOriginLocal = rackOrigin[:2]+( levelZOrigin ,) ###relative to rack origin double sided rack
            else:
                levelOriginLocal = ( rackOrigin[0] + cRackConfig[rackType]['vLSize'][0]/2.0 ,) + rackOrigin[1:2] + ( levelZOrigin ,)
            levelOrigin = self.getRelativePos(rackOrigin,levelOriginLocal)
            
            tLevel = {}
            tLevel.update(cLevel)
            tLevel['vSize'] = self.formStrPoint(cRackConfig[rackType]['vLSize']) ###change based on rack y
            tLevel['frame_name'] = levelName
            tLevel['vGeometry'] = eval(tLevel['vGeometry'])%tLevel

            tConnect = {}
            tConnect.update(cConnect)
            tConnect['join_name'] = levelName + 'join_' + rackName
            tConnect['lOrigin'] = self.formStrPoint(levelOrigin) ###possible issue
            tConnect['parent_frame'] = rackName 
            tConnect['child_frame'] = levelName                    
            self.body += cJoins%tConnect

            self.body += cLinks%tLevel

            barcodeWidth = float(cBarcodeLabel['vSize'].split(" ")[0])
            count=0
            while barcodes:
                if rackType in cRackWallBarcode:
                    barcode = barcodes.pop(count)
                    barcodeOrigin = ( barcode['x']-rackOrigin[0] , barcode['y']-rackOrigin[1], barcode['z']-rackOrigin[2] )
                    barcode['origin'] = barcodeOrigin
                    self.addBarcode(barcode,rackName)
                elif rackType in cRackFloatBarcode: ###check for variable spaced
                    if (rackType in cVariableSpaced and levelsZOrigin[l] <= barcodes[count]['z'] and ((l+1<levels and barcodes[count]['z']<levelsZOrigin[l+1]) or l+1>=levels)) \
                    or (rackType not in cVariableSpaced and levelOriginLocal[2] <= barcodes[count]['z'] < levelOriginLocal[2]+levelSpace):
                        barcode = barcodes.pop(count)
                        barcodeOrigin = ( barcode['x'], barcode['y'], barcode['z'] )
                        barcodeOrigin = self.getRelativePos(levelOriginLocal,barcodeOrigin)
                        barcode['origin'] = barcodeOrigin
                        self.addBarcode(barcode,levelName)
                    else:
                        if count<len(barcodes)-1:
                            count += 1
                        else:
                            count = 0
                            break
                else: ###check for equally spaced
                    if (rackType in cVariableSpaced and ((l-1>=0 and levelsZOrigin[l]-(levelsZOrigin[l]-levelsZOrigin[l-1])/2.0 <= barcodes[count]['z']) or l-1<0) and ((l+1<levels and barcodes[count]['z']<levelsZOrigin[l]+((levelsZOrigin[l+1]-levelsZOrigin[l])/2.0)) or l+1>=levels)) \
                    or (rackType not in cVariableSpaced and -levelSpace/2.0 <= barcodes[count]['z']-levelOriginLocal[2] <= levelSpace/2.0): ###Pushing barcode in its level
                        barcode = barcodes.pop(count)
                        barcodeOrigin = ( levelOriginLocal[0]+(cRackConfig[rackType]['vLSize'][0]/2.0)+(barcodeWidth/2.0) , barcode['y'],levelOriginLocal[2] )
                                      ###Need to change x for double sided
                        barcodeOrigin = self.getRelativePos(levelOriginLocal,barcodeOrigin)
                        barcode['origin'] = barcodeOrigin
                        self.addBarcode(barcode,levelName)
                    else:
                        if count<len(barcodes)-1:
                            count += 1
                        else:
                            count = 0
                            break


    def addBarcode(self,barcode,parentName):
        tBarcodeLabel = {}
        tBarcodeLabel.update(cBarcodeLabel)
        tBarcodeLabel['frame_name'] = barcode['name']
        tBarcodeLabel['vGeometry'] = eval(tBarcodeLabel['vGeometry'])%tBarcodeLabel

        tConnect = {}
        tConnect.update(cConnect)
        tConnect['join_name'] = barcode['name'] + 'join_' + parentName
        tConnect['lOrigin'] = self.formStrPoint(barcode['origin']) ###possible issue
        tConnect['parent_frame'] = parentName 
        tConnect['child_frame'] = barcode['name']

        self.body += cJoins%tConnect

        self.body += cLinks%tBarcodeLabel


    def addItem(self,items=[]):
        if not items:
            items = self.items
        for i, item in enumerate(items):
            tItem = {}
            tItem.update(cItem)
            tItem['frame_name'] = item['name']
            tItem['vGeometry'] = eval(tItem['vGeometry'])%tItem

            parent = item.get('parent','')
            if not parent:
                parent = {}
                parent.update(self.globalOrigin)

            itemOrigin = self.getRelativePos(parent,item)
            tConnect = {}
            tConnect.update(cConnect)
            tConnect['join_name'] = item['name'] + 'join_' + parent['frame_name']
            tConnect['lOrigin'] = self.formStrPoint(itemOrigin) ###possible issue
            tConnect['parent_frame'] = parent['frame_name'] 
            tConnect['child_frame'] = item['name']

            self.body += cJoins%tConnect

            self.body += cLinks%tItem


    def getRelativePos(self,pos1,pos2):
        if isinstance(pos1,dict) and isinstance(pos2,dict):
            return pos2['x']-pos1['x'], pos2['y']-pos1['y'], pos2['z']-pos1['z']
        else:
            return pos2[0]-pos1[0], pos2[1]-pos1[1], pos2[2]-pos1[2]
            
    def getMidPoint(self,rack,rackType="T1"):
        if rack["barcodes"] and rack["barcodes"][0].get("parentrelation",""):
            parentrelation = rack["barcodes"][0].get("parentrelation","")
            x,y,z = map(float, rack["barcodes"][0]["parentrelationalpoint"].split(","))
            if parentrelation=='TR':
                xmid = x if cGlobalFlag else rack['x']
                ymid = y-(cRackConfig[rackType]['vRSize'][1]/2.0) if cGlobalFlag else (rack['y']+y)-(cRackConfig[rackType]['vRSize'][1]/2.0)
                zmid = z-(cRackConfig[rackType]['vRSize'][2]/2.0) if cGlobalFlag else (rack['z']+z)-(cRackConfig[rackType]['vRSize'][2]/2.0)
            elif parentrelation=='BR':
                xmid = x if cGlobalFlag else rack['x']
                ymid = y-(cRackConfig[rackType]['vRSize'][1]/2.0) if cGlobalFlag else (rack['y']+y)-(cRackConfig[rackType]['vRSize'][1]/2.0)
                zmid = z+(cRackConfig[rackType]['vRSize'][2]/2.0) if cGlobalFlag else (rack['z']-z)+(cRackConfig[rackType]['vRSize'][2]/2.0)
            elif parentrelation=='TL':
                xmid = x if cGlobalFlag else rack['x']
                ymid = y+(cRackConfig[rackType]['vRSize'][1]/2.0) if cGlobalFlag else (rack['y']-y)+(cRackConfig[rackType]['vRSize'][1]/2.0)
                zmid = z-(cRackConfig[rackType]['vRSize'][2]/2.0) if cGlobalFlag else (rack['z']+z)-(cRackConfig[rackType]['vRSize'][2]/2.0)
            elif parentrelation=='BL':
                xmid = x if cGlobalFlag else rack['x']
                ymid = y+(cRackConfig[rackType]['vRSize'][1]/2.0) if cGlobalFlag else (rack['y']-y)+(cRackConfig[rackType]['vRSize'][1]/2.0)
                zmid = z+(cRackConfig[rackType]['vRSize'][2]/2.0) if cGlobalFlag else (rack['z']-z)+(cRackConfig[rackType]['vRSize'][2]/2.0)
        else:
            total = float(len(rack['barcodes']))
            xmid = rack['x']/total
            ymid = rack['y']/total
            zmid = rack['z']/total         
        rack['mid'] = {'x':xmid,'y':ymid,'z':zmid}
        return rack['mid']

    def processCSV(self,csvRow):
        if csvRow.get('name','') == 'Origin':
           self.globalOrigin = {}
           self.globalOrigin.update(csvRow)
        elif csvRow.get('name','').startswith('Item'):
           self.items.append(csvRow)
        elif csvRow.get('parent',''):
           rack = csvRow['parent']
           rRelation = csvRow.get('parentrelation','')
           barcode = {}
           barcode.update(csvRow)
           if self.racks.get(rack):
              self.racks[rack]['barcodes'].append(barcode)
              if rRelation:
                  self.racks[rack]['x'] = barcode['x']
                  self.racks[rack]['y'] = barcode['y']
                  self.racks[rack]['z'] = barcode['z']
                  self.racks[rack]['relation']=rRelation
              elif not self.racks[rack]['relation']:
                  self.racks[rack]['x'] += barcode['x']
                  self.racks[rack]['y'] += barcode['y']
                  self.racks[rack]['z'] += barcode['z']
           else:
              self.racks[rack] = { 'barcodes':[barcode], 'x':barcode['x'], 'y':barcode['y'], 'z':barcode['z'], 'relation':rRelation }
    
    def readCSV(self):
        try:
           with open(self.csvFile,'r') as csvRead:
               csvContent = csvRead.readlines()
           self.validateCSV(csvContent)    
        except:
           print "Unable to open the CSV!"
           sys.exit()

    def validateCSV(self,csvContent):
        header = []
        try:
            for data in csvContent:
                data = data.replace('\n','').strip().split('|')
                data  = [ i.strip() for i in data ]
                if not header and data:
                    header = data[:]
                elif header and data:
                    csvRow = { header[i]:float(data[i]) if header[i] in ('x','y','z') else data[i] for i in xrange(len(header))} 
                    ###dict(zip(header,data))
                    self.processCSV(csvRow)
        except:
            print "Invalid csv!"
            sys.exit()

    def writeToXacro(self):
        with open(self.fileName, 'w') as xacroFile:
            xacroFile.write(self.fileContent)
        print "Xacro is successfully generated and saved in %s!"%self.fileName


if __name__ == '__main__':
   csvFile = sys.argv[1].strip()
   fileName = 'environment.xacro' ###'rfhHandheldTest.xacro'
   if os.path.isfile(fileName):
      replaceFile  = raw_input("There is already a file named with "+fileName+". Do you want to replace it?(Yes/No)")
      if not replaceFile.strip().lower() in ('','yes'):
         fileName = fileName.split('.')[0]+datetime.now().strftime('%Y%m%d%H%M%S')+'.xacro'
   x = XacroGenerator(csvFile, fileName)
   x.generateXacro()
