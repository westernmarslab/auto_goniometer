import time
import RPi.GPIO as GPIO
import os
import numpy as np
read_command_loc='/home/pi/PiShare/commands/from_control/'
write_command_loc='/home/pi/PiShare/commands/from_pi/'

INTERVAL=0.25

def main():
    delme=os.listdir(read_command_loc)
    for file in delme:
        os.remove(read_command_loc+file)
    delme=os.listdir(write_command_loc)
    for file in delme:
        os.remove(write_command_loc+file)
    
    pi_controller=PiController(read_command_loc)
    pi_controller.listen()
    #pi_controller.led_on()
    
class PiController():
    def __init__(self,read_command_loc):
        self.goniometer=Goniometer(None,None,self)
        self.cmdfiles0=[]
        self.cmdnum=0
        self.read_command_loc=read_command_loc
        self.write_command_loc=write_command_loc
        self.dir='forward'
        
    def listen(self):
        print('listening!')
        t=0
        while True:
            self.cmdfiles=os.listdir(self.read_command_loc)  
            if self.cmdfiles==self.cmdfiles0:
                pass
            else:
                for cmdfile in self.cmdfiles:
                    if cmdfile not in self.cmdfiles0:
                        #Keep command directories clean by removing files as you go. Occasionally, pi may try to delete a file before control is done writing. That is what the try/except blocks are for.
                        try:
                            os.remove(self.read_command_loc+'\\'+cmdfile)
                        except:
                            try:
                                time.sleep(0.5)
                                os.remove(self.read_command_loc+'\\'+cmdfile)
                            except:
                                pass
                        cmd, params=self.decrypt(cmdfile)
                        for x in range(10):
                            cmd=cmd.replace(str(x),'')
                        print(cmd)
                        if cmd=='movetray':
                            if 'steps' in params:
                                steps=int(params[0])
                                if steps>0:
                                    self.goniometer.tray_motor.forward(steps, change_pos=False)
                                else:
                                    self.goniometer.tray_motor.backward(np.abs(steps), change_pos=False)
                            else:
                                print('moving to pos')
                                print(params[0])
                                self.goniometer.move_sample(params[0])
                            filename=self.encrypt('donemoving')
                            self.send(filename)
                        elif cmd=='movedetector':
                            if 'steps' in params:
                                steps=int(params[0])
                                if steps>0:
                                    self.goniometer.e_motor.forward(steps, change_pos=False)
                                else:
                                    self.goniometer.e_motor.backward(np.abs(steps), change_pos=False)
                                filename=self.encrypt('donemovingdetector')
                            else:
                                self.goniometer.set_emission(params[0])

                                if self.goniometer.e_motor.position!=None:
                                    filename=self.encrypt('donemovingdetector')
                                else:
                                    filename=self.encrypt('nopiconfig')
                            self.send(filename)
                            
                        elif cmd=='movelight':
                            if 'steps' in params:
                                steps=int(params[0])
                                print(params)
                                if steps>0:
                                    self.goniometer.i_motor.forward(steps, change_pos=False)
                                else:
                                    self.goniometer.i_motor.backward(np.abs(steps), change_pos=False)
                                filename=self.encrypt('donemovinglight')
                            else:
                                self.goniometer.set_incidence(params[0])

                                if self.goniometer.i_motor.position!=None:
                                    filename=self.encrypt('donemovinglight')
                                else:
                                    filename=self.encrypt('nopiconfig')
                            self.send(filename)
                            
                        elif cmd=='configure':
                            self.goniometer.configure(params[0],params[1],params[2])
                            filename=self.encrypt('piconfigsuccess')
                            self.send(filename)
                            
                            
                        
                        
            self.cmdfiles0=list(self.cmdfiles)
            t=t+INTERVAL
            time.sleep(INTERVAL)
        
        
    def led_on(self):
        # Use physical pin numbering
        GPIO.setup(18, GPIO.OUT, initial=GPIO.HIGH)  
        
    def send(self,filename):
        try:
            file=open(self.write_command_loc+filename,'w')
        except OSError as e:
            if e.errno==22:
                pass
            else:
                raise e
        except Exception as e:
            raise e
        
    def encrypt(self,cmd,parameters=[]):
        filename=cmd+str(self.cmdnum)
        self.cmdnum+=1
        print(filename)
        for param in parameters:
            param=param.replace('/','+')
            param=param.replace('\\','+')
            param=param.replace(':','=')
            filename=filename+'&'+param
        return filename
        
    def decrypt(self,encrypted):
        cmd=encrypted.split('&')[0]
        params=encrypted.split('&')[1:]
        i=0
        for param in params:
            params[i]=param.replace('+','\\').replace('=',':')
            params[i]=params[i].replace('++','+')
            i=i+1
        return cmd,params
    

class Motor():
    def __init__(self,name,pins,position,step_size, delay):
        self.name=name
        self.pins=pins
        try:
            self.__position=float(position)
        except:
            self.__position=position
            
        self.steps_per_degree=step_size
        self.delay=delay
        
        for pin in self.pins:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW) 
        
    @property
    def position(self):
        try:
            return float(self.__position)
        except:
            return None
        
    @position.setter
    def position(self, val):
        val=float(val)
        print(self.name+' motor moving from '+str(self.position) +'to '+str(val))
        delta=np.abs(self.position-val)
        numsteps=int(delta*self.steps_per_degree)

        if self.position<val:
            self.forward(numsteps)
        else:
            if self.name=='Sample tray':
                numsteps=int(1.02*numsteps)
            self.backward(numsteps)
        
            
    def forward(self,steps, change_pos=True):  
        print('forward!')
        for _ in range(0, steps):
            self.set_step(1, 0, 0, 0)
            time.sleep(self.delay)
            self.set_step(0, 1, 0, 0)
            time.sleep(self.delay)
            self.set_step(0, 0, 1, 0)
            time.sleep(self.delay)
            self.set_step(0, 0, 0, 1)
            time.sleep(self.delay)
            if change_pos:
                self.__position+=1/self.steps_per_degree
        
    def backward(self,steps, change_pos=True):  
        print('backward!')
        for _ in range(0, steps):
            self.set_step(1, 0, 0, 1)
            time.sleep(self.delay)
            self.set_step(0, 1, 0, 1)
            time.sleep(self.delay)
            self.set_step(0, 1, 1, 0)
            time.sleep(self.delay)
            self.set_step(1, 0, 1, 0)
            time.sleep(self.delay)
            if change_pos:
                self.__position-=1/self.steps_per_degree
        
    def set_step(self,w1, w2, w3, w4):
        GPIO.output(self.pins[0], w1)
        GPIO.output(self.pins[1], w2)
        GPIO.output(self.pins[2], w3)
        GPIO.output(self.pins[3], w4)
        

class Goniometer():
    def __init__(self, i, e,controller):
        self.e_motor=Motor('Emission',[21,20,16,12], None,93.4615384615, .001)
        self.i_motor=Motor('Incidence',[24,23,25,18],None,93.4615384615,.001)
        self.tray_motor=Motor('Sample tray',[17,5,22,4],0,8.88888888888888888888889,.001)
        self.controller=controller
        self.tray_dir='forward'
        
    def configure(self, i, e, tray_pos):
        print(tray_pos)
        if tray_pos=='wr' or tray_pos=='-1':
            tray_pos=0
        elif tray_pos=='one' or tray_pos=='0':
            tray_pos=-60
        elif tray_pos=='two' or tray_pos=='1':
            tray_pos=-120
        elif tray_pos=='three' or tray_pos=='2':
            tray_pos=-180
        elif tray_pos=='four' or tray_pos=='3':
            tray_pos=-240
        elif tray_pos=='five' or tray_pos=='4':
            tray_pos=-300
        self.e_motor=Motor('Emission',[21,20,16,12], e,93.4615384615, .001)
        self.i_motor=Motor('Incidence',[24,23,25,18],i,93.4615384615,.001)
        self.tray_motor=Motor('Sample tray',[17,5,22,4],tray_pos,8.88888888888888888888889,.001)
    
    def set_incidence(self,theta):
        if self.i_motor.position==None:
            return
            filename=self.controller.encrypt('nopiconfig')
            self.controller.send(filename)
        else:
            self.i_motor.position=theta

    def set_emission(self,theta):
        if self.e_motor.position==None:
            return
            filename=self.controller.encrypt('nopiconfig')
            self.controller.send(filename)
        else:
            self.e_motor.position=theta
        
    def move_sample(self,pos):

        if pos=='wr':
            self.tray_motor.position=0
        elif pos=='one':
            self.tray_motor.position=-60
        elif pos=='two':
            self.tray_motor.position=-120
        elif pos=='three':
            self.tray_motor.position=-180
        elif pos=='four':
            self.tray_motor.position=-240
        elif pos=='five':
            self.tray_motor.position=-300
        # if self.tray_dir=='forward':
        #     self.tray_dir='backward'
        #     self.tray_motor.position=self.tray_motor.position+180
        # elif self.tray_dir=='backward':
        #     self.tray_dir='forward'
        #     self.tray_motor.position=self.tray_motor.position-180
        

if __name__=='__main__':
    main()
    
        
